# 小球快速定点与提前制动控制设计

## 最终结构修订：位置 PI 主控与连续距离限速

多轮上板结果表明，锁存制动、目标带外爬行下限和分段反向制动力限制会让
速度目标被状态机接管，产生“强制动后完全停住再启动”。本节是最终批准的
控制结构，取代本文后续保留的历史制动方案；后续历史内容只用于说明调试
过程，不再作为实现依据。

位置 PI 每个控制周期持续生成基础目标速度：

```text
v_pid =
    reference_velocity
    + Kp_position * position_error
    + Ki_position * position_integral
```

固定位置目标且 `abs(position_error) > 5 mm` 时，根据当前剩余距离实时
计算连续速度上限：

```text
d = abs(position_error)
a_profile = 30 mm/s^2
T_delay = 0.060 s
aT = a_profile * T_delay

v_distance =
    sqrt(aT * aT + 2 * a_profile * d) - aT

v_target =
    sign(v_pid) * min(abs(v_pid), v_distance)
```

该公式由 `d = v * T_delay + v^2 / (2 * a_profile)` 解出。它不是预先按
时间生成的平滑轨迹，而是每 5 ms 根据实际位置重新计算的状态反馈上限。
当前估计速度不再用于切换制动状态，而是始终由速度环直接比较：

```text
velocity_error = v_target - estimated_velocity
u_velocity = Kp_velocity * velocity_error
```

因此速度高于距离允许值时速度环自动制动；速度低于目标时控制量会在速度
降到零前连续变为正向，不需要锁存、爬行阶段或重启阶段。

最后 `5 mm` 内不应用距离上限，直接交给位置 PI。这样保留位置积分对水管
局部坡度和前馈残差的修正能力。距离上限生效时冻结积分候选值作为抗积分
饱和；退出限速后恢复原有积分规则。

第一轮上板保持以下参数不变：

```text
Kp_position = 2.0
Ki_position = 0.05
Kp_velocity = 4.7
Kd_acceleration = 0.12
velocity_max = 150 mm/s
```

新增参数与诊断量：

```text
BALL_BALANCE_DISTANCE_PROFILE_ACCEL_MM_S2 = 30.0
BALL_BALANCE_DISTANCE_LIMIT_MIN_ERROR_MM = 5.0
ball_balance_position_pid_velocity_mm_s
ball_balance_distance_velocity_limit_mm_s
ball_balance_distance_limited
```

删除运行路径中的制动锁存、停止距离进入判定、软制动目标、爬行下限和
对应状态。位置前馈、车辆加速度前馈、视觉失效保护、输出限幅和 5 us
量化保持不变。

## 背景

竞赛 H 题对小球控制包含两类工况：

1. 小车静止时，小球需要在 5 s 内完成 `0 mm -> +50 mm -> -50 mm`，
   并将两个端点的最大误差绝对值控制在 10 mm 以内。
2. 小车行驶时，小球需要稳定在中心或任意指定位置附近，误差绝对值
   不超过 10 mm。

摆杆由开槽 PPR 水管制成。水管沿长度方向存在小幅、可重复的位置相关
坡度，因此保持小球静止所需的舵机脉宽不是常数。现有分段位置前馈
`FF(x)` 必须保留，用于补偿这部分空间变化。

摄像头发送位置与帧差速度。MSPM0 端已经使用固定 5 ms 的 alpha-beta
观测器，以位置测量校正估计位置和速度，并由估计速度得到滤波加速度。
控制器已经采用位置 PI 外环和速度 P、加速度阻尼内环。

## 当前问题

### 平衡控制范围已与舵机控制范围统一

当前代码存在两级范围：

- 舵机驱动底层范围：`500..2500 us`；
- 手动、开环调节和 `ball_balance` 闭环控制范围：`500..2200 us`。

本设计使用已经确认的 `500..2200 us` 闭环范围，不继续扩大到底层允许的
`500..2500 us`。实施时 `ball_balance` 应复用 `SERVO_CONTROL_MIN_PULSE_US`
和 `SERVO_CONTROL_MAX_PULSE_US`，不再维护另一组相同数值的独立常量。
同时增加未限幅输出和饱和状态诊断。

### 正向到达判定允许提前折返

现有正向状态机在位置进入 `+50 +/- 10 mm`、速度绝对值不超过
`15.9 mm/s` 后进入保持状态。保持期间 PID 目标仍是 `+50 mm`，但是上述
条件只要连续保持 100 ms，任务目标就切换为 `-50 mm`。因此 PID 并未在
`+40 mm` 停止工作，但任务状态机允许小球未精确到达 `+50 mm` 就折返。

### 现有制动速度限制在目标行程内基本不生效

位置外环默认产生：

```text
v_position = 2.0 * position_error
```

现有制动上限为：

```text
v_brake = sqrt(2 * 180 * abs(position_error))
```

在 `0 -> +50 mm` 的起点，二者分别为 `100 mm/s` 和约 `134 mm/s`，
所以制动上限没有限制位置外环。在 `+50 -> -50 mm` 的主要行程内，
目标速度限幅和位置外环也先于该制动上限起作用。当前过冲主要依靠速度
反馈和加速度阻尼事后抑制，没有显式补偿相机、观测和舵机延迟造成的
额外停止距离。

## 设计目标

- 保留远离目标时的快速响应，不为整个行程生成慢速平滑位置轨迹。
- 根据估计速度、剩余距离和系统延迟，在接近目标时及时制动。
- 不新增位置误差 D。继续使用观测器速度提供位置微分阻尼。
- 不新增独立舵机偏置积分。继续使用现有位置积分消除静差，避免双积分
  相互干扰。
- 保留位置前馈、车辆加速度前馈、固定周期观测器、抗积分饱和和视觉
  失效保护。
- 使端点判定与精细定位目标一致，避免状态机提前折返。
- 增加足以区分前馈误差、输出饱和、制动过晚和观测延迟的诊断量。

## 非目标

- 不修改摄像头硬件、摆杆结构或舵机机构。
- 不引入卡尔曼滤波、MPC 或新的完整动力学模型。
- 不继续把舵机闭环脉宽范围扩大到 `500..2500 us`。
- 不在第一阶段实现分位置在线自学习前馈。
- 不直接使用摄像头帧差速度计算新的位置 D 项。

## 控制结构

### 状态与误差

控制器每 5 ms 使用同一份观测器快照：

```text
x = estimated_position_mm
v = estimated_velocity_mm_s
a = estimated_acceleration_mm_s2
e = target_position_mm - x
```

位置前馈继续由当前估计位置计算：

```text
u_ff = FF(x)
```

这样前馈描述的是小球当前所在水管位置的局部平衡脉宽，而不是只描述
目标点。

### 位置 PI

位置 PI 保持现有结构：

```text
integral = clamp(integral + e * Ts)
v_pi = reference_velocity + Kp_position * e + Ki_position * integral
```

继续保留积分限幅和目标速度饱和时的抗积分饱和。第一阶段不增加任何
第二积分器。

### 基于实际速度的提前制动

只在小球正在朝目标运动时计算停止距离：

```text
v_toward = abs(v)
d_dynamic = v_toward * v_toward / (2 * a_stop)
d_delay = v_toward * T_delay
d_stop = d_dynamic + d_delay + d_margin
```

朝向目标的判定为：

```text
e * v > 0
```

其中：

- `a_stop` 是实测可实现的保守制动加速度；
- `T_delay` 汇总摄像头、串口、观测和舵机的有效延迟；
- `d_margin` 是应对位置噪声、局部水管坡度和参数误差的安全距离。

由剩余可制动距离计算安全速度：

```text
d_available =
    max(abs(e) - v_toward * T_delay - d_margin, 0)
v_safe = sqrt(2 * a_stop * d_available)
```

制动阶段进入判定为：

```text
braking_entry_required =
    e * v > 0
    and abs(v) > v_release
    and abs(e) <= d_stop
```

`braking_entry_required` 只负责进入制动阶段。进入后锁存
`soft_braking_active`，不再因为单帧的停止距离或速度变化立即退出，避免
位置 PI 重新加速后再次进入制动。锁存只属于当前固定位置目标；目标改变、
视觉失效、参考速度变为非零或小球已经反向远离目标时立即清除。

上板结果表明，以当前实际速度为基准生成目标会把位置 PI 原本已经给出的
制动量抵消掉：`0 -> +50 mm` 过冲约 10 mm，`-50 -> +50 mm` 过冲约
25 mm，并在越过目标后反向回退。因此锁存制动阶段改为以位置 PI 为主制动
基线，安全速度超差只增加制动量，同时限制单帧允许的最大制动速度误差：

```text
if soft_braking_active:
    speed_excess = max(abs(v) - v_safe, 0)
    aggressive_target =
        max(abs(v_pi_limited)
            - braking_excess_gain * speed_excess,
            0)

    if abs(e) > settle_position:
        creep_floor =
            min(abs(v_pi_limited), braking_creep_velocity)
        target_magnitude =
            clamp(
                aggressive_target,
                creep_floor,
                max(abs(v), creep_floor))
    else:
        max_brake_delta =
            min(max_braking_speed_fraction * abs(v),
                brake_velocity_error_max)
        minimum_target =
            max(abs(v) - max_brake_delta, 0)
        target_magnitude =
            clamp(
                aggressive_target,
                minimum_target,
                abs(v))

    v_target = sign(e) * target_magnitude
else:
    v_target = v_pi_limited
```

其中 `v_pi_limited = clamp(v_pi, -velocity_limit, velocity_limit)`。
目标带外的 `aggressive_target` 恢复最初版本的位置 PI 强制动，并将
`braking_excess_gain` 提高到 `0.5`。`creep_floor` 保证长行程即使提前
降到低速也不会完全停住；上限 `max(abs(v), creep_floor)` 禁止恢复远端
高速加速，但允许从静止连续加速到爬行速度。进入最后 5 mm 后才启用
`minimum_target`，将速度内环反向误差限制为当前速度的一定比例和绝对
上限，保留已经验证有效的防回退特性。

若小球在目标带外提前降到低速，锁存制动阶段不退出，而是以不超过
`braking_creep_velocity` 的速度继续靠近，避免恢复远端位置 PI 后再次
高速起步。只有同时满足 `abs(e) <= settle_position` 和
`abs(v) <= v_release` 时，才退出锁存并交还位置 PI 完成最终精确定位。

制动锁存期间冻结现有位置积分，防止长行程剩余位置误差继续累积并在退出
制动后造成二次加速。该设计不增加位置 D，也不增加第二积分器；远离目标
时仍使用原位置 PI 快速响应，只在停止距离内改变速度目标生成方式。

初始上板参数使用保守值，并通过单变量试验调整：

```text
a_stop = 150 mm/s^2
T_delay = 0.060 s
d_margin = 3 mm
v_release = 10 mm/s
braking_excess_gain = 0.5
max_braking_speed_fraction = 0.6
brake_velocity_error_max = 50 mm/s
braking_creep_velocity = 25 mm/s
settle_position = 5 mm
```

以 `e=20 mm`、`v=80 mm/s`、`v_safe` 约为 `60 mm/s` 为例，
位置 PI 目标约为 `40 mm/s`，最终 `v_target` 约为 `30.249 mm/s`，速度
内环制动误差约为 `-49.751 mm/s`；以
`e=4 mm`、`v=20 mm/s`、`v_safe=0` 为例，`v_target=8 mm/s`，
制动误差被限制为 `-12 mm/s`。前者恢复长行程
高速制动力，后者防止末端目标速度被压得过低而产生反向回退。

这些值对应的不是最终标定结果。第一次上板保持 `a_stop`、`T_delay`、
`d_margin` 和现有 PID 参数不变，只替换制动目标计算和锁存逻辑，避免
同时改变多个变量。

PI 主制动加全程最大制动力限制的版本上板后，反向回退和二次往返基本
消失，但制动体感明显偏软；将 `a_stop` 降至 `120 mm/s^2` 只让柔软减速
更早开始，并没有恢复最初版本快速制动的效果。因此最终混合方案恢复
`a_stop=150 mm/s^2` 和目标带外强制动，只在最后 5 mm 保留最大反向
制动力限制，并用锁存爬行解决长行程提前停住后重新高速加速的问题。

混合强制动版本恢复了快速制动，但 `10 mm/s` 的目标带外最低速度仍会让
小球完全停住再启动。按 `a_stop * T_delay = 150 * 0.060 = 9 mm/s`
估算，撤掉强制动后仍可能损失约 `9 mm/s`，原来的速度裕量不足。因此将
目标带外最低连续靠近速度先提高到 `20 mm/s`，随后按单变量上板反馈继续
放宽到 `25 mm/s`；锁存解除和任务到达速度
阈值仍保持 `10 mm/s`。这样只提前撤掉低速阶段的反向制动力，不削弱
高速强制动，也不放宽最终定位标准。

### 速度阻尼与加速度阻尼

内环保持：

```text
velocity_error = v_target - v
u_velocity = Kp_velocity * velocity_error
u_acceleration = -Kd_velocity * a
```

固定位置目标下，`d(e)/dt = -v`，所以 `-v` 已经是位置误差的微分反馈。
新增位置 D 只会重复增加速度阻尼，并再次依赖噪声较大的速度信息。

若过冲仍然明显，首先小幅增加 `Kp_velocity` 或减小 `Kp_position`。每次
调整幅度不超过当前值的 20%，并保持其他增益不变。只有在估计加速度足够
平滑、速度阻尼已经调好后，才恢复或增加小幅加速度阻尼。

### 舵机输出

最终输出保持：

```text
u_unsaturated =
    u_ff
    + servo_direction * (
        u_velocity
        + u_acceleration
        + u_vehicle_feedforward)

u_servo = quantize(clamp(u_unsaturated, 500 us, 2200 us), 5 us)
```

新增只读诊断量：

- `u_unsaturated`；
- `u_servo`；
- `output_saturated`；
- `d_stop`；
- `v_safe`；
- `braking_active`。

## 静态任务状态机

### 正向端点

正向到达条件改为：

```text
abs(x - 50 mm) <= 5 mm
abs(v) <= 10 mm/s
```

连续满足 100 ms 后才允许切换到负向目标。PID 在这 100 ms 内继续以
`+50 mm` 为目标。若位置或速度离开条件范围，计时立即清零并回到正向移动
状态。

### 负向端点

负向最终稳定条件保持同样的精度和速度阈值：

```text
abs(x + 50 mm) <= 5 mm
abs(v) <= 10 mm/s
```

连续满足 300 ms 后完成任务。正负端点使用相同位置和速度标准，只有保持
时间因“折返”和“最终稳定”的不同而保持 100 ms 与 300 ms 的区别。

### 超调记录

端点误差不能只由最终稳定位置计算。状态机分别记录：

- 正向过程最大位置；
- 负向过程最小位置；
- 正向峰值误差；
- 负向峰值误差；
- 首次进入目标带时间；
- 连续稳定完成时间。

这些量用于区分“到达很快但过冲大”和“没有过冲但响应慢”。

## 调参顺序

每个阶段至少重复同一工况 5 次，只改变一个参数组。

1. 确认位置、速度、舵机方向正确，并记录闭环实际是否触及
   `500/2200 us`。
2. 暂时令位置 `Ki = 0`、加速度阻尼 `Kd_velocity = 0`，只调
   `Kp_position` 和 `Kp_velocity`。
3. 保持远端响应速度，逐步增强速度阻尼，直到无持续振荡。
4. 启用锁存柔和制动，先保持停止距离参数不变。若长行程高速阶段制动力
   不足，先增加 `braking_excess_gain`；若低速末端仍有反向回退，
   先减小 `max_braking_speed_fraction` 或
   `brake_velocity_error_max`。一次只修改一个值。
5. 柔和制动力确定后，再调 `a_stop`、`T_delay` 和 `d_margin`。
   过冲大但制动力已经合适时，减小 `a_stop`、增大 `T_delay` 或增大
   `d_margin`，让减速更早开始；制动过早时反向调整。
6. 恢复小幅位置 `Ki`，只增加到能够消除稳定静差，不以积分承担快速
   运动控制。
7. 若速度已经稳定但仍有高频小幅摆动，再恢复小幅加速度阻尼。
8. 分别验证静态往返、中心保持、车辆启动、匀速、转弯和制动工况。

## 异常与边界处理

- 视觉状态无效或超过 200 ms 未更新：清除位置积分和动态控制状态，
  舵机回中位。
- `a_stop <= 0`：配置拒绝生效，继续使用上一个有效值。
- 停止距离计算前限制速度和位置范围，避免异常测量产生过大结果。
- 输出饱和时沿用现有抗积分饱和，禁止积分继续推动同方向饱和。
- 锁存柔和制动期间冻结位置积分，退出后再恢复原有积分更新规则。
- 目标切换时保留观测器状态，清除端点稳定计时；位置积分是否保留沿用
  当前控制器启停语义，不因正常 `+50 -> -50 mm` 切换而额外清零。
- 小球正在远离目标时清除柔和制动锁存，避免 `v_safe` 阻碍快速回正。

## 验证

### 主机测试

新增纯函数测试提前制动计算：

- 静止时停止距离等于安全余量；
- 速度增加时停止距离单调增加；
- 延迟增加时停止距离单调增加；
- 正在远离目标时不启用制动速度限制；
- 剩余距离小于延迟距离和安全余量时，安全速度为零；
- 最后 5 mm 内制动目标的反向速度误差不超过当前速度比例和绝对上限；
- 低速且仍在目标带外时只给出不超过 10 mm/s 的靠近目标速度；
- 异常和边界输入不会产生 NaN、无穷或负平方根。

扩展控制器测试：

- 远离目标且速度低时保持原有快速位置 PI 输出；
- 高速接近目标且停止距离达到剩余距离时，`braking_active` 锁存；
- `e=20 mm`、`v=80 mm/s` 时目标速度约为 `30.249 mm/s`，恢复最初
  版本的位置 PI 强制动；
- `e=4 mm`、`v=20 mm/s`、`v_safe=0` 时目标速度为 `8 mm/s`，
  最大反向速度误差限制为 `12 mm/s`；
- 瞬时离开制动进入条件不会恢复远端位置 PI 并再次加速；
- 进入目标带且低于释放速度后解除锁存并恢复位置 PI；
- 制动锁存期间位置积分不继续累积；
- 不新增位置 D 项；
- 输出仍限制在 `500..2200 us` 并按 5 us 量化；
- 视觉失效时输出中位；
- 输出饱和时积分不继续向饱和方向累积。

扩展状态机测试：

- `+40 mm` 不再满足正向到达条件；
- `+45..+55 mm` 且低速连续 100 ms 才允许折返；
- 位置在目标带内但速度过高时不折返；
- 离开目标带会清除稳定计时；
- 负向端点连续稳定 300 ms 才完成。

### 完整构建

- 所有主机测试无警告通过；
- `git diff --check` 无格式错误；
- Keil `MSPM0G3507_Project` 构建零错误、零新增警告。

### 上板验收

静态任务连续测试至少 5 次，记录：

- 总时间不超过 5 s；
- 相比当前可接受基线，100 mm 长行程到达时间增加不超过 0.2 s；
- 正负端点峰值误差绝对值不超过 10 mm；
- 内部调试目标为端点稳定误差不超过 5 mm；
- 100 mm 长行程最后 5 mm 单调靠近目标，无反向回退和二次高速加速；
- 无持续振荡；
- 每次过冲方向和幅度可重复解释；
- 输出饱和比例、最大速度、开始制动位置和稳定时间。

车辆工况分别测试启动、直线、转弯和制动：

- 中心或指定位置误差绝对值不超过 10 mm；
- 视觉短时帧间隔内输出连续；
- 视觉超时后舵机安全回中；
- 提前制动逻辑不会在小球远离目标时妨碍回正。

## 后续升级条件

只有完成上述设计并取得日志后，才评估以下升级：

1. 若不同位置仍存在稳定、可重复的残余静差，扩展前馈表到整个可用位置
   范围，或增加离线标定点。
2. 若同一位置的平衡脉宽随时间缓慢变化，且现有位置积分不能消除，再设计
   仅在低速稳定状态更新、运动和饱和时冻结的偏置估计器。
3. 若估计速度噪声仍限制阻尼增益，优先重新调整观测器和摄像头时间戳，
   而不是新增位置误差微分。
