# BALL LAP 一圈后附加直行设计

## 目标

将现有 `RUN_MODE_BALL_LAP` 的完成条件从“完成一圈后停车”改为“完成一圈后沿直道继续行驶 1.00 m，再减速停车”。一圈仍以现有跑道状态机完成第二个半圆作为判定点，附加 1 m 不计入一圈阶段本身。

## 范围

- 修改 `Control/control.c` 中 `BALL LAP` 的启动入口。
- 在 `Control/straight_turn_test.h` 暴露 BALL LAP 的附加距离常量，避免在控制逻辑中写入裸 `1.00f`。
- 复用 `straight_turn_test` 已有的 `POST_LAP`、编码器里程累计和 `BRAKING` 状态。
- 保持 `ONE LAP`、`BALL HOLD LAP`、`BALL STATIC` 及其他模式行为不变。
- 不修改 SysConfig、生成配置文件、TI DriverLib 或硬件驱动。

## 运行流程

```text
BALL LAP 启动
    -> STRAIGHT_1 / ARC_1 / STRAIGHT_2 / ARC_2
    -> 第二个半圆完成，锁存一圈时间
    -> POST_LAP，沿当前直道方向循迹并累计编码器里程
    -> 附加里程达到 1.00 m
    -> BRAKING，平滑减速
    -> DONE，置 Flag_Stop 停车
```

附加里程从第二个半圆完成的时刻清零，使用左右轮当前速度的平均值乘以 5 ms 控制周期累计，并限制单周期增量，沿用现有的编码器计量和直道循迹逻辑。

## 接口与数据流

`Control/control.c` 的按键启动分支根据运行模式选择启动接口：

- `RUN_MODE_BALL_LAP`：调用 `StraightTurnTest_StartWithPostLap()`，传入 BALL 速度、BALL 加速度和 `STRAIGHT_TURN_BALL_POST_LAP_DISTANCE_M`。
- `RUN_MODE_STRAIGHT_TURN`：继续调用 `StraightTurnTest_Start()`，附加距离为 0，保持原有一圈完成行为。

`StraightTurnTest_StartWithPostLap()` 已负责保存附加目标距离。第二个半圆结束后，现有 `straight_turn_run_arc()` 将状态切换到 `STRAIGHT_TURN_POST_LAP`；达到目标后进入 `STRAIGHT_TURN_BRAKING`，最终置 `Flag_Stop`。

## 故障与边界

- 红外持续丢线、IMU 失效和附加段超时继续使用现有故障停车路径。
- 附加距离由现有接口限幅到 0～2 m，因此 1.00 m 在有效范围内。
- `BALL LAP` 使用附加距离接口后，现有实现会跳过仅在“无附加段”时启用的 40 s 总流程超时，并使用附加段 15 s 看门狗；这是 `StartWithPostLap()` 当前约定的一部分。
- 一圈时间仍由 `StraightTurnLapTimeMs` 在第二个半圆完成时锁存，附加 1 m 和制动时间不改变该数据。

## 验证

1. 检查 `BALL LAP` 启动调用传入 1.00 m，`ONE LAP` 仍传入 0 m。
2. 执行 Keil `MSPM0G3507_Project` 构建，确认无新增编译错误或警告。
3. 静态检查 `POST_LAP -> BRAKING -> DONE` 路径及 `Flag_Stop` 行为。
4. 上板验证：完成一圈后车辆继续循迹约 1 m，随后平滑停车；确认普通 `ONE LAP` 仍在一圈结束后停车。
