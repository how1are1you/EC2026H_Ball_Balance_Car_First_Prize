# Report Flowcharts Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 生成两张符合当前固件逻辑、可直接插入 A4 竖版报告的纯黑白流程图 PNG。

**Architecture:** 使用一个独立的 Pillow 绘图程序统一管理字体、标准流程图形、箭头和页面尺寸。控制框图与主程序流程图分别由独立函数生成，最后读取成品图片检查中文字、连线和边界。

**Tech Stack:** Python 3、Pillow、微软雅黑字体、PNG。

## Global Constraints

- 输出为竖向白底黑线图。
- 仅使用黑、白两色，不使用阴影、渐变或彩色。
- 中文字体使用 `C:\Windows\Fonts\msyh.ttc`。
- 图片宽度为 1800 px，适合缩放插入 A4 正文。
- 图中逻辑必须对应当前 `empty.c`、`Control/control.c` 和 `Control/ball_balance.c`。
- 不修改现有固件源代码和用户已有未提交文件。

---

### Task 1: 统一流程图绘图工具与滚球控制框图

**Files:**
- Create: `tools/generate_report_flowcharts.py`
- Create: `docs/figures/rolling_ball_control_block.png`

**Interfaces:**
- Consumes: 当前滚球串级控制的数据流和控制关系。
- Produces: `draw_rolling_ball_control_diagram(output_path: str) -> None`。

- [ ] **Step 1: 建立页面和标准图形绘制函数**

实现居中多行文本、矩形、圆形求和点、箭头、虚线反馈箭头及页边距检查。

- [ ] **Step 2: 绘制滚球串级闭环**

主通道依次绘制目标位置、位置比较、位置 PI、速度限幅、速度比较、速度 PD、前馈叠加、PWM 限幅与量化、舵机摆杆和滚球运动；右侧绘制视觉测量与滤波，反馈到两个比较点。

- [ ] **Step 3: 生成第一张 PNG**

Run:

```powershell
rtk proxy "C:\Users\zhuzhichao\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" tools\generate_report_flowcharts.py --diagram ball
```

Expected: `docs/figures/rolling_ball_control_block.png` 存在且宽度为 1800 px。

- [ ] **Step 4: 检查第一张图片**

用图片查看工具确认所有中文完整、反馈箭头分别指向位置环和速度环、任何图形均未越过页面。

### Task 2: 主程序流程图

**Files:**
- Modify: `tools/generate_report_flowcharts.py`
- Create: `docs/figures/main_program_flowchart.png`

**Interfaces:**
- Consumes: `main()` 初始化流程、主循环和 5 ms 定时中断流程。
- Produces: `draw_main_program_flowchart(output_path: str) -> None`。

- [ ] **Step 1: 绘制主程序主线**

依次绘制上电、外设及控制模块初始化、中断使能、OLED 和 MPU6050 初始化、IMU 成功判断、10 秒稳定等待以及主循环。

- [ ] **Step 2: 绘制 5 ms 中断子流程**

在主循环下方独立绘制定时中断入口、按键和编码器处理、模式复位、滚球控制、静态/停车判断、轨迹状态机、电机 PI、PWM 输出和中断返回。

- [ ] **Step 3: 生成第二张 PNG**

Run:

```powershell
rtk proxy "C:\Users\zhuzhichao\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" tools\generate_report_flowcharts.py --diagram main
```

Expected: `docs/figures/main_program_flowchart.png` 存在且宽度为 1800 px。

- [ ] **Step 4: 检查第二张图片**

确认判断菱形的“是/否”方向正确，静态模式和停车状态均通向零 PWM，主循环回边与定时中断边不相互遮挡。

### Task 3: 最终生成与质量验证

**Files:**
- Modify: `tools/generate_report_flowcharts.py`
- Verify: `docs/figures/rolling_ball_control_block.png`
- Verify: `docs/figures/main_program_flowchart.png`

**Interfaces:**
- Consumes: 两个绘图函数。
- Produces: `--diagram all` 命令和最终成品。

- [ ] **Step 1: 一次生成全部图片**

Run:

```powershell
rtk proxy "C:\Users\zhuzhichao\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" tools\generate_report_flowcharts.py --diagram all
```

Expected: 命令退出码为 0，两张 PNG 均被刷新。

- [ ] **Step 2: 验证图片属性**

读取两张图片，验证格式为 PNG、宽度为 1800 px、高度大于宽度，并检查最外层非白像素没有贴近页面边缘。

- [ ] **Step 3: 最终视觉检查**

逐张查看原始分辨率图片，确认黑白打印效果、中文可读性、节点间距和箭头终点。
