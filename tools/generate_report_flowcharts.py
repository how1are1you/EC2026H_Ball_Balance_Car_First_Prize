from __future__ import annotations

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "docs" / "figures"
FONT_PATH = Path(r"C:\Windows\Fonts\msyh.ttc")
FONT_BOLD_PATH = Path(r"C:\Windows\Fonts\msyhbd.ttc")

BLACK = "black"
WHITE = "white"
LINE_WIDTH = 5


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = FONT_BOLD_PATH if bold else FONT_PATH
    return ImageFont.truetype(str(path), size=size)


def centered_text(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    text: str,
    text_font: ImageFont.FreeTypeFont,
    spacing: int = 8,
) -> None:
    left, top, right, bottom = box
    bounds = draw.multiline_textbbox(
        (0, 0),
        text,
        font=text_font,
        spacing=spacing,
        align="center",
    )
    width = bounds[2] - bounds[0]
    height = bounds[3] - bounds[1]
    x = left + (right - left - width) / 2
    y = top + (bottom - top - height) / 2 - bounds[1]
    draw.multiline_text(
        (x, y),
        text,
        fill=BLACK,
        font=text_font,
        spacing=spacing,
        align="center",
    )


def process_box(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    text: str,
    text_font: ImageFont.FreeTypeFont,
) -> None:
    draw.rectangle(box, outline=BLACK, fill=WHITE, width=LINE_WIDTH)
    centered_text(draw, box, text, text_font)


def input_output_box(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    text: str,
    text_font: ImageFont.FreeTypeFont,
) -> None:
    left, top, right, bottom = box
    skew = min(55, (right - left) // 8)
    points = [
        (left + skew, top),
        (right, top),
        (right - skew, bottom),
        (left, bottom),
    ]
    draw.polygon(points, outline=BLACK, fill=WHITE)
    draw.line(points + [points[0]], fill=BLACK, width=LINE_WIDTH, joint="curve")
    centered_text(draw, box, text, text_font)


def terminator(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    text: str,
    text_font: ImageFont.FreeTypeFont,
) -> None:
    draw.ellipse(box, outline=BLACK, fill=WHITE, width=LINE_WIDTH)
    centered_text(draw, box, text, text_font)


def decision(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    text: str,
    text_font: ImageFont.FreeTypeFont,
) -> None:
    left, top, right, bottom = box
    cx = (left + right) // 2
    cy = (top + bottom) // 2
    points = [(cx, top), (right, cy), (cx, bottom), (left, cy)]
    draw.polygon(points, outline=BLACK, fill=WHITE)
    draw.line(points + [points[0]], fill=BLACK, width=LINE_WIDTH, joint="curve")
    centered_text(
        draw,
        (left + 45, top + 20, right - 45, bottom - 20),
        text,
        text_font,
    )


def summing_point(
    draw: ImageDraw.ImageDraw,
    center: tuple[int, int],
    radius: int,
    positive_label: str = "+",
    negative_label: str = "−",
) -> None:
    cx, cy = center
    draw.ellipse(
        (cx - radius, cy - radius, cx + radius, cy + radius),
        outline=BLACK,
        fill=WHITE,
        width=LINE_WIDTH,
    )
    centered_text(
        draw,
        (cx - radius, cy - radius, cx + radius, cy + radius),
        "Σ",
        font(42, bold=True),
    )
    draw.text(
        (cx - 22, cy - radius - 38),
        positive_label,
        fill=BLACK,
        font=font(30, bold=True),
    )
    draw.text(
        (cx + radius + 10, cy - 22),
        negative_label,
        fill=BLACK,
        font=font(30, bold=True),
    )


def arrow_head(
    draw: ImageDraw.ImageDraw,
    start: tuple[int, int],
    end: tuple[int, int],
    size: int = 20,
) -> None:
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    left = (
        end[0] - size * math.cos(angle - math.pi / 6),
        end[1] - size * math.sin(angle - math.pi / 6),
    )
    right = (
        end[0] - size * math.cos(angle + math.pi / 6),
        end[1] - size * math.sin(angle + math.pi / 6),
    )
    draw.polygon([end, left, right], fill=BLACK)


def arrow(
    draw: ImageDraw.ImageDraw,
    points: list[tuple[int, int]],
    width: int = LINE_WIDTH,
) -> None:
    draw.line(points, fill=BLACK, width=width, joint="curve")
    arrow_head(draw, points[-2], points[-1])


def dashed_segment(
    draw: ImageDraw.ImageDraw,
    start: tuple[int, int],
    end: tuple[int, int],
    dash: int = 18,
    gap: int = 12,
    width: int = 4,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return
    ux = dx / length
    uy = dy / length
    cursor = 0.0
    while cursor < length:
        segment_end = min(cursor + dash, length)
        p1 = (start[0] + ux * cursor, start[1] + uy * cursor)
        p2 = (start[0] + ux * segment_end, start[1] + uy * segment_end)
        draw.line((p1, p2), fill=BLACK, width=width)
        cursor += dash + gap


def dashed_arrow(
    draw: ImageDraw.ImageDraw,
    points: list[tuple[int, int]],
    width: int = 4,
) -> None:
    for start, end in zip(points, points[1:]):
        dashed_segment(draw, start, end, width=width)
    arrow_head(draw, points[-2], points[-1])


def line_label(
    draw: ImageDraw.ImageDraw,
    position: tuple[int, int],
    text: str,
    bold: bool = False,
    anchor: str = "mm",
) -> None:
    draw.text(
        position,
        text,
        fill=BLACK,
        font=font(30, bold=bold),
        anchor=anchor,
    )


def save_image(image: Image.Image, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path, format="PNG", dpi=(300, 300), optimize=True)


def draw_rolling_ball_control_diagram(output_path: Path) -> None:
    width, height = 1800, 3100
    image = Image.new("RGB", (width, height), WHITE)
    draw = ImageDraw.Draw(image)

    main_left, main_right = 585, 1215
    cx = width // 2
    body_font = font(38)
    small_font = font(32)

    target_box = (650, 80, 1150, 205)
    input_output_box(draw, target_box, "目标位置  x*", body_font)

    sum_pos = (cx, 355)
    summing_point(draw, sum_pos, 58)
    arrow(draw, [(cx, 205), (cx, 297)])
    line_label(draw, (cx - 90, 252), "设定值")

    pos_pi = (main_left, 475, main_right, 635)
    process_box(
        draw,
        pos_pi,
        "位置 PI 外环\nKpp·e_x + Kpi·∫e_x dt",
        body_font,
    )
    arrow(draw, [(cx, 413), (cx, 475)])

    target_velocity = (main_left, 735, main_right, 925)
    process_box(
        draw,
        target_velocity,
        "目标速度生成\n参考速度叠加、限幅与制动约束",
        body_font,
    )
    arrow(draw, [(cx, 635), (cx, 735)])

    reference_box = (90, 755, 465, 905)
    input_output_box(draw, reference_box, "参考速度\nvref", body_font)
    arrow(draw, [(465, 830), (585, 830)])

    sum_vel = (cx, 1065)
    summing_point(draw, sum_vel, 58)
    arrow(draw, [(cx, 925), (cx, 1007)])

    vel_pd = (main_left, 1185, main_right, 1355)
    process_box(
        draw,
        vel_pd,
        "速度 PD 内环\nKvp·e_v + Kvd·de_v/dt",
        body_font,
    )
    arrow(draw, [(cx, 1123), (cx, 1185)])

    sum_control = (cx, 1505)
    summing_point(draw, sum_control, 58)
    arrow(draw, [(cx, 1355), (cx, 1447)])

    acceleration_box = (70, 1390, 445, 1535)
    input_output_box(draw, acceleration_box, "小车纵向加速度\nacar", small_font)
    feedforward_box = (490, 1390, 735, 1535)
    process_box(draw, feedforward_box, "加速度前馈\nKff·acar", small_font)
    arrow(draw, [(445, 1462), (490, 1462)])
    arrow(draw, [(735, 1462), (790, 1462), (790, 1505), (842, 1505)])

    pwm_box = (main_left, 1640, main_right, 1820)
    process_box(
        draw,
        pwm_box,
        "舵机脉宽限幅与量化\n800～1800 μs，步进 10 μs",
        body_font,
    )
    arrow(draw, [(cx, 1563), (cx, 1640)])

    servo_box = (main_left, 1930, main_right, 2085)
    process_box(draw, servo_box, "舵机执行机构\n改变摆杆倾角", body_font)
    arrow(draw, [(cx, 1820), (cx, 1930)])

    ball_box = (main_left, 2195, main_right, 2350)
    process_box(draw, ball_box, "摆杆—滚球被控对象", body_font)
    arrow(draw, [(cx, 2085), (cx, 2195)])

    output_box = (650, 2460, 1150, 2590)
    input_output_box(draw, output_box, "滚球位置 x、速度 v", body_font)
    arrow(draw, [(cx, 2350), (cx, 2460)])

    camera_box = (1215, 2385, 1690, 2545)
    process_box(draw, camera_box, "摄像头视觉测量", body_font)
    arrow(draw, [(1150, 2525), (1215, 2525)])

    filter_box = (1110, 2700, 1690, 2910)
    process_box(
        draw,
        filter_box,
        "UART1 帧解析\n单位换算与低通滤波",
        body_font,
    )
    arrow(draw, [(1452, 2545), (1452, 2700)])

    dashed_arrow(
        draw,
        [(1560, 2700), (1760, 2700), (1760, 355), (958, 355)],
    )
    line_label(draw, (1735, 610), "位置反馈  x_m", anchor="rm")

    dashed_arrow(
        draw,
        [(1310, 2700), (1710, 2700), (1710, 1065), (958, 1065)],
    )
    line_label(draw, (1685, 1220), "速度反馈  v_m", anchor="rm")

    note_box = (55, 2670, 465, 2925)
    process_box(
        draw,
        note_box,
        "视觉数据超过 200 ms\n未更新：进入 STALE\n状态，舵机回中",
        font(28),
    )
    dashed_arrow(
        draw,
        [(465, 2795), (520, 2795), (520, 1730), (585, 1730)],
    )

    line_label(draw, (cx, 3035), "实线：控制与执行通道    虚线：反馈及安全通道")
    save_image(image, output_path)


def draw_main_program_flowchart(output_path: Path) -> None:
    width, height = 1800, 3460
    image = Image.new("RGB", (width, height), WHITE)
    draw = ImageDraw.Draw(image)

    cx = width // 2
    body_font = font(34)
    small_font = font(29)
    label_font = font(35, bold=True)

    start = (700, 55, 1100, 155)
    terminator(draw, start, "上电 / 复位", body_font)

    syscfg = (560, 230, 1240, 345)
    process_box(draw, syscfg, "SYSCFG_DL_init()\n初始化时钟及外设", body_font)
    arrow(draw, [(cx, 155), (cx, 230)])

    control_init = (500, 425, 1300, 585)
    process_box(
        draw,
        control_init,
        "复位两路串口状态\n初始化舵机、滚球控制及任务状态",
        body_font,
    )
    arrow(draw, [(cx, 345), (cx, 425)])

    irq_init = (560, 665, 1240, 800)
    process_box(draw, irq_init, "清除并使能 GPIO、UART、\n5 ms 定时器和 ADC 中断", body_font)
    arrow(draw, [(cx, 585), (cx, 665)])

    oled_init = (620, 880, 1180, 990)
    process_box(draw, oled_init, "OLED_Init()", body_font)
    arrow(draw, [(cx, 800), (cx, 880)])

    imu_init = (620, 1070, 1180, 1180)
    process_box(draw, imu_init, "imu_init()", body_font)
    arrow(draw, [(cx, 990), (cx, 1070)])

    imu_decision = (690, 1250, 1110, 1435)
    decision(draw, imu_decision, "IMU 初始化\n成功？", body_font)
    arrow(draw, [(cx, 1180), (cx, 1250)])

    stabilize = (570, 1510, 1230, 1640)
    process_box(draw, stabilize, "等待 IMU 稳定 10 s\n期间持续 imu_service()", body_font)
    arrow(draw, [(cx, 1435), (cx, 1510)])
    line_label(draw, (945, 1470), "是", bold=True)

    skip = (1280, 1278, 1710, 1410)
    process_box(draw, skip, "跳过稳定等待\n保留故障状态", small_font)
    arrow(draw, [(1110, 1342), (1280, 1342)])
    line_label(draw, (1190, 1305), "否", bold=True)
    arrow(draw, [(1495, 1410), (1495, 1695), (cx, 1695)])
    arrow(draw, [(cx, 1640), (cx, 1695)])

    draw.line((90, 1760, 1710, 1760), fill=BLACK, width=3)
    draw.text((440, 1795), "主循环", fill=BLACK, font=label_font, anchor="mm")
    draw.text((1325, 1795), "5 ms 定时中断", fill=BLACK, font=label_font, anchor="mm")

    main_loop = (230, 1880, 650, 1985)
    terminator(draw, main_loop, "进入 while (1)", body_font)
    arrow(draw, [(cx, 1695), (440, 1695), (440, 1880)])

    imu_service = (145, 2070, 735, 2175)
    process_box(draw, imu_service, "imu_service()", body_font)
    arrow(draw, [(440, 1985), (440, 2070)])

    uart_service = (145, 2250, 735, 2355)
    process_box(draw, uart_service, "control_uart_service()", body_font)
    arrow(draw, [(440, 2175), (440, 2250)])

    static_service = (125, 2430, 755, 2550)
    process_box(draw, static_service, "ball_static_task_service()\n输出静态滚球测试数据", small_font)
    arrow(draw, [(440, 2355), (440, 2430)])

    voltage = (145, 2625, 735, 2730)
    input_output_box(draw, voltage, "采样电池电压", body_font)
    arrow(draw, [(440, 2550), (440, 2625)])

    oled_show = (145, 2805, 735, 2910)
    process_box(draw, oled_show, "oled_show()", body_font)
    arrow(draw, [(440, 2730), (440, 2805)])
    arrow(draw, [(440, 2910), (80, 2910), (80, 1932), (230, 1932)])

    irq_start = (1110, 1880, 1540, 1985)
    terminator(draw, irq_start, "TIMER_0 中断", body_font)
    dashed_arrow(
        draw,
        [(1240, 732), (1660, 732), (1660, 1932), (1540, 1932)],
    )
    line_label(draw, (1635, 1020), "每 5 ms 触发", anchor="rm")

    tick_key = (1020, 2060, 1630, 2180)
    process_box(draw, tick_key, "tick_ms += 5\n按键扫描、LED 状态更新", small_font)
    arrow(draw, [(1325, 1985), (1325, 2060)])

    encoder = (1020, 2250, 1630, 2370)
    process_box(draw, encoder, "读取编码器计数\n换算左右轮实际速度", small_font)
    arrow(draw, [(1325, 2180), (1325, 2250)])

    mode_task = (990, 2440, 1660, 2580)
    process_box(draw, mode_task, "模式变化时复位各状态机\n更新 BALL HOLD / STATIC 任务", small_font)
    arrow(draw, [(1325, 2370), (1325, 2440)])

    balance = (1010, 2650, 1640, 2770)
    process_box(draw, balance, "配置滚球控制使能与加速度前馈\n执行 ball_balance_update()", small_font)
    arrow(draw, [(1325, 2580), (1325, 2650)])

    stop_decision = (1110, 2840, 1540, 3010)
    decision(draw, stop_decision, "静态模式\n或停车标志？", small_font)
    arrow(draw, [(1325, 2770), (1325, 2840)])

    zero_pwm = (1510, 3050, 1770, 3190)
    process_box(draw, zero_pwm, "速度目标清零\n电机 PWM = 0", small_font)
    arrow(draw, [(1540, 2925), (1665, 2925), (1665, 3050)])
    line_label(draw, (1595, 2885), "是", bold=True)

    trajectory = (920, 3075, 1450, 3205)
    process_box(draw, trajectory, "按运行模式执行轨迹状态机\n生成左右轮目标速度", small_font)
    arrow(draw, [(1325, 3010), (1325, 3075)])
    line_label(draw, (1280, 3040), "否", bold=True)

    motor_pi = (885, 3270, 1475, 3390)
    process_box(draw, motor_pi, "左右轮增量式 PI\nPWM 限幅并输出到电机", small_font)
    arrow(draw, [(1185, 3205), (1185, 3270)])

    irq_return = (1510, 3270, 1770, 3390)
    terminator(draw, irq_return, "中断返回", small_font)
    arrow(draw, [(1475, 3330), (1510, 3330)])
    arrow(draw, [(1665, 3190), (1665, 3270)])

    save_image(image, output_path)


def validate_output(path: Path) -> None:
    with Image.open(path) as image:
        if image.format != "PNG":
            raise RuntimeError(f"{path} is not a PNG")
        if image.width != 1800 or image.height <= image.width:
            raise RuntimeError(f"Unexpected size for {path}: {image.size}")
        grayscale = image.convert("L")
        inverted = Image.eval(grayscale, lambda value: 255 - value)
        bounds = inverted.getbbox()
        if bounds is None:
            raise RuntimeError(f"{path} is blank")
        left, top, right, bottom = bounds
        if left < 20 or top < 20 or right > image.width - 20 or bottom > image.height - 20:
            raise RuntimeError(f"Content is too close to page edge in {path}: {bounds}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--diagram",
        choices=("ball", "main", "all"),
        default="all",
    )
    args = parser.parse_args()

    ball_path = OUTPUT_DIR / "rolling_ball_control_block.png"
    main_path = OUTPUT_DIR / "main_program_flowchart.png"

    generated: list[Path] = []
    if args.diagram in ("ball", "all"):
        draw_rolling_ball_control_diagram(ball_path)
        generated.append(ball_path)
    if args.diagram in ("main", "all"):
        draw_main_program_flowchart(main_path)
        generated.append(main_path)

    for path in generated:
        validate_output(path)
        with Image.open(path) as image:
            print(f"{path}: {image.width}x{image.height}, {image.format}")


if __name__ == "__main__":
    main()
