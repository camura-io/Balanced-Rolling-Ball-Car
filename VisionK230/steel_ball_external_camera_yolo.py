"""
程序用途：CanMV K230 外置摄像头钢球 YOLO 检测。
功能说明：选择 CSI0 外置摄像头，将画面显示到 LCD 和 CanMV IDE，
并逐帧运行 YOLO11 钢球检测模型。

实验名称：在线训练-YOLO图像检测: 基于摄像头
实验平台：01Studio CanMV K230/CanMV K230 mini
说明：可以通过display="xxx"参数选择"hdmi"、"lcd3_5"(3.5寸mipi屏)或"lcd2_4"(2.4寸mipi屏)显示方式
01科技（01Studio）在线训练平台：https://ai.01studio.cc
"""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
from libs.Utils import *
from media.sensor import *
from machine import FPIOA, UART
import os, sys, gc
import ulab.numpy as np
import image
import time

# 这里为自动生成内容，自定义场景请修改为您自己的模型路径、标签名称、模型输入大小
kmodel_path="/sdcard/yolo11n_det_320gq.kmodel"
labels = {0: '钢球'}
model_input_size = [320, 320]
# 显示模式，可以选择"hdmi"、"lcd3_5"(3.5寸mipi屏)和"lcd2_4"(2.4寸mipi屏)
display = "lcd3_5"

if display == "hdmi":
    display_mode = "hdmi"
    display_size = [1920, 1080]

elif display == "lcd3_5":
    display_mode = "st7701"
    display_size = [800, 480]

elif display == "lcd2_4":
    display_mode = "st7701"
    display_size = [640, 480]

rgb888p_size = [640, 360]
CAMERA_ID = 0  # CSI0 外置摄像头。
BOX_COLOR = (255, 0, 255, 0)
LABEL_FONT_SIZE = 18
LABEL_Y_OFFSET = 24
UART_SEND_INTERVAL_MS = 25  # 视觉外环约40Hz发送，STM32侧按最新一帧闭环。

# 黑点只用于人工标定，不在程序里识别黑点。
# 当前相机画面中水管长度方向对应OSD缓冲区x轴，所以标定线画成竖线。
# 根据K230实际检测输出标定：五个黑点处的钢球框中心cx约为
# 152.5、289.5、427.5、567.5、702.5，相邻黑点间隔5cm，中间点为O点。
BALL_AXIS = "x"
BALL_CAL_POINTS = [
    (-10.0, 152.5),
    (-5.0, 289.5),
    (0.0, 427.5),
    (5.0, 567.5),
    (10.0, 702.5),
]
BALL_LINE_TOP = 25
BALL_LINE_BOTTOM = 455
BALL_LINE_COLOR = (255, 0, 220, 0)
BALL_CENTER_LINE_COLOR = (255, 255, 40, 0)
BALL_TEXT_COLOR = (255, 255, 255, 255)
BALL_DOT_COLOR = (255, 255, 255, 0)

stm32_uart = None
last_uart_send_ms = 0


def uart1_init():
    fpioa = FPIOA()
    # CanMV K230 UART1：板上TX1/RX1，临时调试用。
    fpioa.set_function(3, FPIOA.UART1_TXD)
    fpioa.set_function(4, FPIOA.UART1_RXD)
    return UART(
        UART.UART1,
        baudrate=115200,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


def now_ms():
    try:
        return time.ticks_ms()
    except Exception:
        return int(time.time() * 1000)


def ms_diff(newer, older):
    try:
        return time.ticks_diff(newer, older)
    except Exception:
        return newer - older


def parse_best_target(res):
    best = None
    best_score = -1.0

    try:
        if res is None or len(res) < 3:
            return None
    except Exception:
        return None

    try:
        boxes = res[0]
        class_ids = res[1]
        scores = res[2]
        for i in range(len(boxes)):
            box = boxes[i]
            if len(box) < 4:
                continue

            score = float(scores[i]) if i < len(scores) else 0.0
            if score < best_score:
                continue

            x, y, w, h = normalize_box(box)
            class_id = int(class_ids[i]) if i < len(class_ids) else 0

            if w <= 0 or h <= 0:
                continue

            best_score = score
            best = {
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "cx": x + int(w / 2),
                "cy": y + int(h / 2),
                "class_id": class_id,
                "score": score,
            }
    except Exception:
        return None

    return best


def normalize_box(box):
    x = int(box[0])
    y = int(box[1])
    x2_or_w = int(box[2])
    y2_or_h = int(box[3])

    # 01Studio YOLO示例常见输出为x1,y1,x2,y2；统一转成x,y,w,h，避免中心点落到框边。
    if x2_or_w > x and y2_or_h > y:
        w = x2_or_w - x
        h = y2_or_h - y
    else:
        w = x2_or_w
        h = y2_or_h

    return x, y, w, h


def clamp_int(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value


def ball_axis_px(target):
    if BALL_AXIS == "y":
        return int(target["cy"])
    return int(target["cx"])


def ball_pos_cm_from_px(px):
    points = BALL_CAL_POINTS
    if len(points) < 2:
        return 0.0

    for i in range(len(points) - 1):
        cm0, px0 = points[i]
        cm1, px1 = points[i + 1]
        if (px >= px0 and px <= px1) or (px >= px1 and px <= px0):
            if px1 == px0:
                return cm0
            return cm0 + (float(px - px0) * (cm1 - cm0) / float(px1 - px0))

    cm0, px0 = points[0]
    cm1, px1 = points[1]
    if (px < px0 and px0 < px1) or (px > px0 and px0 > px1):
        if px1 == px0:
            return cm0
        return cm0 + (float(px - px0) * (cm1 - cm0) / float(px1 - px0))

    cm0, px0 = points[-2]
    cm1, px1 = points[-1]
    if px1 == px0:
        return cm1
    return cm0 + (float(px - px0) * (cm1 - cm0) / float(px1 - px0))


def ball_pos_mm_from_target(target):
    if not target:
        return 0
    return int(ball_pos_cm_from_px(ball_axis_px(target)) * 10.0)


def send_target_to_stm32(target, img_w, img_h):
    global last_uart_send_ms

    if stm32_uart is None:
        return

    t = now_ms()
    if ms_diff(t, last_uart_send_ms) < UART_SEND_INTERVAL_MS:
        return

    last_uart_send_ms = t
    if target:
        dx = int(target["cx"] - int(img_w / 2))
        dy = int(target["cy"] - int(img_h / 2))
        pos_mm = ball_pos_mm_from_target(target)
        msg = "T,{},{},{},{},{},{},{}\n".format(
            target["cx"], target["cy"], dx, dy, target["w"], target["h"], pos_mm
        )
    else:
        msg = "N\n"

    stm32_uart.write(msg)


def draw_ball_calibration(osd_img, target, img_w, img_h):
    top = clamp_int(BALL_LINE_TOP, 0, img_h - 1)
    bottom = clamp_int(BALL_LINE_BOTTOM, 0, img_h - 1)

    for cm, px in BALL_CAL_POINTS:
        px = clamp_int(int(px), 0, img_w - 1)
        color = BALL_CENTER_LINE_COLOR if cm == 0.0 else BALL_LINE_COLOR
        if BALL_AXIS == "y":
            osd_img.draw_line(0, px, img_w - 1, px, color=color, thickness=2)
            osd_img.draw_string_advanced(4, clamp_int(px + 3, 0, img_h - 22),
                                         16, "{:.0f}cm".format(cm), color=color)
        else:
            osd_img.draw_line(px, top, px, bottom, color=color, thickness=2)
            osd_img.draw_string_advanced(clamp_int(px + 3, 0, img_w - 62), top,
                                         16, "{:.0f}cm".format(cm), color=color)

    if target:
        cx = clamp_int(int(target["cx"]), 0, img_w - 1)
        cy = clamp_int(int(target["cy"]), 0, img_h - 1)
        pos_cm = ball_pos_cm_from_px(ball_axis_px(target))
        osd_img.draw_circle(cx, cy, 5, color=BALL_DOT_COLOR, thickness=2)
        osd_img.draw_string_advanced(8, 4, 20, "ball={:.1f}cm".format(pos_cm),
                                     color=BALL_TEXT_COLOR)
    else:
        osd_img.draw_string_advanced(8, 4, 20, "ball=N", color=BALL_TEXT_COLOR)


def draw_small_result(res, osd_img):
    osd_img.clear()

    try:
        if res is None or len(res) == 0:
            return
    except Exception:
        return

    try:
        boxes = res[0]
        class_ids = res[1] if len(res) > 1 else []
        scores = res[2] if len(res) > 2 else []
        if len(res) >= 3 and len(boxes) > 0:
            for i in range(len(boxes)):
                box = boxes[i]
                if len(box) < 4:
                    continue

                x, y, w, h = normalize_box(box)
                class_id = int(class_ids[i]) if i < len(class_ids) else 0
                score = float(scores[i]) if i < len(scores) else 0
                draw_one_result(osd_img, x, y, w, h, class_id, score)
            return
    except Exception:
        pass

    try:
        for det in res:
            if len(det) < 4:
                continue

            x = int(det[0])
            y = int(det[1])
            x2_or_w = int(det[2])
            y2_or_h = int(det[3])
            score = float(det[4]) if len(det) > 4 else 0
            class_id = int(det[5]) if len(det) > 5 else 0

            if x2_or_w > x and y2_or_h > y:
                w = x2_or_w - x
                h = y2_or_h - y
            else:
                w = x2_or_w
                h = y2_or_h

            draw_one_result(osd_img, x, y, w, h, class_id, score)
    except Exception as e:
        print("draw result failed:", e)


def draw_one_result(osd_img, x, y, w, h, class_id, score):
    if w <= 0 or h <= 0:
        return

    text_y = y - LABEL_Y_OFFSET
    if text_y < 0:
        text_y = y + 2

    label = labels.get(class_id, str(class_id))
    text = "{} {:.2f}".format(label, score)
    osd_img.draw_rectangle(x, y, w, h, color=BOX_COLOR, thickness=2)
    osd_img.draw_string_advanced(x, text_y, LABEL_FONT_SIZE, text, color=BOX_COLOR)

# 初始化PipeLine
pl = PipeLine(
    rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode
)

if display == "lcd2_4":
    sensor = Sensor(id=CAMERA_ID, width=1280, height=960)  # CSI0 外置摄像头，4:3画面。

else:
    sensor = Sensor(id=CAMERA_ID, width=1920, height=1080)  # CSI0 外置摄像头。

pl.create(sensor=sensor)

display_size = pl.get_display_size()

# 初始化YOLO11实例
confidence_threshold = 0.6  # 置信度
nms_threshold = 0.45
yolo = YOLO11(
    task_type="detect",
    mode="video",
    kmodel_path=kmodel_path,
    labels=labels,
    rgb888p_size=rgb888p_size,
    model_input_size=model_input_size,
    display_size=display_size,
    conf_thresh=confidence_threshold,
    nms_thresh=nms_threshold,
    max_boxes_num=50,
    debug_mode=0,
)
yolo.config_preprocess()

clock = time.clock()
stm32_uart = uart1_init()

while True:

    clock.tick()

    # 逐帧推理
    img = pl.get_frame()
    res = yolo.run(img)
    target = parse_best_target(res)
    draw_small_result(res, pl.osd_img)
    draw_ball_calibration(pl.osd_img, target, display_size[0], display_size[1])
    send_target_to_stm32(target, display_size[0], display_size[1])
    print(res)  # 打印识别结果
    pl.show_image()
    gc.collect()

    print("FPS:", clock.fps())  # 打印帧率

# 释放资源
yolo.deinit()
pl.destroy()
