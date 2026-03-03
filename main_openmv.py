import sensor, time, display
from pyb import millis, Servo, UART
import ustruct

# 目标颜色阈值（LAB）
target_threshold = (97, 73, -53, -16, 106, 19)#(15, 44, -44, -7, 8, 32)(13, 37, -38, -18, 8, 38)

# 识别过滤参数
PIXELS_THRESHOLD = 150
AREA_THRESHOLD   = 150
MERGE_BLOBS      = True

uart = UART(3, 115200)

# ---------- 应答机制 ----------
ACK = b'\xAC'
awaiting_ack = False
last_pkt = None
last_send_ms = 0

ACK_TIMEOUT_MS = 25
MAX_RETRY = 5
retry_cnt = 0

# ---------- ROI：从 QVGA(320x240) 截取中心 128x160 ----------
ROI_W, ROI_H = 128, 160
# QVGA: 320x240 -> 中心裁剪：x=96, y=40
ROI_X = (320 - ROI_W) // 2   # 96
ROI_Y = (240 - ROI_H) // 2   # 40
ROI = (ROI_X, ROI_Y, ROI_W, ROI_H)

# ---------------- 工具函数 ----------------
def find_max(blobs):
    max_blob = blobs[0]
    max_area = max_blob.w() * max_blob.h()
    for b in blobs[1:]:
        a = b.w() * b.h()
        if a > max_area:
            max_blob = b
            max_area = a
    return max_blob

def clamp(v, vmin, vmax):
    if v < vmin: return vmin
    if v > vmax: return vmax
    return v

def make_pkt(c1, c2, c3):
    return ustruct.pack("<BBBBBB", 0xA5, 0xA6, c1, c2, c3, 0x5B)

def try_send(pkt):
    global awaiting_ack, last_pkt, last_send_ms, retry_cnt
    uart.write(pkt)
    awaiting_ack = True
    last_pkt = pkt
    last_send_ms = millis()
    retry_cnt = 0

def comm_update():
    """每帧调用：读取ACK、超时重发"""
    global awaiting_ack, last_send_ms, retry_cnt

    if not awaiting_ack:
        return True

    if uart.any():
        b = uart.read(1)
        if b == ACK:
            awaiting_ack = False
            return True

    if millis() - last_send_ms > ACK_TIMEOUT_MS:
        if retry_cnt < MAX_RETRY and last_pkt is not None:
            uart.write(last_pkt)
            last_send_ms = millis()
            retry_cnt += 1
        else:
            awaiting_ack = False
            return True

    return False

# ---------------- 初始化 ----------------
def init_setup():
    global pan_servo, tilt_servo, clock, lcd

    pan_servo = Servo(1)
    tilt_servo = Servo(2)
    pan_servo.calibration(500, 2500, 1500)
    tilt_servo.calibration(500, 2500, 1500)

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)   # 320x240 识别不变
    sensor.skip_frames(time=1000)
    sensor.set_auto_whitebal(False)

    clock = time.clock()

    # TFT：用你验证过的方式
    lcd = display.SPIDisplay()
    lcd.backlight(100)
    lcd.clear()

def middle():
    pan_servo.angle(0, 1000)
    tilt_servo.angle(0, 1000)
    time.sleep(1)

# ---------------- 主循环 ----------------
def main():
    while True:
        clock.tick()
        img = sensor.snapshot()  # 320x240

        # 先做识别（在全图上找）
        blobs = img.find_blobs(
            [target_threshold],
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=MERGE_BLOBS
        )

        can_send = comm_update()

        # 生成 TFT 要显示的中心画面（重要：用 copy(roi=...)）
        view = img.copy(roi=ROI)  # 128x160

        # 在 view 上画中心十字（128x160 的中心）
        cxv, cyv = ROI_W // 2, ROI_H // 2
        view.draw_cross(cxv, cyv, color=(255, 255, 255))

        if blobs:
            b = find_max(blobs)

            # --- 把全图坐标映射到 ROI(view) 坐标 ---
            bx = b.cx() - ROI_X
            by = b.cy() - ROI_Y

            # 只在目标中心落入 ROI 时，才画十字
            if 0 <= bx < ROI_W and 0 <= by < ROI_H:
                view.draw_cross(bx, by, color=(0, 255, 0))

            # 框也映射一下（做个简单裁剪，避免越界）
            rx = b.x() - ROI_X
            ry = b.y() - ROI_Y
            rw = b.w()
            rh = b.h()

            # 裁剪到 ROI 内（简化处理）
            x1 = clamp(rx, 0, ROI_W - 1)
            y1 = clamp(ry, 0, ROI_H - 1)
            x2 = clamp(rx + rw, 0, ROI_W)
            y2 = clamp(ry + rh, 0, ROI_H)
            if x2 > x1 and y2 > y1:
                view.draw_rectangle(x1, y1, x2 - x1, y2 - y1, color=(0, 255, 0))

            # 文字信息画在 view 上
            view.draw_string(0, 0, "FPS:%.1f" % clock.fps(), color=(255,255,255))
            view.draw_string(0, 12, "Blob:%d" % len(blobs), color=(255,255,0))
            view.draw_string(0, 24, "x:%d y:%d" % (b.cx(), b.cy()), color=(255,255,0))

            # 串口发送（用全图坐标 b.cx/b.cy）
            if can_send:
                if b.cx() > 255:
                    pkt = make_pkt(255, b.cx() - 255, b.cy())
                else:
                    pkt = make_pkt(0, b.cx(), b.cy())
                try_send(pkt)

        else:
            view.draw_string(0, 0, "NO TARGET", color=(255,0,0))
            if can_send:            #表示没有检测到目标
                pkt = make_pkt(255, 255, 255)
                try_send(pkt)


        #写到 TFT
        lcd.write(view)

# ---------------- 入口 ----------------
if __name__ == "__main__":
    init_setup()
    try:
        main()
    finally:
        middle()
