import sensor, image, time, lcd, math
from machine import UART
from fpioa_manager import fm

# --- UART ピン設定（HY2.0-4P: TX=GPIO34, RX=GPIO35）---
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, read_buf_len=0)

# --- カメラ初期化 ---
lcd.init(freq=15000000)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(False)
sensor.skip_frames(time=2000)
clock = time.clock()

YELLOW_T = (30, 80, -20,  0,  28,  78)
BLUE_T   = ( 0, 60, -10, 50, -127, -20)
IMG_CX   = 160
IMG_CY   = 120


def calc_angle(blob):
    dx = blob.cx() - IMG_CX
    dy = IMG_CY - blob.cy()
    return math.degrees(math.atan2(dx, dy))


def angle_to_bytes(angle):
    v = max(-32768, min(32767, int(angle)))
    if v < 0:
        v += 65536
    return bytes([(v >> 8) & 0xFF, v & 0xFF])


def send_packet(y_angle, b_angle):
    y_b = angle_to_bytes(y_angle) if y_angle is not None else b'\x7f\xff'
    b_b = angle_to_bytes(b_angle) if b_angle is not None else b'\x7f\xff'
    uart.write(b'\xaa' + y_b + b_b + b'\xff')


while True:
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs(
        [YELLOW_T, BLUE_T],
        pixels_threshold=200,
        area_threshold=200,
        merge=False
    )

    yg = max([b for b in blobs if b.code() & 1],
             key=lambda b: b.area(), default=None)
    bg = max([b for b in blobs if b.code() & 2],
             key=lambda b: b.area(), default=None)

    y_angle = calc_angle(yg) if yg else None
    b_angle = calc_angle(bg) if bg else None

    # UART送信（STM32へ）
    send_packet(y_angle, b_angle)

    # 画面中心マーク
    img.draw_cross(IMG_CX, IMG_CY, color=(255, 255, 255), size=10)

    # 描画
    if yg:
        img.draw_rectangle(yg.rect(), color=(255, 200, 0))
        img.draw_line(IMG_CX, IMG_CY, yg.cx(), yg.cy(), color=(255, 200, 0), thickness=2)
        img.draw_string(yg.x(), yg.y() - 10,
                        "Y:%.0f" % y_angle, color=(255, 200, 0))
    if bg:
        img.draw_rectangle(bg.rect(), color=(0, 80, 255))
        img.draw_line(IMG_CX, IMG_CY, bg.cx(), bg.cy(), color=(0, 80, 255), thickness=2)
        img.draw_string(bg.x(), bg.y() - 10,
                        "B:%.0f" % b_angle, color=(0, 80, 255))

    # USB送信（PCへ）
    print("Y:%-7s B:%-7s fps:%.1f" % (
        "%.0f" % y_angle if y_angle is not None else "NONE",
        "%.0f" % b_angle if b_angle is not None else "NONE",
        clock.fps()
    ))

    lcd.display(img)
