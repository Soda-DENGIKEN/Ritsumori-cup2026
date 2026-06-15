import sensor, image, time, lcd, math

from machine import UART

from fpioa_manager import fm



# ============================================================
# ★ 設定
# ============================================================

USE_MIRROR     = False   # True: ミラーあり / False: 通常カメラ
MIRROR_INNER_R = 30
MIRROR_OUTER_R = 110
SMOOTH         = 0.4     # 角度の平滑化係数（0〜1）

# ★ デバッグ設定（それぞれ個別にオン/オフ可能）
DEBUG_LCD   = False  # True: LCD表示・描画をオン
DEBUG_PRINT = True  # True: PCへのprint出力をオン

# ============================================================
# UART ピン設定
# ============================================================

fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, read_buf_len=0)

# ============================================================
# カメラ初期化
# ============================================================
lcd.init(freq=15000000)
sensor.reset()

sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

sensor.set_vflip(False)
sensor.set_hmirror(False)

sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)

sensor.set_contrast(2)
sensor.set_saturation(2)

sensor.skip_frames(time=2000)

clock = time.clock()

YELLOW_T = (65, 100,  -5,   8,  20, 100)
BLUE_T   = (0, 60, -15, 60, -128, -17)

IMG_CX = 160
IMG_CY = 120

y_smooth = None
b_smooth = None

def in_mirror_roi(blob):
    dx = blob.cx() - IMG_CX
    dy = blob.cy() - IMG_CY
    r = math.sqrt(dx * dx + dy * dy)
    return MIRROR_INNER_R < r < MIRROR_OUTER_R

def calc_distance(blob):
    dx = blob.cx() - IMG_CX
    dy = blob.cy() - IMG_CY

    return math.sqrt(dx * dx + dy * dy)

def calc_angle_from_y(y):
    dy = IMG_CY - y

    return dy / IMG_CY * 90.0

def smooth_angle(prev, new_val):
    if prev is None:
        return new_val
    diff = new_val - prev
    if diff >  180: diff -= 360
    if diff < -180: diff += 360
    return prev + SMOOTH * diff

def val_to_bytes(val, signed=True):
    if signed:
        v = max(-32768, min(32767, int(val)))
        if v < 0: v += 65536
    else:
        v = max(0, min(65534, int(val)))
    return bytes([(v >> 8) & 0xFF, v & 0xFF])

NOT_DET = b'\x7f\xff'


def send_packet(y_ang, y_dist, b_ang, b_dist):

    """
    10バイトパケット:
    AA [Y角度×2] [Y距離×2] [B角度×2] [B距離×2] FF
    未検出は 0x7FFF
    """

    pkt = (b'\xaa'
           + (val_to_bytes(y_ang)         if y_ang  is not None else NOT_DET)
           + (val_to_bytes(y_dist, False)  if y_dist is not None else NOT_DET)
           + (val_to_bytes(b_ang)         if b_ang  is not None else NOT_DET)
           + (val_to_bytes(b_dist, False)  if b_dist is not None else NOT_DET)
           + b'\xff')

    uart.write(pkt)

while True:
    clock.tick()
    img = sensor.snapshot()

    if DEBUG_LCD:
        img = img.rotation_corr(z_rotation=90)

    # --- ブロブ検出 ---

    if USE_MIRROR:
        y_blobs = [b for b in img.find_blobs([YELLOW_T], pixels_threshold=150, area_threshold=150, merge=True, margin=5) if in_mirror_roi(b)]
        b_blobs = [b for b in img.find_blobs([BLUE_T],   pixels_threshold=150, area_threshold=150, merge=True, margin=5) if in_mirror_roi(b)]
    else:
        y_blobs = img.find_blobs([YELLOW_T], pixels_threshold=200, area_threshold=200, merge=True, margin=5)
        b_blobs = img.find_blobs([BLUE_T],   pixels_threshold=200, area_threshold=200, merge=True, margin=5)

    yg = max(y_blobs, key=lambda b: b.area(), default=None)
    bg = max(b_blobs, key=lambda b: b.area(), default=None)

    # --- 黄色 ---
    if yg:
        y_raw = calc_angle_from_y(yg.cy())
        y_smooth = smooth_angle(y_smooth, y_raw)
        y_dist = calc_distance(yg)
    else:
        y_smooth = None
        y_dist = None

    # --- 青 ---
    if bg:
        b_raw = calc_angle_from_y(bg.cy())
        b_smooth = smooth_angle(b_smooth, b_raw)
        b_dist = calc_distance(bg)
    else:
        b_smooth = None
        b_dist = None

    # --- UART送信 ---
    send_packet(
        y_smooth,
        y_dist,
        b_smooth,
        b_dist
    )

    if DEBUG_PRINT:
        print(
            "Y_ang={} Y_dist={} B_ang={} B_dist={}".format(
                int(y_smooth) if y_smooth is not None else -1,
                int(y_dist) if y_dist is not None else -1,
                int(b_smooth) if b_smooth is not None else -1,
                int(b_dist) if b_dist is not None else -1
            )
        )

    # --- LCD表示・描画（DEBUG_LCD = True のとき）---
    if DEBUG_LCD:
        img.draw_cross(IMG_CX, IMG_CY, color=(255, 255, 255), size=10)

        if USE_MIRROR:
            img.draw_circle(IMG_CX, IMG_CY, MIRROR_INNER_R, color=(100, 100, 100))
            img.draw_circle(IMG_CX, IMG_CY, MIRROR_OUTER_R, color=(100, 100, 100))

        if yg:
            img.draw_rectangle(yg.rect(), color=(255, 200, 0))
            img.draw_cross(
                yg.cx(),
                yg.cy(),
                color=(255,0,0),
                size=8
            )

            img.draw_line(
                IMG_CX,
                IMG_CY,
                yg.cx(),
                yg.cy(),
                color=(255,0,0),
                thickness=2
            )
            img.draw_string(yg.x(), yg.y() - 10,

                            "Y:%.0f d:%.0f" % (y_smooth, y_dist), color=(255, 200, 0))

        if bg:

            img.draw_rectangle(bg.rect(), color=(0, 80, 255))
            img.draw_cross(
                bg.cx(),
                bg.cy(),
                color=(255,0,0),
                size=8
            )

            img.draw_line(
                IMG_CX,
                IMG_CY,
                bg.cx(),
                bg.cy(),
                color=(255,0,0),
                thickness=2
            )
            img.draw_string(bg.x(), bg.y() - 10,

                            "B:%.0f d:%.0f" % (b_smooth, b_dist), color=(0, 80, 255))



        lcd.display(img)
