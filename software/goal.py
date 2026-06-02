import sensor, image, time, lcd, math
from machine import UART
from fpioa_manager import fm

# ============================================================
# ★ 設定（ここだけ変えればミラーあり/なし切り替え可能）
# ============================================================
USE_MIRROR = False        # 双曲線ミラーあり: True / なし: False

MIRROR_INNER_R = 30      # 中心から除外する半径（鏡面反射対策）
MIRROR_OUTER_R = 110     # 有効範囲の外半径

NORMAL_ROI = (0, 60, 320, 180)  # 通常カメラ時のROI（上下端カット）

SMOOTH = 0.4  # 角度の平滑化係数（0〜1: 小さいほど滑らか・遅延大）

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
sensor.skip_frames(time=2000)
clock = time.clock()

YELLOW_T = (30, 80, -20,  0,  28,  78)
BLUE_T   = ( 0, 60, -10, 50, -127, -20)
IMG_CX   = 160
IMG_CY   = 120

y_smooth = None
b_smooth = None


def in_mirror_roi(blob):
    """ミラーモード: 中心から適切な距離にあるブロブだけ通す"""
    dx = blob.cx() - IMG_CX
    dy = blob.cy() - IMG_CY
    r = math.sqrt(dx * dx + dy * dy)
    return MIRROR_INNER_R < r < MIRROR_OUTER_R


def calc_angle(blob):
    dx = blob.cx() - IMG_CX
    dy = IMG_CY - blob.cy()  # 上が正
    return math.degrees(math.atan2(dx, dy))


def smooth_angle(prev, new_val):
    """EMA平滑化（±180折り返しを考慮）"""
    if prev is None:
        return new_val
    diff = new_val - prev
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
    return prev + SMOOTH * diff


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

    # --- ブロブ検出 ---
    if USE_MIRROR:
        # ミラーモード: 全体検索 → ROIフィルタで絞る
        blobs = img.find_blobs(
            [YELLOW_T, BLUE_T],
            pixels_threshold=150,
            area_threshold=150,
            merge=True,
            margin=5
        )
        blobs = [b for b in blobs if in_mirror_roi(b)]
    else:
        # 通常カメラモード: ROIで検索範囲を絞る（FPS向上）
        blobs = img.find_blobs(
            [YELLOW_T, BLUE_T],
            roi=NORMAL_ROI,
            pixels_threshold=200,
            area_threshold=200,
            merge=True,
            margin=5
        )

    yg = max([b for b in blobs if b.code() & 1],
             key=lambda b: b.area(), default=None)
    bg = max([b for b in blobs if b.code() & 2],
             key=lambda b: b.area(), default=None)

    # 角度計算 + 平滑化
    y_raw = calc_angle(yg) if yg else None
    b_raw = calc_angle(bg) if bg else None
    y_smooth = smooth_angle(y_smooth, y_raw) if y_raw is not None else None
    b_smooth = smooth_angle(b_smooth, b_raw) if b_raw is not None else None

    # UART送信（STM32へ）
    send_packet(y_smooth, b_smooth)

    # 画面中心マーク
    img.draw_cross(IMG_CX, IMG_CY, color=(255, 255, 255), size=10)

    # ミラーモード時: 有効範囲を円で表示
    if USE_MIRROR:
        img.draw_circle(IMG_CX, IMG_CY, MIRROR_INNER_R, color=(100, 100, 100))
        img.draw_circle(IMG_CX, IMG_CY, MIRROR_OUTER_R, color=(100, 100, 100))

    # 描画
    if yg:
        img.draw_rectangle(yg.rect(), color=(255, 200, 0))
        img.draw_line(IMG_CX, IMG_CY, yg.cx(), yg.cy(),
                      color=(255, 200, 0), thickness=2)
        img.draw_string(yg.x(), yg.y() - 10,
                        "Y:%.0f" % y_smooth, color=(255, 200, 0))
    if bg:
        img.draw_rectangle(bg.rect(), color=(0, 80, 255))
        img.draw_line(IMG_CX, IMG_CY, bg.cx(), bg.cy(),
                      color=(0, 80, 255), thickness=2)
        img.draw_string(bg.x(), bg.y() - 10,
                        "B:%.0f" % b_smooth, color=(0, 80, 255))

    # USB送信（PCへ）
    print("Y:%-7s B:%-7s fps:%.1f" % (
        "%.0f" % y_smooth if y_smooth is not None else "NONE",
        "%.0f" % b_smooth if b_smooth is not None else "NONE",
        clock.fps()
    ))

    lcd.display(img)
