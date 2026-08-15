/*
 * defense.c
 *
 * ゴールキーパー制御ロジック（ハンチング対策・安定追従版）
 */

#include "defense.h"
#include "sensor.h"
#include "motor.h"
#include <math.h>

/* ==================== 調整パラメータ ==================== */

/* 左右(ボール追従)制御 */
#define KEEPER_TRACK_KP            8.5f     /* 追従ゲイン */
#define KEEPER_TRACK_DEADZONE      1.5f     /* 不感帯（揺らぎ防止のため少し拡大） */
#define KEEPER_TRACK_MAX_SPEED     800.0f   /* 追従の限界速度 */
#define KEEPER_RETURN_KP           0.3f     /* ボール未検出時の減衰係数 */

/* ライン制御パラメータ */
#define LINE_SENSOR_COUNT         12
#define LINE_SENSOR_ANGLE_STEP    30.0f
#define LINE_SENSOR_A6_ANGLE       0.0f

#define ROBOT_RADIUS           62.0f
#define LINE_POS_KP               3.5f     /* ライン中心復帰ゲイン */

/* パターン制御パラメータ */
#define KEEPER_PUSH_VY_SPEED      -350.0f
#define VELOCITY_LPF_ALPHA         0.30f    /* 少し滑らかにしてハンチング抑止 */

#define TARGET_PATTERN_1_MASK ((1u << 0) | (1u << 6) | (1u << 7) | (1u << 12))
#define TARGET_PATTERN_2_MASK ((1u << 1) | (1u << 3) | (1u << 4) | (1u << 9))
#define TARGET_PATTERN_3_MASK ((1u << 2) | (1u << 6) | (1u << 10) | (1u << 14) | (1u << 15))
#define TARGET_PATTERN_4_MASK ((1u << 1) | (1u << 2) | (1u << 6) | (1u << 11))
#define TARGET_PATTERN_5_MASK ((1u << 2) | (1u << 8))
#define TARGET_PATTERN_6_MASK ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 13))

#define SLOW_DOWN_FACTOR          0.5f
#define DURATION_2_SEC_CYCLES     200
#define KEEPER_MAX_TOTAL_SPEED     850.0f

/* 内部状態 */
static float   ball_angle_filtered = 0.0f;
static float   vx_prev             = 0.0f;
static float   vx_filtered         = 0.0f;
static float   vy_filtered         = 0.0f;
static uint16_t slowdown_timer     = 0;
static uint16_t push_back_timer    = 0;

static float SensorIndexToAngleDeg(uint8_t idx)
{
    float angle = LINE_SENSOR_A6_ANGLE + ((float)idx - 6.0f) * LINE_SENSOR_ANGLE_STEP;
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static void AngleToXY(float angle_deg, float *x, float *y)
{
    float rad = angle_deg * 3.14159265f / 180.0f;
    *x = sinf(rad);   /* 90°で X=1 (右) */
    *y = cosf(rad);   /*  0°で Y=1 (正面) */
}

/* ラインの法線ベクトル（ロボットを押し戻す方向）と中心ズレ(d)のみを計算 */
static uint8_t ComputeLineNormal(float *out_nx, float *out_ny, float *out_d)
{
    uint16_t main_bits = line_sensor_bits & 0x0FFF;
    uint8_t count = 0;
    float sum_x = 0.0f;
    float sum_y = 0.0f;

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        if (main_bits & (1u << i))
        {
            float ax, ay;
            AngleToXY(SensorIndexToAngleDeg(i), &ax, &ay);
            sum_x += ax;
            sum_y += ay;
            count++;
        }
    }

    if (count == 0)
    {
        *out_nx = 0.0f; *out_ny = 0.0f; *out_d = 0.0f;
        return 0;
    }

    /* 反応したセンサーの平均ベクトル */
    float mx = sum_x / (float)count;
    float my = sum_y / (float)count;
    float len = sqrtf(mx * mx + my * my);

    if (len > 1e-3f)
    {
        *out_nx = mx / len;
        *out_ny = my / len;
        *out_d  = len * ROBOT_RADIUS;
    }
    else
    {
        *out_nx = 0.0f; *out_ny = 0.0f; *out_d = 0.0f;
    }

    return count;
}

static float Rad2DegLocal(float rad)
{
    return rad * 180.0f / 3.14159265f;
}

static float Clamp(float v, float min_v, float max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

/* メイン処理 */
void Defense_Update(float omega)
{
    float vx = 0.0f;
    float vy = 0.0f;

    /* ① ボール追従速度（純粋な左右移動 X軸） */
    if (ball_detected)
    {
        ball_angle_filtered = 0.7f * ball_angle_filtered + 0.3f * ball_angle;

        if (fabsf(ball_angle_filtered) > KEEPER_TRACK_DEADZONE)
        {
            /* ボールが右(+角度)なら +X(右) へ、左(-角度)なら -X(左) へ */
            vx = KEEPER_TRACK_KP * ball_angle_filtered;
            vx = Clamp(vx, -KEEPER_TRACK_MAX_SPEED, KEEPER_TRACK_MAX_SPEED);
        }
    }
    else
    {
        ball_angle_filtered = 0.0f;
        vx = vx_prev * (1.0f - KEEPER_RETURN_KP);
        if (fabsf(vx) < 5.0f) vx = 0.0f;
    }

    /* ② ライン位置補正（前後の押し戻し Y軸 ＋ ズレ抑制） */
    float norm_x = 0.0f, norm_y = 0.0f, chord_d = 0.0f;
    uint8_t line_hit_count = ComputeLineNormal(&norm_x, &norm_y, &chord_d);

    if (line_hit_count >= 1)
    {
        /* 白線から脱走しないように前後に押す力を Y 軸に加える */
        /* norm_y が正（前側に白線）なら正面へ、負（後側に白線）なら後ろへ補正 */
        float push_y = norm_y * chord_d * LINE_POS_KP;
        vy += push_y;

        /* ラインから大幅にハミ出そうになっている場合は、追従速度(vx)を安全に制限 */
        if (chord_d > 45.0f)
        {
            vx *= 0.3f;
        }
    }

    /* ③ パターン検出（特定ラインパターンの緊急制御） */
    uint8_t p1_match = ((line_sensor_bits & TARGET_PATTERN_1_MASK) == TARGET_PATTERN_1_MASK);
    uint8_t p2_match = ((line_sensor_bits & TARGET_PATTERN_2_MASK) == TARGET_PATTERN_2_MASK);
    uint8_t p3_match = ((line_sensor_bits & TARGET_PATTERN_3_MASK) == TARGET_PATTERN_3_MASK);
    uint8_t p4_match = ((line_sensor_bits & TARGET_PATTERN_4_MASK) == TARGET_PATTERN_4_MASK);
    uint8_t p5_match = ((line_sensor_bits & TARGET_PATTERN_5_MASK) == TARGET_PATTERN_5_MASK);
    uint8_t p6_match = ((line_sensor_bits & TARGET_PATTERN_6_MASK) == TARGET_PATTERN_6_MASK);

    if (p3_match || p4_match) push_back_timer = DURATION_2_SEC_CYCLES;
    if (p1_match || p2_match || p5_match || p6_match) slowdown_timer = DURATION_2_SEC_CYCLES;

    if (push_back_timer > 0)
    {
        vx = 0.0f;
        vy = KEEPER_PUSH_VY_SPEED;
        push_back_timer--;
    }
    else if (slowdown_timer > 0)
    {
        if (p1_match) { if (vx < 0.0f) vx = 0.0f; if (vy > 0.0f) vy = 0.0f; }
        if (p2_match) { if (vx > 0.0f) vx = 0.0f; if (vy > 0.0f) vy = 0.0f; }
        vx *= SLOW_DOWN_FACTOR;
        vy *= SLOW_DOWN_FACTOR;
        slowdown_timer--;
    }

    vx_prev = vx;

    /* ④ LPF（平滑化） */
    vx_filtered = (1.0f - VELOCITY_LPF_ALPHA) * vx_filtered + VELOCITY_LPF_ALPHA * vx;
    vy_filtered = (1.0f - VELOCITY_LPF_ALPHA) * vy_filtered + VELOCITY_LPF_ALPHA * vy;

    if (push_back_timer > 0)
    {
        vx_filtered = 0.0f;
        vy_filtered = KEEPER_PUSH_VY_SPEED;
    }

    /* ⑤ 出力計算 */
    float speed = sqrtf(vx_filtered * vx_filtered + vy_filtered * vy_filtered);
    if (speed > KEEPER_MAX_TOTAL_SPEED) speed = KEEPER_MAX_TOTAL_SPEED;

    /* 実績コードと同じ変換 */
    float angle = Rad2DegLocal(atan2f(vx_filtered, vy_filtered));

    Omni_Drive(angle, speed, omega);
}
