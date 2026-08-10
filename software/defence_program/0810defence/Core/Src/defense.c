/*
 * defense.c
 *
 * ゴールキーパー制御ロジック
 *
 * 設計のポイント:
 *  - 直線・ライン上において、ボールを常にロボットの【真正面】に捉えるよう追従精度を強化
 *  - 💡 パターン1, 2検出時：指定方向の進入をブロック (0.0f)
 *  - 💡 パターン3, 4検出時：ボール追従を中断し、真前（エリア内）へ押し戻す
 *  - 💡 パターン1,2,5,6検出時：減速タイマーを発動
 *  - 💡 ローパスフィルタ(LPF)の導入：パターン切替時のガクガク（速度急変）をなめらかに補正
 *  - 💡【変更】ライン維持を最優先とし、ラインからのズレ量に応じてボール追従(vx)を
 *          段階的に制限する「ライン優先モデル」に変更。斜めライン走行中に
 *          ボールへ引っ張られてラインを外れる問題を解消する。
 */

#include "defense.h"
#include "sensor.h"
#include "motor.h"
#include <math.h>

/* ==================== 調整パラメータ ==================== */

#define KEEPER_BASE_HOLD_SPEED     150.0f   /* 静止構え時の基礎応答速度スケール */

/* 前後(深さ)制御 */
#define KEEPER_DEPTH_KP            6.0f     /* depth_error 1度あたりの前後速度 */
#define KEEPER_DEPTH_DEADZONE      15.0f    /* 不感帯を狭めて斜めラインへの反応を早くする */
#define KEEPER_DEPTH_MAX_SPEED     450.0f   /* 前後方向の速度上限（高速化のため引き上げ） */
#define KEEPER_LOST_LINE_SPEED     300.0f   /* ライン見失い時、再接触に向かう後退速度 */

/* 左右(ボール追従)制御 💡全体的に高速化 */
#define KEEPER_TRACK_KP            8.5f     /* 追従ゲイン強化（6.5 -> 8.5） */
#define KEEPER_TRACK_DEADZONE      1.0f     /* 不感帯を最小化 */
#define KEEPER_TRACK_MAX_SPEED     800.0f   /* 追従の限界速度 */
#define KEEPER_RETURN_KP           0.3f     /* ボール未検出時、中央へ戻す減衰係数 */

/* 💡【変更】ライン優先モデル用パラメータ */
/* depth_errorがこの角度を超えたら、ボール追従(vx)の制限を開始する */
#define KEEPER_LINE_PRIORITY_START_ANGLE   KEEPER_DEPTH_DEADZONE  /* 15.0f: deadzoneと同時に制限開始 */
/* この角度でボール追従(vx)を完全に0にする（safe_ratioが0になる角度） */
#define KEEPER_LINE_PRIORITY_ZERO_ANGLE    70.0f
/* この角度を超えたら問答無用でvx=0にするハードリミット（安全弁） */
#define KEEPER_LINE_PRIORITY_HARD_LIMIT    60.0f
/* ズレを押し戻す力のゲイン（depth制御に上乗せする分） */
#define KEEPER_CURVE_PUSH_KP       4.0f

/* 💡 パターン3, 4検出時の押し戻し速度（より素早く復帰するように強化） */
#define KEEPER_PUSH_VY_SPEED      -350.0f   /* 前進方向（エリア内）へ押す速度 (-200 -> -350) */

/* 💡 ガクガク防止用ローパスフィルタ（LPF）の係数 */
/* 1.0fに近いほど応答が早く（ガクガクしやすい）、小さくするほど滑らか（応答は少し遅れる） */
#define VELOCITY_LPF_ALPHA         0.35f

/* 💡特定のセンサーパターンのビットマスク定義 */
/* パターン1: [1000001100001000] (bit0, bit6, bit7, bit12) */
#define TARGET_PATTERN_1_MASK ((1u << 0) | (1u << 6) | (1u << 7) | (1u << 12))

/* パターン2: [0000001000011010] (bit1, bit3, bit4, bit9) */
#define TARGET_PATTERN_2_MASK ((1u << 1) | (1u << 3) | (1u << 4) | (1u << 9))

/* パターン3: [0010001000100011] (bit2, bit6, bit10, bit14, bit15) */
#define TARGET_PATTERN_3_MASK ((1u << 2) | (1u << 6) | (1u << 10) | (1u << 14) | (1u << 15))

/* パターン4: [0110001000010000] (bit1, bit2, bit6, bit11) */
#define TARGET_PATTERN_4_MASK ((1u << 1) | (1u << 2) | (1u << 6) | (1u << 11))

/* パターン5: [0010000010000000] (bit2, bit8) -> 減速用 */
#define TARGET_PATTERN_5_MASK ((1u << 2) | (1u << 8))

/* パターン6: [0111000000000100] (bit1, bit2, bit3, bit13) -> 減速用 */
#define TARGET_PATTERN_6_MASK ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 13))

/* 減速制御用の定数 */
#define SLOW_DOWN_FACTOR          0.5f   /* パターン検出時の速度倍率（50%に落とす） */
#define DURATION_2_SEC_CYCLES     200    /* 約2秒間のサイクル数 */

/* 全体速度の最終上限（より速く動くように拡大） */
#define KEEPER_MAX_TOTAL_SPEED     850.0f

/* ==================== 内部状態 ==================== */

static float   ball_angle_filtered = 0.0f;
static float   vx_prev             = 0.0f;
static float   vx_filtered         = 0.0f;  /* 💡 LPF適用後の滑らかな速度 */
static float   vy_filtered         = 0.0f;  /* 💡 LPF適用後の滑らかな速度 */
static uint16_t slowdown_timer     = 0;  /* パターン1, 2, 5, 6用減速タイマー */
static uint16_t push_back_timer    = 0;  /* パターン3, 4用押し戻しタイマー (2秒) */

/* ==================== 内部ユーティリティ ==================== */

static float NormalizeAngle(float angle)
{
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
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

/* ==================== メイン処理 ==================== */

void Defense_Update(float omega)
{
    float vx = 0.0f;   /* 左右速度（+が右） */
    float vy = 0.0f;   /* 前後速度（+が後退、-が前進） */

    /* ---------------------------------------------------------
     * ① 深さ(前後位置)制御 ＆ 斜めコーナー脱輪防止
     * --------------------------------------------------------- */
    float depth_error = 0.0f;
    if (line_on_line)
    {
        float abs_line_angle = fabsf(NormalizeAngle(line_angle));
        depth_error = abs_line_angle - 90.0f;

        if (fabsf(depth_error) > KEEPER_DEPTH_DEADZONE)
        {
            vy = -KEEPER_DEPTH_KP * depth_error;
            vy = Clamp(vy, -KEEPER_DEPTH_MAX_SPEED, KEEPER_DEPTH_MAX_SPEED);
        }
    }
    else
    {
        vy = KEEPER_LOST_LINE_SPEED;
    }

    /* ---------------------------------------------------------
     * ② 左右(ボール追従)制御
     * --------------------------------------------------------- */
    if (ball_detected)
    {
        ball_angle_filtered = 0.6f * ball_angle_filtered + 0.4f * ball_angle;

        if (fabsf(ball_angle_filtered) > KEEPER_TRACK_DEADZONE)
        {
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

    /* ---------------------------------------------------------
     * 💡②.5 ライン優先モデル：
     *   ラインからのズレ(depth_error)が大きいほどボール追従(vx)を
     *   段階的に絞り、ライン維持(vy)の力を上乗せする。
     *   これにより「ボールは追うが、ラインからは外れない」を実現する。
     * --------------------------------------------------------- */
    if (line_on_line)
    {
        float abs_d = fabsf(depth_error);

        if (abs_d > KEEPER_LINE_PRIORITY_START_ANGLE)
        {
            /* ズレ量に応じてボール追従の許容割合を線形に減らす (1.0 -> 0.0) */
            float span = KEEPER_LINE_PRIORITY_ZERO_ANGLE - KEEPER_LINE_PRIORITY_START_ANGLE;
            float safe_ratio = 1.0f - ((abs_d - KEEPER_LINE_PRIORITY_START_ANGLE) / span);
            safe_ratio = Clamp(safe_ratio, 0.0f, 1.0f);
            vx *= safe_ratio;

            /* ライン復帰の力を追加で上乗せ（deadzone分に加えて更に押し戻す） */
            float extra_push = -KEEPER_CURVE_PUSH_KP * (abs_d - KEEPER_LINE_PRIORITY_START_ANGLE);
            vy += extra_push;
            vy = Clamp(vy, -KEEPER_DEPTH_MAX_SPEED, KEEPER_DEPTH_MAX_SPEED);
        }

        /* 安全弁：大きくズレたら問答無用でボール追従を完全停止 */
        if (abs_d > KEEPER_LINE_PRIORITY_HARD_LIMIT)
        {
            vx = 0.0f;
        }
    }

    /* ---------------------------------------------------------
     * ②.6 特定パターン検出時の制御
     * --------------------------------------------------------- */
    uint8_t p1_match = ((line_sensor_bits & TARGET_PATTERN_1_MASK) == TARGET_PATTERN_1_MASK);
    uint8_t p2_match = ((line_sensor_bits & TARGET_PATTERN_2_MASK) == TARGET_PATTERN_2_MASK);
    uint8_t p3_match = ((line_sensor_bits & TARGET_PATTERN_3_MASK) == TARGET_PATTERN_3_MASK);
    uint8_t p4_match = ((line_sensor_bits & TARGET_PATTERN_4_MASK) == TARGET_PATTERN_4_MASK);
    uint8_t p5_match = ((line_sensor_bits & TARGET_PATTERN_5_MASK) == TARGET_PATTERN_5_MASK);
    uint8_t p6_match = ((line_sensor_bits & TARGET_PATTERN_6_MASK) == TARGET_PATTERN_6_MASK);

    /* A. パターン3, 4の時：2秒間の押し戻しタイマーを発動 */
    if (p3_match || p4_match)
    {
        push_back_timer = DURATION_2_SEC_CYCLES;
    }

    /* B. パターン1, 2, 5, 6の時：2秒間の減速タイマーを発動 */
    if (p1_match || p2_match || p5_match || p6_match)
    {
        slowdown_timer = DURATION_2_SEC_CYCLES;
    }

    /* --- 優先制御処理 --- */

    /* 1. 押し戻し動作（パターン3, 4のタイマー作動中） */
    if (push_back_timer > 0)
    {
        vx = 0.0f;                  /* ボール追従をストップ */
        vy = KEEPER_PUSH_VY_SPEED;  /* 真前へ強く復帰 (-350.0f) */
        push_back_timer--;
    }
    /* 2. 通常減速動作（パターン1, 2, 5, 6のタイマー作動中） */
    else if (slowdown_timer > 0)
    {
        /* パターン1・2作動中の壁方向進入禁止 */
        if (p1_match)
        {
            if (vx < 0.0f) vx = 0.0f;
            if (vy > 0.0f) vy = 0.0f;
        }
        if (p2_match)
        {
            if (vx > 0.0f) vx = 0.0f;
            if (vy > 0.0f) vy = 0.0f;
        }

        vx *= SLOW_DOWN_FACTOR;
        vy *= SLOW_DOWN_FACTOR;
        slowdown_timer--;
    }

    /* 次周期保持 */
    vx_prev = vx;

    /* ---------------------------------------------------------
     * 💡 ③ ローパスフィルタ（LPF）による速度平滑化（ガクガク防止）
     * --------------------------------------------------------- */
    vx_filtered = (1.0f - VELOCITY_LPF_ALPHA) * vx_filtered + VELOCITY_LPF_ALPHA * vx;
    vy_filtered = (1.0f - VELOCITY_LPF_ALPHA) * vy_filtered + VELOCITY_LPF_ALPHA * vy;

    /* ---------------------------------------------------------
     * ④ ベクトル合成して Omni_Drive へ出力
     * --------------------------------------------------------- */
    float speed = sqrtf(vx_filtered * vx_filtered + vy_filtered * vy_filtered);
    if (speed > KEEPER_MAX_TOTAL_SPEED) speed = KEEPER_MAX_TOTAL_SPEED;

    float angle = Rad2DegLocal(atan2f(vx_filtered, vy_filtered));

    Omni_Drive(angle, speed, omega);
}
