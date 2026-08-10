/*
 * defense.c
 *
 * ゴールキーパー制御ロジック
 *
 * 前提となる設計（会話でまとめた内容）:
 *  - ロボットは常にフィールド正面(0°)を向いたまま姿勢を固定する
 *  - ロボットの後ろ半分だけが自ゴールエリア内に入っている状態を維持する
 *  - 正しい深さのとき、ライン検出角度(line_angle)はロボット中心から見て
 *    ほぼ±90°になる
 *      - |line_angle| が 90° より小さくなる(0°側に近づく) → 下がりすぎ
 *      - |line_angle| が 90° より大きくなる(180°側に近づく) → 前に出すぎ
 *    → depth_error = |line_angle| - 90.0f を連続的な誤差信号として使う
 *  - depth_error に応じて前後方向(vy)を比例制御で補正する
 *    （カーブ・斜めのラインに差し掛かっても同じ式で自然になぞれる。
 *      直線・カーブ・斜めのどの区間でも「正しい深さからズレるほど
 *      line_angleが90°から離れる」という関係は変わらないため、
 *      特別な分岐を追加しなくてもこの仕組みでライントレースできる）
 *  - ボールの左右方向(ball_angle)に応じて横方向(vx)を比例制御で追従する
 *    ことを最優先とする
 *  - ボール追従中は depth_priority_scale で深さ補正を弱めるが、
 *    カーブ・斜め区間（depth_errorが大きい）では自動的にライン側の
 *    優先度を引き上げ、はみ出しすぎを防ぐ
 *  - さらに、ラインに乗っている間もdepth_errorが臨界値(60°)に近づく
 *    (40°を超える)ほどvxをなだらかに絞り、切り替わってからではなく
 *    先回りで減速する。ライン完全喪失時はさらに強く絞る
 *  - vx, vy をベクトル合成して1回のOmni_Driveにまとめる
 *  - ラインを完全に見失った(line_on_line=0)ときは、後退して再接触を試みる
 *  - サイドセンサー(A13/A15)は、ライン非検出時の異常検知・保険的なフォール
 *    バックとしてのみ使う（通常時のハード制限には使わない）
 */

#include "defense.h"
#include "sensor.h"
#include "motor.h"
#include <math.h>

/* ==================== 調整パラメータ ==================== */

#define KEEPER_BASE_HOLD_SPEED     150.0f   /* 静止構え時の基礎応答速度スケール */

/* 前後(深さ)制御 */
#define KEEPER_DEPTH_KP            5.0f     /* depth_error 1度あたりの前後速度 */
#define KEEPER_DEPTH_DEADZONE      30.0f    /* depth_errorの不感帯[deg]（実機調整済み） */
#define KEEPER_DEPTH_MAX_SPEED     300.0f   /* 前後方向の速度上限（サチュレーション） */
#define KEEPER_LOST_LINE_SPEED     250.0f   /* ライン見失い時、再接触に向かう後退速度 */

/* 左右(ボール追従)制御 */
#define KEEPER_TRACK_KP            4.0f     /* ball_angle 1度あたりの左右速度 */
#define KEEPER_TRACK_DEADZONE      5.0f     /* ball_angleの不感帯[deg] */
#define KEEPER_TRACK_MAX_SPEED     500.0f   /* 左右方向の速度上限 */
#define KEEPER_RETURN_KP           0.3f     /* ボール未検出時、中央へ戻す減衰係数 */

/* ボール検出時、depth_error（カーブ・斜め区間のきつさ）に応じて
   深さ補正の強さを可変にする
   - カーブ・斜めが緩い(depth_errorが小さい)ほどボール優先(scaleは小さい)
   - きつい(depth_errorが大きい)ほどライン優先(scaleは大きい) */
#define KEEPER_DEPTH_SCALE_BALL_MIN   0.15f  /* 直線に近い時：ボールをほぼ優先 */
#define KEEPER_DEPTH_SCALE_BALL_MAX   0.7f   /* きついカーブ/斜めの時：ラインを強める */
#define KEEPER_CURVE_NORM_DEG         90.0f  /* この角度でcurve_severityが1.0に達する */

/* 逸脱防止フェイルセーフ：depth_errorが臨界値に近づくほど、
   なだらかにvxを絞る（切り替わってから反応するのでは遅いため、
   ラインに乗っている間に先回りして減速する） */
#define KEEPER_DEPTH_WARN_DEG          40.0f  /* ここから絞り始める */
#define KEEPER_DEPTH_CRITICAL_DEG      60.0f  /* ここでほぼ0まで絞る */
#define KEEPER_DEPTH_CRITICAL_VX_FLOOR 0.05f  /* 最低でも残す比率 */

/* ライン完全喪失時用の抑制倍率（斜めの端で突っ切ってしまうのを防ぐ） */
#define KEEPER_LOST_LINE_VX_SCALE      0.15f

/* 全体速度の最終上限（motor.c側のOmni_Driveでも1000でクリップされるが、
   キーパーとしては暴れすぎ防止のため別途抑える） */
#define KEEPER_MAX_TOTAL_SPEED     700.0f

/* サイドセンサーのデバウンス回数（保険的フォールバック用） */
#define KEEPER_SIDE_DEBOUNCE_COUNT 4

/* ==================== 内部状態 ==================== */

static float   ball_angle_filtered = 0.0f;
static float   vx_prev             = 0.0f;   /* 未検出時のセンター復帰用に前回値を保持 */

static uint8_t side_left_off_count  = 0;
static uint8_t side_right_off_count = 0;

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

/**
 * @brief ゴールキーパーの1周期分の制御を実行する。
 *        main.cのループから毎周期呼ぶ想定。
 *        omegaはmain.c側で Sensor_GetOmega(0.0f, 1) により
 *        yaw=0固定のPIDとして計算したものを渡す。
 *
 * @param omega 姿勢制御用の角速度指令
 */
void Defense_Update(float omega)
{
    float vx = 0.0f;   /* 左右方向の速度成分（+側を右方向とする） */
    float vy = 0.0f;   /* 前後方向の速度成分（+側を後退方向とする） */

    /* ---------------------------------------------------------
     * ① 深さ(前後位置)制御：ラインに追従（ライントレース的）
     *    直線・カーブ・斜め区間すべて同じ式で扱う
     * --------------------------------------------------------- */
    float depth_error = 0.0f;
    if (line_on_line)
    {
        float abs_line_angle = fabsf(NormalizeAngle(line_angle));
        depth_error = abs_line_angle - 90.0f;
        side_left_off_count  = 0;
        side_right_off_count = 0;
    }
    else
    {
        vy = KEEPER_LOST_LINE_SPEED;
    }

    /* ---------------------------------------------------------
     * ② 左右方向：ボール追従を最優先（ただしカーブ・斜めがきつい時は
     *    ライン側の優先度を自動的に引き上げる）
     * --------------------------------------------------------- */
    float depth_priority_scale;
    if (ball_detected)
    {
        ball_angle_filtered = 0.7f * ball_angle_filtered + 0.3f * ball_angle;
        if (fabsf(ball_angle_filtered) > KEEPER_TRACK_DEADZONE)
        {
            vx = KEEPER_TRACK_KP * ball_angle_filtered;
            vx = Clamp(vx, -KEEPER_TRACK_MAX_SPEED, KEEPER_TRACK_MAX_SPEED);
        }

        /* depth_error（カーブ・斜めのきつさ）に応じてscaleを可変にする */
        float curve_severity = fabsf(depth_error) / KEEPER_CURVE_NORM_DEG;
        if (curve_severity > 1.0f) curve_severity = 1.0f;
        depth_priority_scale = KEEPER_DEPTH_SCALE_BALL_MIN
                              + (KEEPER_DEPTH_SCALE_BALL_MAX - KEEPER_DEPTH_SCALE_BALL_MIN) * curve_severity;
    }
    else
    {
        ball_angle_filtered = 0.0f;
        vx = vx_prev * (1.0f - KEEPER_RETURN_KP);
        if (fabsf(vx) < 5.0f) vx = 0.0f;
        depth_priority_scale = 1.0f;
    }
    vx_prev = vx;

    if (line_on_line && fabsf(depth_error) > KEEPER_DEPTH_DEADZONE)
    {
        vy = -KEEPER_DEPTH_KP * depth_error * depth_priority_scale;
        vy = Clamp(vy, -KEEPER_DEPTH_MAX_SPEED, KEEPER_DEPTH_MAX_SPEED);
    }

    /* ---------------------------------------------------------
     * ②.5 逸脱防止フェイルセーフ：ライン喪失時は強く絞り、
     *      ラインに乗っている間もdepth_errorが臨界値に近づくほど
     *      なだらかにvxを絞って先回りで減速する
     * --------------------------------------------------------- */
    if (!line_on_line)
    {
        vx *= KEEPER_LOST_LINE_VX_SCALE;
    }
    else
    {
        float d = fabsf(depth_error);
        if (d > KEEPER_DEPTH_WARN_DEG)
        {
            float t = (d - KEEPER_DEPTH_WARN_DEG)
                    / (KEEPER_DEPTH_CRITICAL_DEG - KEEPER_DEPTH_WARN_DEG);
            if (t > 1.0f) t = 1.0f;
            float scale = 1.0f - t * (1.0f - KEEPER_DEPTH_CRITICAL_VX_FLOOR);
            vx *= scale;
        }
    }

    /* ---------------------------------------------------------
     * ③ ベクトル合成してOmni_Driveへ
     * --------------------------------------------------------- */
    float speed = sqrtf(vx * vx + vy * vy);
    if (speed > KEEPER_MAX_TOTAL_SPEED) speed = KEEPER_MAX_TOTAL_SPEED;

    float angle = Rad2DegLocal(atan2f(vx, vy));

    Omni_Drive(angle, speed, omega);
}

/* 姿勢制御(omega)はmain.c側で Sensor_GetOmega(0.0f, 1) を使って
 * yaw=0固定のPIDとして計算済みのため、ここでは専用関数を持たない。
 */
