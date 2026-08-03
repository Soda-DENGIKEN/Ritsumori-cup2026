#include "attack.h"
#include "sensor.h"
#include "motor.h"
#include <math.h>

#define BASE_SPEED           500
#define ESCAPE_SPEED_MAX     650  // 合成時の最大速度制限

// 外部定義のグローバル変数を参照
extern volatile float    ball_angle;
extern volatile uint8_t  ball_detected;
extern volatile float    goal_blue_angle;
extern volatile uint8_t  goal_blue_detected;
extern volatile float    goal_yellow_angle;
extern volatile uint8_t  goal_yellow_detected;
extern volatile uint8_t  line_pushed_out;
extern volatile uint8_t  attack_goal_color;
extern volatile uint8_t  line_detected_count;

// 回り込みの計算状態を保持する静的変数[cite: 12]
static float ball_angle_filtered = 0.0f;
static float move_dir_filtered   = 0.0f;
static float ball_angle_prev     = 0.0f;

// ボールの周りを回り込む軌道を計算する関数[cite: 12]
static void CalculateBallOrbit(float b_angle, float omega, float *out_angle, float *out_speed, float *out_omega)
{
    // ローパスフィルタでボール角度の急激な変化を抑制[cite: 12]
    ball_angle_filtered = 0.8f * ball_angle_filtered + 0.2f * b_angle;

    float raw_dir = 0.0f;
    float abs_angle = fabsf(ball_angle_filtered);

    // ボールの角度に応じて回り込み角度を決定[cite: 12]
    if (abs_angle < 30.0f)
    {
        raw_dir = ball_angle_filtered * 2.5f;
    }
    else if (abs_angle < 90.0f)
    {
        float sign = (ball_angle_filtered > 0.0f) ? 1.0f : -1.0f;
        raw_dir = ball_angle_filtered + sign * 45.0f;
    }
    else
    {
        float sign = (ball_angle_filtered > 0.0f) ? 1.0f : -1.0f;
        raw_dir = ball_angle_filtered + sign * 90.0f;
    }

    // 移動方向の急変を防ぐフィルタ[cite: 12]
    move_dir_filtered = 0.7f * move_dir_filtered + 0.3f * raw_dir;

    while (move_dir_filtered >  180.0f) move_dir_filtered -= 360.0f;
    while (move_dir_filtered < -180.0f) move_dir_filtered += 360.0f;

    *out_angle = move_dir_filtered;
    *out_speed = BASE_SPEED;
    *out_omega = omega;

    ball_angle_prev = ball_angle_filtered;
}

// アタックタスクのメイン更新ルーチン
void Attack_Update(float omega)
{
    // -------------------------------------------------------------
    // 1. 通常時の「ロボットが本来進むべき移動方向と速度」の仮計算
    // -------------------------------------------------------------
    float ball_move_angle = 0.0f;
    float ball_move_speed = 0.0f;
    float ball_move_omega = omega;

    uint8_t sample_detected = ball_detected;
    float sample_angle = ball_angle;

    if (sample_detected)
    {
        // ボールがある時の回り込み計算を実行[cite: 12]
        CalculateBallOrbit(sample_angle, omega, &ball_move_angle, &ball_move_speed, &ball_move_omega);

        if (ball_move_speed > 500.0f)
        {
            ball_move_speed = 500.0f; // 通常時の最高速度を 500 に制限[cite: 12]
        }

        // ゴール情報の取得[cite: 12]
        float goal_angle = (attack_goal_color == 0) ? goal_blue_angle : goal_yellow_angle;
        uint8_t goal_detected = (attack_goal_color == 0) ? goal_blue_detected : goal_yellow_detected;

        // ゴールアタック判定（チャンス時のシュート角度上書き）[cite: 12]
        if (fabsf(sample_angle) < 15.0f && goal_detected)
        {
            ball_move_angle = 0.7f * sample_angle + 0.3f * goal_angle;
            ball_move_speed = 350.0f; // シュート体制時の基本速度[cite: 12]
        }
    }
    else
    {
        // ボールを見失った時はその場停止（旋回のみ）[cite: 12]
        ball_angle_filtered = 0.0f;
        move_dir_filtered   = 0.0f;
        ball_angle_prev     = 0.0f;

        ball_move_angle = 0.0f;
        ball_move_speed = 0.0f;
    }

    // -------------------------------------------------------------
    // 2. 新システム：ライン検出時の「ベクトル合成」処理
    // -------------------------------------------------------------
    float escape_angle;
    if (Sensor_GetEscapeAngle(&escape_angle))
    {
        // ラインから逃げるパワー（斥力）の決定[cite: 12]
        float escape_speed = 300.0f;

        if (line_detected_count >= 3)
        {
            escape_speed = 650.0f; // 3個以上反応：最大パワー[cite: 12]
        }
        else if (line_detected_count == 2)
        {
            escape_speed = 450.0f; // 2個反応：中パワー[cite: 12]
        }
        else
        {
            escape_speed = 300.0f; // 1個反応または離脱直後：低パワー（ブレーキ）[cite: 12]
        }

        if (line_pushed_out) escape_speed = 650.0f; // 押し出し時は問答無用でMAX[cite: 12]

        // 【新コアロジック】進みたい方向（ボール）と逃げたい方向（ライン）のベクトルを合成
        float ball_rad   = ball_move_angle * 3.14159265f / 180.0f;
        float escape_rad = escape_angle * 3.14159265f / 180.0f;

        // X成分（横）とY成分（縦）に分解して足し算
        float total_vx = (ball_move_speed * cosf(ball_rad)) + (escape_speed * cosf(escape_rad));
        float total_vy = (ball_move_speed * sinf(ball_rad)) + (escape_speed * sinf(escape_rad));

        // 合成ベクトルから最終的な「移動速度」と「移動角度」を逆算
        float final_speed = hypotf(total_vx, total_vy);
        float final_angle = atan2f(total_vy, total_vx) * 180.0f / 3.14159265f;

        // 速度がモーターの最大能力を超えないように制限
        if (final_speed > ESCAPE_SPEED_MAX) final_speed = ESCAPE_SPEED_MAX;

        // 合成された方向へオムニ駆動命令を発射！
        Drive_Omni(final_angle, final_speed, omega);
    }
    else
    {
        // -------------------------------------------------------------
        // 3. 通常時（ラインを踏んでいない）の走行処理
        // -------------------------------------------------------------
        uint8_t side_warning = Sensor_GetSideWarning();

        // 警告フラグがある、または反応個数が1個以上ある場合は事前減速[cite: 12]
        if (side_warning == 1 || line_detected_count > 0)
        {
            ball_move_speed = 300.0f;
        }

        Drive_Omni(ball_move_angle, ball_move_speed, ball_move_omega);
    }
}
