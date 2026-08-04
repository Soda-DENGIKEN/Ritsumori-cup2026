#include "attack.h"
#include "sensor.h"
#include "motor.h"
#include "main.h"
#include <math.h>

// 💡【回避スピード設定（生の数値）】
#define AVOID_SPEED_HEAVY   600.0f   // 4個以上：深く踏んだ時の回避スピード数値
#define AVOID_SPEED_MID     400.0f   // 2〜3個：普通に踏んだ時の回避スピード数値
#define AVOID_SPEED_LIGHT   300.0f   // 1個：かすめた時の回避スピード数値

// 💡【通常走行スピード設定】
#define BASE_SPEED          500.0f   // 通常追従速度

// 状態保持用の静的変数（ライン回避）
static uint8_t avoid_counter   = 0;    // 残り回避ループ数
static float   avoid_direction = 0.0f; // 逃げる方向 (角度)
static float   avoid_speed     = 0.0f; // 逃げる速度 (生の数値)

// 状態保持用の静的変数（ボール追従）
static float ball_angle_filtered = 0.0f;
static float move_dir_filtered   = 0.0f;
static float ball_angle_prev     = 0.0f;

// 外部参照（sensor.c等から参照されている変数を明示）
extern volatile uint16_t line_sensor_bits;
extern volatile float    line_angle;

void Attack_Update(float omega)
{
    // -------------------------------------------------------------
    // 1. 反応しているラインセンサーの「個数」をカウント
    // -------------------------------------------------------------
    uint8_t line_count = 0;
    for (int i = 0; i < 16; i++) {
        if (line_sensor_bits & (1u << i)) {
            line_count++;
        }
    }

    // -------------------------------------------------------------
    // 2. 新しくラインを検知した瞬間：生数値の割り当て ＆ 40回分のカウンタセット
    // -------------------------------------------------------------
    if (line_count > 0 && avoid_counter == 0)
    {
        // 回避時間を「40ループ分」で固定
        avoid_counter = 40;

        // 逃げる方向：ライン角度の真逆（+180度）
        avoid_direction = line_angle + 180.0f;
        if (avoid_direction > 180.0f) {
            avoid_direction -= 360.0f;
        }

        // 💡【コア処理】反応個数に応じた「速度数値」の場合分け
        if (line_count >= 4) {
            avoid_speed = AVOID_SPEED_HEAVY; // 深く踏んだ時（例: 900）
        }
        else if (line_count >= 2) {
            avoid_speed = AVOID_SPEED_MID;   // 普通に踏んだ時（例: 600）
        }
        else {
            avoid_speed = AVOID_SPEED_LIGHT; // 1個だけかすめた時（例: 300）
        }
    }

    // -------------------------------------------------------------
    // 3. 回避動作の実行（40回消化するまでループ）
    // -------------------------------------------------------------
    if (avoid_counter > 0)
    {
        avoid_counter--; // 1ループ通過ごとに1減らす

        // 💡 Motor_MoveAngle から Omni_Drive に修正して実行
        Omni_Drive(avoid_direction, avoid_speed, omega);
        return; // 通常のアタック処理を行わずにここで抜ける
    }

    // -------------------------------------------------------------
    // 4. 通常のアタック処理（avoid_counter が 0 のとき）
    // -------------------------------------------------------------
    if (ball_detected)
    {
        // ローパスフィルタ
        ball_angle_filtered = 0.7f * ball_angle_filtered + 0.3f * ball_angle;

        float ball_dir = ball_angle_filtered;
        float abs_dir  = fabsf(ball_dir);
        float rad      = abs_dir * 3.14159265f / 180.0f;

        // sin曲線で回り込み方向計算
        float move_abs = abs_dir + 55.0f * sinf(rad * 0.8f);
        float move_dir = (ball_dir >= 0.0f) ? move_abs : -move_abs;

        // 先読み補正
        float ball_angular_vel = ball_angle - ball_angle_prev;

        while (ball_angular_vel > 180.0f)
            ball_angular_vel -= 360.0f;

        while (ball_angular_vel < -180.0f)
            ball_angular_vel += 360.0f;

        ball_angle_prev = ball_angle;

        if (fabsf(ball_angular_vel) < 40.0f)
            move_dir += ball_angular_vel * 3.0f;

        while (move_dir > 180.0f)
            move_dir -= 360.0f;

        while (move_dir < -180.0f)
            move_dir += 360.0f;

        // ローパスで滑らかに
        move_dir_filtered = 0.6f * move_dir_filtered + 0.4f * move_dir;

        float t = abs_dir / 180.0f;
        float speed_factor = 1.0f + 0.5f * t * t;
        float omega_scale  = 1.0f - 0.6f * (abs_dir / 180.0f);

        // サイドセンサーのみ反応 → 早期警告で減速
        if (Sensor_GetSideWarning())
            speed_factor *= 0.6f;

        Omni_Drive(move_dir_filtered, BASE_SPEED * speed_factor, omega * omega_scale);
    }
    else
    {
        // ボールを見失っている時
        ball_angle_filtered = 0.0f;
        move_dir_filtered   = 0.0f;
        ball_angle_prev     = 0.0f;
        Omni_Drive(0.0f, 0.0f, omega);
    }
}
