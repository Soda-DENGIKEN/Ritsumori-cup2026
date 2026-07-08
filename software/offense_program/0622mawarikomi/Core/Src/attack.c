//直線だけめっちゃ速い
#include "attack.h"
#include "sensor.h"
#include "motor.h"
#include <math.h>

#define BASE_SPEED           500
#define ESCAPE_SPEED         350
#define ESCAPE_SPEED_PUSHED  550

static float ball_angle_filtered = 0.0f;
static float move_dir_filtered   = 0.0f;
static float ball_angle_prev     = 0.0f;

#define ATTACK_BLUE   0
#define ATTACK_YELLOW 1

volatile uint8_t attack_goal_color = ATTACK_BLUE;

void Attack_Update(float omega)
{
    // ライン検出 → 脱出処理（最優先）
    float escape_angle;
    if (Sensor_GetEscapeAngle(&escape_angle))
    {
        float speed = line_pushed_out ? ESCAPE_SPEED_PUSHED : ESCAPE_SPEED;
        Omni_Drive(escape_angle, speed, omega);
        return;
    }

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

        // ===== 45°付近 =====
        if (abs_dir > 35.0f && abs_dir < 55.0f)
        {
            float target_dir;

            if (ball_dir > 0.0f)
                target_dir = 135.0f;
            else
                target_dir = -135.0f;

            move_dir = 0.3f * move_dir + 0.7f * target_dir;
        }

        // ===== 90°付近 =====
        else if (abs_dir >= 55.0f && abs_dir < 125.0f)
        {
            move_dir = 0.3f * move_dir + 0.7f * 180.0f;
        }

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

        // ボールが正面付近なら700まで加速
        if (abs_dir < 15.0f)
        {
            speed_factor = 700.0f / BASE_SPEED;
        }

        // サイドセンサーのみ反応 → 早期警告で減速
        if (Sensor_GetSideWarning())
            speed_factor *= 0.6f;



        float goal_angle;
        uint8_t goal_detected;

        if (attack_goal_color == ATTACK_BLUE)
        {
            goal_angle = goal_blue_angle;
            goal_detected = goal_blue_detected;
        }
        else
        {
            goal_angle = goal_yellow_angle;
            goal_detected = goal_yellow_detected;
        }

        if (fabsf(ball_angle) < 15.0f && goal_detected)
        {
            float attack_dir;

            attack_dir = 0.7f * ball_angle + 0.3f * goal_angle;

            Omni_Drive(attack_dir, BASE_SPEED, omega);
            return;
        }

        // 通常の回り込み
        Omni_Drive(move_dir_filtered,
                   BASE_SPEED * speed_factor,
                   omega * omega_scale);
    }
    else
    {
        ball_angle_filtered = 0.0f;
        move_dir_filtered   = 0.0f;
        ball_angle_prev     = 0.0f;

        Omni_Drive(0.0f, 0.0f, omega);
    }
}
