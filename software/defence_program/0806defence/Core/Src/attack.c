#include "attack.h"
#include "sensor.h"
#include "motor.h"
#include <math.h>

#define BASE_SPEED          500
#define ESCAPE_SPEED         350
#define ESCAPE_SPEED_PUSHED  550

static float ball_angle_filtered = 0.0f;
static float move_dir_filtered   = 0.0f;
static float ball_angle_prev     = 0.0f;

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

        // 先読み補正
        float ball_angular_vel = ball_angle - ball_angle_prev;

        while(ball_angular_vel > 180.0f)
            ball_angular_vel -= 360.0f;

        while(ball_angular_vel < -180.0f)
            ball_angular_vel += 360.0f;

        ball_angle_prev = ball_angle;
        if (fabsf(ball_angular_vel) < 40.0f)
            move_dir += ball_angular_vel * 3.0f;
        while(move_dir > 180.0f)
            move_dir -= 360.0f;

        while(move_dir < -180.0f)
            move_dir += 360.0f;

        // ローパスで滑らかに
        move_dir_filtered = 0.6f * move_dir_filtered + 0.4f * move_dir;

        float t = abs_dir / 180.0f;
        float speed_factor = 1.0f + 0.5f * t * t;
        float omega_scale = 1.0f - 0.6f * (abs_dir / 180.0f);

        // サイドセンサーのみ反応 → 早期警告で減速
        if (Sensor_GetSideWarning())
            speed_factor *= 0.6f;

        Omni_Drive(move_dir_filtered, BASE_SPEED * speed_factor, omega * omega_scale);
    }
    else
    {
        ball_angle_filtered = 0.0f;
        move_dir_filtered   = 0.0f;
        ball_angle_prev     = 0.0f;
        Omni_Drive(0.0f, 0.0f, omega);
    }
}
