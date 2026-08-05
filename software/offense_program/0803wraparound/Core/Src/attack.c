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

// 💡【追加】進行方向とセンサー方向の一致判定用
static float AngleDiff(float a, float b)
{
    float diff = a - b;
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

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
        avoid_counter = 40;

        float escape_angle;
        uint8_t got_angle = 0;

        // ① メインリングが実際に踏んでいる場合はこちらを優先（従来通り line_angle 基準）
        if (line_on_line)
        {
            escape_angle = line_angle + 180.0f;
            if (escape_angle > 180.0f) escape_angle -= 360.0f;
            got_angle = 1;
        }
        // ② サイドセンサーのみが早期反応している場合はベクトル計算を使う
        else
        {
            got_angle = Sensor_GetSideEscapeAngle(&escape_angle);
        }

        avoid_direction = got_angle ? escape_angle : (line_angle + 180.0f);
        if (avoid_direction > 180.0f) avoid_direction -= 360.0f;

        // 速度判定はメインリングの反応数(line_detected_count)を使う方が正確
        // （line_countだとサイドセンサーの分まで一緒に数えてしまう）
        const float FRONT_FORCE_HEAVY_WINDOW = 30.0f;
        uint8_t approaching_front =
            fabsf(AngleDiff(move_dir_filtered, 0.0f)) < FRONT_FORCE_HEAVY_WINDOW;

        if (approaching_front || line_detected_count >= 4) {
            avoid_speed = AVOID_SPEED_HEAVY;
        }
        else if (line_detected_count >= 2) {
            avoid_speed = AVOID_SPEED_MID;
        }
        else {
            avoid_speed = AVOID_SPEED_LIGHT;
        }
    }

    // -------------------------------------------------------------
    // 3. 回避動作の実行（40回消化するまでループ）
    // -------------------------------------------------------------
    if (avoid_counter > 0)
    {
        avoid_counter--; // 1ループ通過ごとに1減らす

        // 💡【ボール追従を確実に止める】
        // ここで return するため、下のボール追従ブロック(ステップ4)は
        // 一切実行されない＝回避中はボール方向に一切反応しない。
        //
        // ただし ball_angle_prev だけは更新し続けておく。
        // これを更新しないと、回避が終わった瞬間に
        // 「回避前 のボール角度」と「回避後の現在のボール角度」の差分が
        // 大きくなり、先読み補正(角速度項)が過剰に効いて
        // 回避直後に急にボール方向へ振られる（＝再びライン側へ寄る）
        // 原因になるため。
        ball_angle_prev = ball_angle;

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

        // 💡【進行方向と一致するサイドセンサーだけ強くブレーキ】
        // 一律0.6倍だった早期警告ブレーキをやめて、
        // 「今まさに進もうとしている方向 (move_dir_filtered)」に
        // 一致する側のセンサーが反応しているときだけ強く減速する。
        // これにより、ボールが正面にあってもライン方向へ
        // 近づいていない限りフルスピードを維持できる。
        const float SIDE_BRAKE_FACTOR = 0.3f;   // 一致時の速度倍率
        const float SIDE_BRAKE_WINDOW = 60.0f;  // 一致とみなす角度幅(±度)

        if (line_side_front && fabsf(AngleDiff(move_dir_filtered, 0.0f))   < SIDE_BRAKE_WINDOW)
            speed_factor *= SIDE_BRAKE_FACTOR;
        if (line_side_back  && fabsf(AngleDiff(move_dir_filtered, 180.0f)) < SIDE_BRAKE_WINDOW)
            speed_factor *= SIDE_BRAKE_FACTOR;
        if (line_side_left  && fabsf(AngleDiff(move_dir_filtered, 90.0f))  < SIDE_BRAKE_WINDOW)
            speed_factor *= SIDE_BRAKE_FACTOR;
        if (line_side_right && fabsf(AngleDiff(move_dir_filtered, -90.0f)) < SIDE_BRAKE_WINDOW)
            speed_factor *= SIDE_BRAKE_FACTOR;

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
