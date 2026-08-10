/*
 * sensor.c
 *
 *  Created on: May 30, 2026
 *      Author: tomo-
 */

#include "sensor.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

extern I2C_HandleTypeDef  hi2c2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;

#define BNO055_ADDR      (0x28 << 1)
#define BNO055_OPR_MODE  0x3D
#define BNO055_NDOF_MODE 0x08
#define BNO055_EUL_H_LSB 0x1A
#define CONTROL_DT       0.01f
#define MAX_OMEGA        1000.0f
#define UART_BUF_SIZE    256
#define GOAL_PKT_SIZE    10
#define GOAL_NOT_DET     ((uint16_t)0x7FFF)
#define LINE_PKT_SIZE    9
#define LINE_PKT_HEADER  0xFFu
#define LINE_PKT_FOOTER  0xFEu

// ---- クラスタ判定用サイドフラグ ----
#define CLUSTER_SIDE_FRONT 0x01
#define CLUSTER_SIDE_BACK  0x02
#define CLUSTER_SIDE_LEFT  0x04
#define CLUSTER_SIDE_RIGHT 0x08

// ---- 遷移(斜め通過中)判定 ----
#define TRANSITION_THRESHOLD_DEG  15.0f  /* 直近5サンプルの合計変化量がこれを超えたら「動いている」 */

// ---- IRセンサ ----
volatile float   ball_angle    = 0.0f;
volatile float   ball_strength = 0.0f;
volatile uint8_t ball_detected = 0;
volatile uint8_t line_calib_state = 0;

static uint8_t           rx_byte;
static uint8_t           rx_buf[UART_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static char              line_buf[64];
static uint8_t           line_len = 0;

// ---- ゴールカメラ ----
volatile float   goal_yellow_angle    = 0.0f;
volatile float   goal_blue_angle      = 0.0f;
volatile uint8_t goal_yellow_detected = 0;
volatile uint8_t goal_blue_detected   = 0;

static uint8_t goal_rx_byte;
static uint8_t goal_pkt[GOAL_PKT_SIZE];
static uint8_t goal_pkt_idx    = 0;
static uint8_t goal_collecting = 0;

// ---- ラインセンサー ----
volatile uint8_t  line_on_line       = 0;
volatile uint16_t line_sensor_bits   = 0;
volatile float    line_angle         = 0.0f;
volatile float    line_confidence    = 0.0f;
volatile uint8_t  line_side_front    = 0;
volatile uint8_t  line_side_back     = 0;
volatile uint8_t  line_side_left     = 0;
volatile uint8_t  line_side_right    = 0;
volatile uint8_t  line_data_valid    = 0;
volatile uint8_t  line_pushed_out    = 0;
volatile uint8_t  line_detected_count = 0; // 💡 追加！

// 💡【追加】クラスタ判定結果
volatile uint8_t  line_cluster_count      = 0;      // 検出されたクラスタ数(0〜2)
volatile float    line_cluster_angle[2]   = {0,0};   // 各クラスタの代表角度
volatile uint8_t  line_cluster_side[2]    = {0,0};   // 各クラスタに関与したサイドセンサー
                                                       // bit0=前(A14) bit1=後(A12) bit2=左(A13) bit3=右(A15)

// 💡【追加】主たる接触角度と、時間変化による遷移(斜め通過中)判定
volatile float   line_primary_angle     = 0.0f;
volatile uint8_t line_is_transitioning  = 0;
// 💡【追加】100msデバッグ表示では捕捉しきれない短時間の遷移を見逃さないためのstickyフラグと、
// 実際のラインパケット受信レートを実測するためのカウンタ。どちらもmain.c側で
// デバッグ出力の直後にリセットする想定（sticky/counterとも「直近リセットからの累積」）。
volatile uint8_t  line_was_transitioning = 0;
volatile uint16_t line_packet_count      = 0;

static float   primary_angle_hist[5]     = {0,0,0,0,0};
static uint8_t primary_angle_hist_idx    = 0;
static uint8_t primary_angle_hist_count  = 0;

static uint8_t line_rx_byte;
static uint8_t line_pkt[LINE_PKT_SIZE];
static uint8_t line_pkt_idx    = 0;
static uint8_t line_collecting = 0;

// 押し出し検知用
static float   line_angle_base  = 0.0f;  // 最初にラインを踏んだときの角度
static uint8_t line_valid_prev  = 0;

// 脱出タイマー＆角度保持用 (💡 一時無効化のため未使用化)
static uint32_t escape_timer_start = 0;
static float    last_escape_angle  = 0.0f;

// ---- BNO055 ----
static float yaw_offset   = 0.0f;

// ---- PID ----
static float pid_integral = 0.0f;
static float pid_prev_err = 0.0f;

// ---- 内部関数 ----
static float AngleDiff(float a, float b)
{
    float diff = a - b;
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

// 💡【追加】ラインクラスタ検出用のビン定義
// A0〜A11(メインリング, 30°間隔)とA12〜A15(サイド)を角度順の12ビンにまとめる
typedef struct {
    float   angle;
    int8_t  main_bit;   // メインリングのビット番号
    int8_t  side_bit;   // サイドセンサーのビット番号(-1ならなし)
    uint8_t side_flag;  // このビンのサイドセンサーが立った時に付与するフラグ
} LineBin;

static const LineBin line_bins[12] = {
    /* i=0 */  {  180.0f,  0, 12, CLUSTER_SIDE_BACK  },  // A0
    /* i=1 */  { -150.0f,  1, -1, 0 },                   // A1
    /* i=2 */  { -120.0f,  2, -1, 0 },                   // A2
    /* i=3 */  {  -90.0f,  3, 13, CLUSTER_SIDE_LEFT  },  // A3
    /* i=4 */  {  -60.0f,  4, -1, 0 },                   // A4
    /* i=5 */  {  -30.0f,  5, -1, 0 },                   // A5
    /* i=6 */  {    0.0f,  6, 14, CLUSTER_SIDE_FRONT },  // A6
    /* i=7 */  {   30.0f,  7, -1, 0 },                   // A7
    /* i=8 */  {   60.0f,  8, -1, 0 },                   // A8
    /* i=9 */  {   90.0f,  9, 15, CLUSTER_SIDE_RIGHT },  // A9
    /* i=10 */ {  120.0f, 10, -1, 0 },                   // A10
    /* i=11 */ {  150.0f, 11, -1, 0 },                   // A11
};

// 💡【追加】sbits(16bit生データ)からクラスタ数・代表角度・関与サイドセンサーを計算する
// 円環上で連続する反応ビンを1方向にまとめ、最大2クラスタまで検出する
static void ComputeLineClusters(uint16_t sbits)
{
    uint8_t active[12];
    for (int i = 0; i < 12; i++)
    {
        uint8_t main_on = (sbits >> line_bins[i].main_bit) & 0x01;
        uint8_t side_on = (line_bins[i].side_bit >= 0) &&
                           ((sbits >> line_bins[i].side_bit) & 0x01);
        active[i] = main_on || side_on;
    }

    uint8_t visited[12] = {0};
    uint8_t cluster_count = 0;
    float   cluster_sin[2] = {0,0};
    float   cluster_cos[2] = {0,0};
    uint8_t cluster_side[2] = {0,0};

    uint8_t all_active = 1;
    for (int i = 0; i < 12; i++) if (!active[i]) { all_active = 0; break; }

    for (int start = 0; start < 12 && cluster_count < 2; start++)
    {
        if (!active[start] || visited[start]) continue;
        int prev = (start - 1 + 12) % 12;
        if (active[prev] && !visited[prev] && !all_active) continue;

        float sin_sum = 0.0f, cos_sum = 0.0f;
        uint8_t side_flag = 0;
        int idx = start;
        int steps = 0;
        while (active[idx] && !visited[idx] && steps < 12)
        {
            visited[idx] = 1;
            float rad = line_bins[idx].angle * 3.14159265f / 180.0f;
            sin_sum += sinf(rad);
            cos_sum += cosf(rad);
            uint8_t side_on = (line_bins[idx].side_bit >= 0) &&
                               ((sbits >> line_bins[idx].side_bit) & 0x01);
            if (side_on) side_flag |= line_bins[idx].side_flag;
            idx = (idx + 1) % 12;
            steps++;
        }

        cluster_sin[cluster_count]  = sin_sum;
        cluster_cos[cluster_count]  = cos_sum;
        cluster_side[cluster_count] = side_flag;
        cluster_count++;
    }

    line_cluster_count = cluster_count;
    for (int k = 0; k < 2; k++)
    {
        if (k < cluster_count)
        {
            line_cluster_angle[k] = atan2f(cluster_sin[k], cluster_cos[k]) * 180.0f / 3.14159265f;
            line_cluster_side[k]  = cluster_side[k];
        }
        else
        {
            line_cluster_angle[k] = 0.0f;
            line_cluster_side[k]  = 0;
        }
    }
}

// 💡【追加】主たる接触角度(line_primary_angle)を求め、直近5サンプルの
// 角度変化量から「今動いている最中(斜め通過中)かどうか」を判定する
static void UpdateLinePrimaryAngle(void)
{
    float primary;

    if (line_cluster_count == 0)
    {
        primary = primary_angle_hist_count > 0
                 ? primary_angle_hist[(primary_angle_hist_idx + 4) % 5]
                 : 0.0f;
    }
    else if (line_cluster_count == 1)
    {
        primary = line_cluster_angle[0];
    }
    else
    {
        float d0 = fabsf(fabsf(line_cluster_angle[0]) - 90.0f);
        float d1 = fabsf(fabsf(line_cluster_angle[1]) - 90.0f);
        primary = (d0 <= d1) ? line_cluster_angle[0] : line_cluster_angle[1];
    }

    line_primary_angle = primary;

    primary_angle_hist[primary_angle_hist_idx] = primary;
    primary_angle_hist_idx = (primary_angle_hist_idx + 1) % 5;
    if (primary_angle_hist_count < 5) primary_angle_hist_count++;

    if (primary_angle_hist_count >= 5)
    {
        float total_change = 0.0f;
        for (int i = 0; i < 4; i++)
        {
            uint8_t a = (primary_angle_hist_idx + i) % 5;
            uint8_t b = (primary_angle_hist_idx + i + 1) % 5;
            total_change += fabsf(AngleDiff(primary_angle_hist[b], primary_angle_hist[a]));
        }
        line_is_transitioning = (total_change > TRANSITION_THRESHOLD_DEG) ? 1 : 0;
    }
    else
    {
        line_is_transitioning = 0;
    }

    // 💡【追加】100ms表示のリセットまでの間に一度でも1になったら立てっぱなしにする
    if (line_is_transitioning) line_was_transitioning = 1;
}

// ---- UART受信割り込み ----
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // IRセンサ（USART1）
    if (huart->Instance == USART1)
    {
        uint16_t next = (rx_head + 1) % UART_BUF_SIZE;
        if (next != rx_tail)
        {
            rx_buf[rx_head] = rx_byte;
            rx_head = next;
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }

    // ゴールカメラ（UART5）
    if (huart->Instance == UART5)
    {
        uint8_t b = goal_rx_byte;
        if (!goal_collecting)
        {
            if (b == 0xAA)
            {
                goal_pkt[0]     = b;
                goal_pkt_idx    = 1;
                goal_collecting = 1;
            }
        }
        else
        {
            if (goal_pkt_idx < GOAL_PKT_SIZE)
                goal_pkt[goal_pkt_idx++] = b;
            else
                goal_collecting = 0;

            if (goal_pkt_idx >= GOAL_PKT_SIZE)
            {
                goal_collecting = 0;
                goal_pkt_idx    = 0;
                if (goal_pkt[9] == 0xFF)
                {
                    uint16_t y_raw   = ((uint16_t)goal_pkt[1] << 8) | goal_pkt[2];
                    uint16_t y_d_raw = ((uint16_t)goal_pkt[3] << 8) | goal_pkt[4];
                    uint16_t b_raw   = ((uint16_t)goal_pkt[5] << 8) | goal_pkt[6];
                    uint16_t b_d_raw = ((uint16_t)goal_pkt[7] << 8) | goal_pkt[8];
                    (void)y_d_raw; (void)b_d_raw;
                    if (y_raw == GOAL_NOT_DET) goal_yellow_detected = 0;
                    else { goal_yellow_angle = (float)(int16_t)y_raw; goal_yellow_detected = 1; }
                    if (b_raw == GOAL_NOT_DET) goal_blue_detected = 0;
                    else { goal_blue_angle = (float)(int16_t)b_raw; goal_blue_detected = 1; }
                }
            }
        }
        HAL_UART_Receive_IT(&huart5, &goal_rx_byte, 1);
    }

    // ラインセンサー（USART2）
    if (huart->Instance == USART2)
    {
        uint8_t b = line_rx_byte;
        if (!line_collecting)
        {
            if (b == LINE_PKT_HEADER)
            {
                line_pkt[0]     = b;
                line_pkt_idx    = 1;
                line_collecting = 1;
            }
        }
        else
        {
            if (line_pkt_idx < LINE_PKT_SIZE)
                line_pkt[line_pkt_idx++] = b;
            else
                line_collecting = 0;

            if (line_pkt_idx >= LINE_PKT_SIZE)
            {
                line_collecting = 0;
                line_pkt_idx    = 0;
                if (line_pkt[8] == LINE_PKT_FOOTER)
                {
                    uint8_t checksum = line_pkt[1] ^ line_pkt[2] ^ line_pkt[3]
                                     ^ line_pkt[4] ^ line_pkt[5] ^ line_pkt[6];
                    if (checksum == line_pkt[7])
                    {
                        uint8_t  flags       = line_pkt[1];
                        uint16_t sbits       = ((uint16_t)line_pkt[2] << 8) | line_pkt[3];
                        int16_t  angle_q     = (int16_t)(((uint16_t)line_pkt[4] << 8) | line_pkt[5]);
                        uint8_t  c_state     = line_pkt[6];

                        uint8_t new_line_on  = flags & 0x01;

                        // 💡【追加】反応しているラインセンサの個数を計算 (メインリング12個分: sbits & 0x0FFF)
                            uint16_t main_bits = sbits & 0x0FFF;
                            uint8_t  count     = 0;
                            while (main_bits > 0)
                            {
                                count += (main_bits & 1);
                                main_bits >>= 1;
                            }
                            line_detected_count = count; // 反応数を更新

                        // 💡【一時無効化】250ms(実質50ms)保持ロジック用の
                        // last_escape_angle / escape_timer_start 更新をコメントアウト。
                        // 回り込みへの影響を切り分けるための一時措置。
                        /*
                        if (new_line_on)
                        {
                            float dir = (float)angle_q / 10.0f;
                            float escape = dir + 180.0f;
                            if (escape >  180.0f) escape -= 360.0f;
                            if (escape < -180.0f) escape += 360.0f;

                            last_escape_angle  = escape;
                            escape_timer_start = HAL_GetTick();
                        }
                        */

                        line_on_line     = new_line_on;
                        line_sensor_bits = sbits;
                        line_angle       = (float)angle_q / 10.0f;

                        line_calib_state = c_state;
                        line_confidence  = 0.0f;

                        line_side_back  = (sbits >> 12) & 0x01;
                        line_side_left  = (sbits >> 13) & 0x01;
                        line_side_front = (sbits >> 14) & 0x01;
                        line_side_right = (sbits >> 15) & 0x01;

                        line_data_valid = 1;
                        ComputeLineClusters(sbits);   // 💡【追加】クラスタ検出
                        UpdateLinePrimaryAngle();      // 💡【追加】主たる接触角度＋遷移判定
                        line_packet_count++;           // 💡【追加】実際のパケット受信数を計測
                    }
                }
            }
        }
        HAL_UART_Receive_IT(&huart2, &line_rx_byte, 1);
    }
}

void Sensor_InitYawOffset(void)
{
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    HAL_UART_Receive_IT(&huart5, &goal_rx_byte, 1);
    HAL_UART_Receive_IT(&huart2, &line_rx_byte, 1);
    yaw_offset = BNO055_GetYaw();
}

void Sensor_Update(void)
{
    while (rx_head != rx_tail)
    {
        uint8_t b = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % UART_BUF_SIZE;
        if (b == '\n' || b == '\r')
        {
            if (line_len > 0)
            {
                line_buf[line_len] = '\0';
                ParseIRData(line_buf);
                line_len = 0;
            }
        }
        else
        {
            if (line_len < sizeof(line_buf) - 1)
                line_buf[line_len++] = (char)b;
        }
    }
}

float Sensor_GetOmega(float goal_angle, uint8_t goal_detected)
{
    float yaw    = BNO055_GetYaw();
    float target = goal_detected ? goal_angle : 0.0f;
    float error  = target - yaw;
    if (error >  180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    return PID_Update(error);
}

// 脱出方向計算 + 押し出し検知
// 💡【一時無効化】250ms(実質50ms)保持ロジックを無効化。
// line_on_line が0になった瞬間、即座に0を返す（＝前のバージョンの挙動に戻す）。
uint8_t Sensor_GetEscapeAngle(float *escape_angle)
{
    // 1. 現在白線を踏んでいる場合
    if (line_on_line)
    {
        float dir = line_angle;  // メインラインの角度

        if (!line_valid_prev)
        {
            // 最初にラインを踏んだときの角度を基準として記録
            line_angle_base = dir;
            line_valid_prev = 1;
        }
        else
        {
            // 基準角度からの差分が150°以上 → ラインを越えて半分以上押された
            float diff = fabsf(AngleDiff(dir, line_angle_base));
            if (diff >= 150.0f)
                line_pushed_out = 1;
        }

        // 脱出方向 = ラインの逆方向
        float escape = dir + 180.0f;
        if (escape >  180.0f) escape -= 360.0f;
        if (escape < -180.0f) escape += 360.0f;

        last_escape_angle  = escape;
        escape_timer_start = HAL_GetTick(); // 参照はしないが値は保持しておく

        *escape_angle = escape;
        return 1;
    }

    // 💡【一時無効化】ここが「250ms(実質50ms)保持」部分。丸ごとコメントアウト。
    /*
    if (escape_timer_start != 0 && (now - escape_timer_start < 50))
    {
        *escape_angle = last_escape_angle;
        return 1;
    }
    */

    // 2. ラインを離れたら即座にリセット（＝前のバージョンと同じ挙動）
    line_pushed_out    = 0;
    line_valid_prev    = 0;
    escape_timer_start = 0;
    return 0;
}

void ParseIRData(char *line)
{
    char *comma = strchr(line, ',');
    if (comma == NULL) return;
    *comma = '\0';
    float angle = strtof(line, NULL);

    if (angle >= 400.0f)
    {
        ball_detected = 0;
        ball_angle    = 0.0f;  // 見失った時は 0.0 にリセット
        ball_strength = 0.0f;  // 強度も 0.0 にリセット
    }
    else
    {
        ball_angle    = angle;
        ball_strength = strtof(comma + 1, NULL);
        ball_detected = 1;
    }
}

void BNO055_Init(void)
{
    HAL_Delay(700);
    uint8_t mode = BNO055_NDOF_MODE;
    HAL_I2C_Mem_Write(&hi2c2, BNO055_ADDR,
                      BNO055_OPR_MODE, I2C_MEMADD_SIZE_8BIT,
                      &mode, 1, 100);
    HAL_Delay(20);
}

float BNO055_GetYaw(void)
{
    uint8_t buf[2];
    HAL_I2C_Mem_Read(&hi2c2, BNO055_ADDR,
                     BNO055_EUL_H_LSB, I2C_MEMADD_SIZE_8BIT,
                     buf, 2, 100);
    int16_t raw    = (int16_t)(buf[0] | (buf[1] << 8));
    float   yaw    = (float)raw / 16.0f;
    float   result = yaw - yaw_offset;
    if (result >  180.0f) result -= 360.0f;
    if (result < -180.0f) result += 360.0f;
    return result;
}

float PID_Update(float error)
{
    if (error > -3.5f && error < 3.5f)
    {
        pid_integral = 0.0f;
        return 0.0f;
    }
    const float Kp      = 2.0f;
    const float Ki      = 0.0f;
    const float Kd      = 0.0f;
    const float I_LIMIT = 200.0f;

    pid_integral += error * CONTROL_DT;
    if (pid_integral >  I_LIMIT) pid_integral =  I_LIMIT;
    if (pid_integral < -I_LIMIT) pid_integral = -I_LIMIT;

    float derivative = (error - pid_prev_err) / CONTROL_DT;
    pid_prev_err     = error;

    float output = Kp * error + Ki * pid_integral + Kd * derivative;
    if (output >  MAX_OMEGA) output =  MAX_OMEGA;
    if (output < -MAX_OMEGA) output = -MAX_OMEGA;
    return output;
}

// サイドセンサーのみ反応（メイン未反応）→ 早期警告
uint8_t Sensor_GetSideWarning(void)
{
    if (line_on_line) return 0;  // メイン反応中は脱出処理に任せる

    if (line_side_front || line_side_back ||
        line_side_left  || line_side_right)
        return 1;

    return 0;
}

// 現在の向きを強制的に正面（ゼロ）にリセットする関数
void Sensor_ResetYawOnly(void)
{
    yaw_offset += BNO055_GetYaw();
}

// ライン基板からのUART受信割り込みを再スタートさせる関数
void Sensor_ResumeLineRx(void)
{
    HAL_UART_Receive_IT(&huart2, &line_rx_byte, 1);
}

uint8_t Sensor_GetSideEscapeAngle(float *escape_angle)
{
    // メインラインを踏んでいる時は、メインラインの脱出処理に任せる
    if (line_on_line) return 0;

    float vx = 0.0f;
    float vy = 0.0f;

    if (line_side_front) vy -= 1.0f; // 前にライン ➔ 後ろ(-Y)へ回避
    if (line_side_back)  vy += 1.0f; // 後ろにライン ➔ 前(+Y)へ回避
    if (line_side_right) vx -= 1.0f; // 右にライン ➔ 左(-X)へ回避
    if (line_side_left)  vx += 1.0f; // 左にライン ➔ 右(+X)へ回避

    // どのサブラインも反応していなければ 0
    if (vx == 0.0f && vy == 0.0f) return 0;

    // ベクトルから逃げ角度（正面=0度）を算出
    float rad = atan2f(vx, vy);
    *escape_angle = rad * 180.0f / 3.14159265f;

    return 1;
}
