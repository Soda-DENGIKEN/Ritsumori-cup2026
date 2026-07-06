/*元のセンサー
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

// ---- IRセンサ ----
volatile float   ball_angle    = 0.0f;
volatile float   ball_strength = 0.0f;
volatile uint8_t ball_detected = 0;

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

static uint8_t line_rx_byte;
static uint8_t line_pkt[LINE_PKT_SIZE];
static uint8_t line_pkt_idx    = 0;
static uint8_t line_collecting = 0;

// 押し出し検知用
static float   line_angle_base  = 0.0f;  // 最初にラインを踏んだときの角度
static uint8_t line_valid_prev  = 0;

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

                        sbits &= ~(1 << 9);
                        int16_t  angle_q     = (int16_t)(((uint16_t)line_pkt[4] << 8) | line_pkt[5]);
                        uint8_t  conf_q      = line_pkt[6];

                        line_on_line     = flags & 0x01;
                        line_sensor_bits = sbits;
                        line_angle       = (float)angle_q / 10.0f;
                        line_confidence  = (float)conf_q / 255.0f;

                        line_side_back  = (sbits >> 12) & 0x01;
                        line_side_left  = (sbits >> 13) & 0x01;
                        line_side_front = (sbits >> 14) & 0x01;
                        line_side_right = (sbits >> 15) & 0x01;

                        line_data_valid = 1;
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
uint8_t Sensor_GetEscapeAngle(float *escape_angle)
{
    if (!line_on_line)
    {
        // ライン離れたらリセット
        line_pushed_out = 0;
        line_valid_prev = 0;
        return 0;
    }

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

    *escape_angle = escape;
    return 1;
}

void ParseIRData(char *line)
{
    char *comma = strchr(line, ',');
    if (comma == NULL) return;
    *comma = '\0';
    float angle = strtof(line, NULL);
    if (angle >= 400.0f)
        ball_detected = 0;
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
