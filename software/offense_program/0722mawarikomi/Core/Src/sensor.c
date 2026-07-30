/*
 * sensor.c
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

static uint8_t           line_rx_byte;
static uint8_t           line_rx_buf[UART_BUF_SIZE];
static volatile uint16_t line_rx_head = 0;
static volatile uint16_t line_rx_tail = 0;

// ★新設：原因特定のための診断用カウンター
volatile uint32_t debug_err_header   = 0;
volatile uint32_t debug_err_footer   = 0;
volatile uint32_t debug_err_checksum = 0;
volatile uint32_t debug_ok_packet    = 0;

// 押し出し検知用
static float   line_angle_base  = 0.0f;
static uint8_t line_valid_prev  = 0;

// ---- BNO055 ----
volatile float yaw_offset   = 0.0f;

// ---- PID ----
static float pid_integral = 0.0f;
static float pid_prev_err = 0.0f;

// ---- ライン基板通信ステート ＆ 自己診断カウンター ----
volatile uint8_t  line_calib_state  = 0;
volatile uint32_t line_packet_count = 0;

// ---- 内部関数 ----
void ParseIRData(char *line);
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
        line_packet_count++; // 割り込みに入った総合回数

        uint16_t next = (line_rx_head + 1) % UART_BUF_SIZE;
        if (next != line_rx_tail)
        {
            line_rx_buf[line_rx_head] = line_rx_byte;
            line_rx_head = next;
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
    // ---- IRセンサの解析 ----
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

    // ---- ラインセンサーの解析 (診断カウンター付きスライド窓パース) ----
    while (((line_rx_head + UART_BUF_SIZE - line_rx_tail) % UART_BUF_SIZE) >= LINE_PKT_SIZE)
    {
        uint16_t t_idx = line_rx_tail;

        // 1. 先頭がヘッダー(0xFF)かチェック
        if (line_rx_buf[t_idx] != LINE_PKT_HEADER)
        {
            debug_err_header++; // ヘッダーエラーをカウント
            line_rx_tail = (line_rx_tail + 1) % UART_BUF_SIZE;
            continue;
        }

        // 9バイト分を一時配列へコピー
        uint8_t work_pkt[LINE_PKT_SIZE];
        for (int i = 0; i < LINE_PKT_SIZE; i++)
        {
            work_pkt[i] = line_rx_buf[(line_rx_tail + i) % UART_BUF_SIZE];
        }

        // 2. 末尾がフッター(0xFE)かチェック
        if (work_pkt[8] != LINE_PKT_FOOTER)
        {
            debug_err_footer++; // フッターエラーをカウント
            line_rx_tail = (line_rx_tail + 1) % UART_BUF_SIZE;
            continue;
        }

        // 3. チェックサム検証 (1〜6バイト目のXOR)
        uint8_t checksum = work_pkt[1] ^ work_pkt[2] ^ work_pkt[3]
                         ^ work_pkt[4] ^ work_pkt[5] ^ work_pkt[6];
        if (checksum != work_pkt[7])
        {
            debug_err_checksum++; // チェックサムエラーをカウント
            line_rx_tail = (line_rx_tail + 1) % UART_BUF_SIZE;
            continue;
        }

        // --- 🎉 すべての検証に完全成功したパケット ---
        debug_ok_packet++;

        uint8_t  flags       = work_pkt[1];
        uint16_t sbits       = ((uint16_t)work_pkt[2] << 8) | work_pkt[3];
        int16_t  angle_q     = (int16_t)(((uint16_t)work_pkt[4] << 8) | work_pkt[5]);
        uint8_t  calib_s     = work_pkt[6];

        line_on_line     = flags & 0x01;
        line_sensor_bits = sbits;
        line_angle       = (float)angle_q / 10.0f;
        line_confidence  = 1.0f;
        line_calib_state = calib_s;

        line_side_back  = (sbits >> 12) & 0x01;
        line_side_left  = (sbits >> 13) & 0x01;
        line_side_front = (sbits >> 14) & 0x01;
        line_side_right = (sbits >> 15) & 0x01;

        line_data_valid = 1;

        // パケットサイズ分バッファを進める
        line_rx_tail = (line_rx_tail + LINE_PKT_SIZE) % UART_BUF_SIZE;
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

uint8_t Sensor_GetEscapeAngle(float *escape_angle)
{
    if (!line_on_line)
    {
        line_pushed_out = 0;
        line_valid_prev = 0;
        return 0;
    }

    float dir = line_angle;

    if (!line_valid_prev)
    {
        line_angle_base = dir;
        line_valid_prev = 1;
    }
    else
    {
        float diff = fabsf(AngleDiff(dir, line_angle_base));
        if (diff >= 150.0f)
            line_pushed_out = 1;
    }

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

uint8_t Sensor_GetSideWarning(void)
{
    if (line_on_line) return 0;
    if (line_side_front || line_side_back || line_side_left || line_side_right) return 1;
    return 0;
}

void Sensor_ResetYawOnly(void)
{
    yaw_offset = BNO055_GetYaw();
}

void Sensor_ResumeLineRx(void)
{
    line_rx_head = 0;
    line_rx_tail = 0;

    huart2.Lock    = HAL_UNLOCKED;
    huart2.gState  = HAL_UART_STATE_READY;
    huart2.RxState = HAL_UART_STATE_READY;

    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_PEFLAG(&huart2);

    HAL_UART_Receive_IT(&huart2, &line_rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        Sensor_ResumeLineRx();
    }
}
