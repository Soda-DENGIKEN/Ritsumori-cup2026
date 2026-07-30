#ifndef __SENSOR_H
#define __SENSOR_H

#include "main.h"

// ---- 関数プロトタイプ宣言 ----
void BNO055_Init(void);
float BNO055_GetYaw(void);
void Sensor_InitYawOffset(void);
void Sensor_Update(void);
float Sensor_GetOmega(float goal_angle, uint8_t goal_detected);
float PID_Update(float error);
void Sensor_ResetYawOnly(void);
void Sensor_ResumeLineRx(void);

// ★ライン回避で使う重要関数
uint8_t Sensor_GetEscapeAngle(float *escape_angle);
uint8_t Sensor_GetSideWarning(void);

// ---- 外部参照変数 (extern) ----
extern volatile float   ball_angle;
extern volatile float   ball_strength;
extern volatile uint8_t ball_detected;

extern volatile float   goal_yellow_angle;
extern volatile float   goal_blue_angle;
extern volatile uint8_t goal_yellow_detected;
extern volatile uint8_t goal_blue_detected;

extern volatile uint8_t  line_on_line;
extern volatile uint16_t line_sensor_bits;
extern volatile float    line_angle;
extern volatile float    line_confidence;
extern volatile uint8_t  line_side_front;
extern volatile uint8_t  line_side_back;
extern volatile uint8_t  line_side_left;
extern volatile uint8_t  line_side_right;
extern volatile uint8_t  line_data_valid;
extern volatile uint8_t  line_pushed_out; // ★押し出されたフラグ

extern volatile uint8_t  line_calib_state;
extern volatile uint32_t line_packet_count;

#endif /* __SENSOR_H */
