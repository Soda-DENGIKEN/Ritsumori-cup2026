/*
 * sensor.h
 *
 *  Created on: May 30, 2026
 *      Author: tomo-
 */

#ifndef SENSOR_H
#define SENSOR_H

#include "main.h"

// IRセンサ
extern volatile float   ball_angle;
extern volatile float   ball_strength;
extern volatile uint8_t ball_detected;

// ゴールカメラ
extern volatile float   goal_yellow_angle;
extern volatile float   goal_blue_angle;
extern volatile uint8_t goal_yellow_detected;
extern volatile uint8_t goal_blue_detected;

// ラインセンサー
extern volatile uint8_t  line_on_line;
extern volatile uint16_t line_sensor_bits;
extern volatile float    line_angle;
extern volatile float    line_confidence;
extern volatile uint8_t  line_side_front;
extern volatile uint8_t  line_side_back;
extern volatile uint8_t  line_side_left;
extern volatile uint8_t  line_side_right;
extern volatile uint8_t  line_data_valid;
extern volatile uint8_t  line_pushed_out;

void Sensor_InitYawOffset(void);
void Sensor_Update(void);
float Sensor_GetOmega(float goal_angle, uint8_t goal_detected);
void ParseIRData(char *line);
void BNO055_Init(void);
float BNO055_GetYaw(void);
float PID_Update(float error);
uint8_t Sensor_GetEscapeAngle(float *escape_angle);
uint8_t Sensor_GetSideWarning(void);

#endif /* INC_SENSOR_H_ */
