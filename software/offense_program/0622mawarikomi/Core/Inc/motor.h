/*
 * motor.h
 *
 *  Created on: May 30, 2026
 *      Author: tomo-
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include <math.h>

void SetMotorSpeed(uint8_t motor_id, int16_t speed);
void Omni_Drive(float angle, float speed, float omega);

#endif /* INC_MOTOR_H_ */
