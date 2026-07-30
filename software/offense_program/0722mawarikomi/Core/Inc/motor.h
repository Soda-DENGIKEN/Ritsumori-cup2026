#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

void Drive_Omni(float degrees, float speed, float turn_power);
void SetMotorSpeed(uint8_t motor_id, int16_t speed);

#endif /* __MOTOR_H */
