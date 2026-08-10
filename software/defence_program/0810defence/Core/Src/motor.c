/*
 * motor.c
 *
 *  Created on: May 30, 2026
 *      Author: tomo-
 */

#include "motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

void Omni_Drive(float angle, float speed, float omega)
{
    float rad = angle * 3.14159265f / 180.0f;
    float w[4];
    w[0] = speed * sinf(rad - 3.14159265f * 45.0f  / 180.0f) + omega;
    w[1] = speed * sinf(rad - 3.14159265f * 135.0f / 180.0f) + omega;
    w[2] = speed * sinf(rad - 3.14159265f * 225.0f / 180.0f) + omega;
    w[3] = speed * sinf(rad - 3.14159265f * 315.0f / 180.0f) + omega;

    float max_val = 0.0f;
    for (int i = 0; i < 4; i++) {
        float abs_w = (w[i] < 0) ? -w[i] : w[i];
        if (abs_w > max_val) max_val = abs_w;
    }
    if (max_val > 1000.0f)
        for (int i = 0; i < 4; i++) w[i] = w[i] * 1000.0f / max_val;

    for (int i = 0; i < 4; i++)
        SetMotorSpeed(i + 1, (int16_t)w[i]);
}

void SetMotorSpeed(uint8_t motor_id, int16_t speed)
{
    uint32_t pwm_val = 0;
    uint8_t  dir = 1;
    if (speed < 0) { dir = 0; pwm_val = (uint32_t)(-speed); }
    else            {          pwm_val = (uint32_t)( speed); }

    const uint32_t DEAD_ZONE = 5;
    const uint32_t MIN_PWM   = 260;
    if (pwm_val < DEAD_ZONE) pwm_val = 0;
    else pwm_val += MIN_PWM;
    if (pwm_val > 999) pwm_val = 999;

    switch (motor_id) {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, dir ? pwm_val : 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dir ? 0 : pwm_val);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dir ? pwm_val : 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dir ? 0 : pwm_val);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, dir ? pwm_val : 0);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, dir ? 0 : pwm_val);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, dir ? pwm_val : 0);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, dir ? 0 : pwm_val);
            break;
    }
}
