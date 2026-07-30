/*
 * motor.c
 *
 * Created on: May 30, 2026
 * Author: tomo-
 */

#include "motor.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

// 💡 修正：motor.hの宣言と合わせるため static を削除
void SetMotorSpeed(uint8_t motor_id, int16_t speed);

/**
  * @brief  4輪オムニホイール走行関数 (main.cからの呼び出し用)
  * @param  degrees: 移動方向 (度)
  * @param  speed: 移動速度
  * @param  turn_power: 旋回力 (omega)
  */
void Drive_Omni(float degrees, float speed, float turn_power)
{
    float rad = degrees * 3.14159265f / 180.0f;
    float w[4];

    // ホイール配置ベクトルに合わせたsin計算
    w[0] = speed * sinf(rad - 3.14159265f * 45.0f  / 180.0f) + turn_power;
    w[1] = speed * sinf(rad - 3.14159265f * 135.0f / 180.0f) + turn_power;
    w[2] = speed * sinf(rad - 3.14159265f * 225.0f / 180.0f) + turn_power;
    w[3] = speed * sinf(rad - 3.14159265f * 315.0f / 180.0f) + turn_power;

    float max_val = 0.0f;
    for (int i = 0; i < 4; i++) {
        float abs_w = (w[i] < 0) ? -w[i] : w[i];
        if (abs_w > max_val) max_val = abs_w;
    }

    // 最大値が1000を超えた場合のスケーリング
    if (max_val > 1000.0f) {
        for (int i = 0; i < 4; i++) {
            w[i] = w[i] * 1000.0f / max_val;
        }
    }

    // 各モーターへ割り振り出力
    for (int i = 0; i < 4; i++) {
        SetMotorSpeed(i + 1, (int16_t)w[i]);
    }
}

/**
  * @brief  個別モーターのPWMピン出力制御関数
  * 💡 修正：実体定義から static を削除して公開関数化
  */
void SetMotorSpeed(uint8_t motor_id, int16_t speed)
{
    uint32_t pwm_val = 0;
    uint8_t  dir = 1;
    if (speed < 0) {
        dir = 0;
        pwm_val = (uint32_t)(-speed);
    } else {
        pwm_val = (uint32_t)(speed);
    }

    const uint32_t DEAD_ZONE = 5;
    const uint32_t MIN_PWM   = 260;

    if (pwm_val < DEAD_ZONE) {
        pwm_val = 0;
    } else {
        pwm_val += MIN_PWM;
    }

    if (pwm_val > 999) pwm_val = 999;

    switch (motor_id) {
        case 1: // 前右
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, dir ? pwm_val : 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dir ? 0 : pwm_val);
            break;
        case 2: // 後右
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dir ? pwm_val : 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dir ? 0 : pwm_val);
            break;
        case 3: // 後左
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, dir ? pwm_val : 0);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, dir ? 0 : pwm_val);
            break;
        case 4: // 前左
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, dir ? pwm_val : 0);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, dir ? 0 : pwm_val);
            break;
    }
}
