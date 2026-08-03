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

// motor.hの宣言と合わせるため static を削除
void SetMotorSpeed(uint8_t motor_id, int16_t speed);

/**
  * @brief  4輪オムニホイール走行関数 (旋回力最優先スケーリング版)
  * @param  degrees: 移動方向 (度)
  * @param  speed: 移動速度
  * @param  turn_power: 旋回力 (omega)
  */
void Drive_Omni(float degrees, float speed, float turn_power)
{
    float rad = degrees * 3.14159265f / 180.0f;
    float w_move[4];

    // 1. まず「移動成分だけ」を仮計算する（turn_powerはまだ足さない）
    w_move[0] = speed * sinf(rad - 3.14159265f * 45.0f  / 180.0f);
    w_move[1] = speed * sinf(rad - 3.14159265f * 135.0f / 180.0f);
    w_move[2] = speed * sinf(rad - 3.14159265f * 225.0f / 180.0f);
    w_move[3] = speed * sinf(rad - 3.14159265f * 315.0f / 180.0f);

    // 2. 移動成分の中で、一番パワーが大きいモーターの絶対値を探す
    float max_move = 0.0f;
    for (int i = 0; i < 4; i++) {
        float abs_move = (w_move[i] < 0) ? -w_move[i] : w_move[i];
        if (abs_move > max_move) max_move = abs_move;
    }

    // 3. 【ここがコアロジック！】ジャイロ（旋回）の枠を絶対に死守する
    // モーターの限界値「1000」から、旋回が絶対に使いたいパワーを最初に差し引く
    float allowable_max_move = 1000.0f - fabsf(turn_power);

    // 万が一、旋回パワーだけで1000を超えそうな場合は移動枠を0にする
    if (allowable_max_move < 0.0f) allowable_max_move = 0.0f;

    // 4. 移動パワーが許容枠を超えていたら、進む向き（比率）を保ったまま全体を縮小する
    if (max_move > allowable_max_move && max_move > 0.0f) {
        float scale = allowable_max_move / max_move;
        for (int i = 0; i < 4; i++) {
            w_move[i] *= scale;
        }
    }

    // 5. 確保しておいた安全な残り枠の中に、満を持して最優先の旋回力を足し算する
    float w_final[4];
    for (int i = 0; i < 4; i++) {
        w_final[i] = w_move[i] + turn_power;
    }

    // 各モーターへ割り振り出力
    for (int i = 0; i < 4; i++) {
        SetMotorSpeed(i + 1, (int16_t)w_final[i]);
    }
}

/**
  * @brief  個別モーターのPWMピン出力制御関数
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
