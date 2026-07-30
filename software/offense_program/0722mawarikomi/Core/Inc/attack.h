#ifndef __ATTACK_H
#define __ATTACK_H

#include "main.h"

// ゴール色設定用のグローバル変数
#define ATTACK_BLUE   0
#define ATTACK_YELLOW 1
extern volatile uint8_t attack_goal_color;

void Attack_Update(float omega);

#endif /* __ATTACK_H */
