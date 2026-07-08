/*
 * attack.h
 *
 *  Created on: May 30, 2026
 *      Author: tomo-
 */
#ifndef ATTACK_H
#define ATTACK_H

#include "main.h"

#define ATTACK_BLUE   0
#define ATTACK_YELLOW 1

extern volatile uint8_t attack_goal_color;

#include "main.h"

void Attack_Update(float omega);

#endif /* INC_ATTACK_H_ */

#define ATTACK_BLUE   0
#define ATTACK_YELLOW 1

extern volatile uint8_t attack_goal_color;
