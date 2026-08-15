#ifndef DEFENSE_H
#define DEFENSE_H

/* main.cのループから毎周期呼び出す。
 * omegaはmain.c側で Sensor_GetOmega(0.0f, 1) により
 * yaw=0固定のPIDとして計算したものをそのまま渡す。
 * 例:
 *   float omega = Sensor_GetOmega(0.0f, 1);
 *   Defense_Update(omega);
 */
void Defense_Update(float omega);
void Defense_ResetLateralPosition(void);

#endif /* DEFENSE_H */
