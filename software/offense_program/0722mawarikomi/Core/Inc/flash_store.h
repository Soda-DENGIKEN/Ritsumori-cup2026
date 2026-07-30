#ifndef __FLASH_STORE_H
#define __FLASH_STORE_H

#include "main.h"

// STM32F446の最終セクタ7のアドレス
#define FLASH_SAVE_ADDR  0x08060000

// 💡 電源を切っても保存したいデータをここに定義
typedef struct {
    uint8_t  is_calibrated; // キャリブレーション完了フラグ (1なら完了)
    float    yaw_offset;    // ジャイロのオフセット値
} CalibrationData;

void Flash_SaveData(CalibrationData *data);
void Flash_LoadData(CalibrationData *data);

#endif /* __FLASH_STORE_H */
