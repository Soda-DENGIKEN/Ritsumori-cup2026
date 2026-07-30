/*
 * flash_store.c
 */

#include "flash_store.h"
#include <string.h>

// セクタ消去用構造体 (F446用にSector 7を指定)
static void Erase_Flash_Sector(void)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector        = FLASH_SECTOR_7; // STM32F446の最終セクタ7
    EraseInitStruct.NbSectors     = 1;

    // 💡 【重要】消去の直前に過去のエラーフラグをすべてクリア
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
}

// データの保存
void Flash_SaveData(CalibrationData *data)
{
    // 1. 書き込むためにフラッシュのロックを解除
    HAL_FLASH_Unlock();

    // 💡 書き込みプロセス開始時にもフラグをクリア
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    // 2. 既存のデータをセクタ単位で消去 (書き込み前に必須)
    Erase_Flash_Sector();

    // 3. 32bit(Word)単位で構造体データを書き込む
    uint32_t *data_ptr = (uint32_t *)data;
    uint32_t size_in_words = sizeof(CalibrationData) / 4;
    if (sizeof(CalibrationData) % 4 != 0) size_in_words++;

    for (uint32_t i = 0; i < size_in_words; i++) {
        // 💡 確実に書き込ませるため、各ワードの書き込み前にもフラグをクリア
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                               FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SAVE_ADDR + (i * 4), data_ptr[i]);
    }

    // 4. フラッシュを再ロックして保護
    HAL_FLASH_Lock();
}

// データの読み出し
void Flash_LoadData(CalibrationData *data)
{
    // フラッシュメモリのアドレスから直接構造体にデータをコピー
    memcpy(data, (void *)FLASH_SAVE_ADDR, sizeof(CalibrationData));
}
