/*
 * log.c
 *
 *  Created on: Aug 22, 2025
 *      Author: reoch
 */


#include "log.h"
#include <string.h>

static uint32_t log_write_address;
static uint16_t log_count;

/**
 * @brief ログ機能を初期化します。
 */
void Log_Init(void) {
    // 書き込みアドレスの初期化
    log_write_address = LOG_FLASH_START_ADDR;
    log_count = 0;
}

/**
 * @brief ログデータをFlashメモリに保存します。
 * @param data 保存するログデータ
 */
void Log_SaveData(LogData_t data) {
    if (log_count >= LOG_MAX_ENTRIES) {
        // ログバッファが満杯
        return;
    }

    HAL_FLASH_Unlock();

    // 構造体を64ビットデータとして書き込む
    // Flashメモリの書き込み単位は64bitなので、この処理が必要です
    uint64_t *data_ptr = (uint64_t *)&data;
    uint8_t num_words = sizeof(LogData_t) / sizeof(uint64_t);

    for (uint8_t i = 0; i < num_words; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, log_write_address, data_ptr[i]);
        log_write_address += sizeof(uint64_t);
    }

    HAL_FLASH_Lock();

    // ログエントリ数を更新
    log_count++;
}

/**
 * @brief Flashメモリから指定したインデックスのログデータを読み出します。
 * @param data 読み出したデータの格納先ポインタ
 * @param index 読み出すデータのインデックス
 */
void Log_ReadData(LogData_t *data, uint16_t index) {
    if (index >= log_count) {
        // インデックスが無効
        return;
    }

    uint32_t read_address = LOG_FLASH_START_ADDR + (index * sizeof(LogData_t));
    memcpy(data, (void *)read_address, sizeof(LogData_t));
}

/**
 * @brief Flashメモリのログ領域を全て消去します。
 */
void Log_Erase(void) {
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;

    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector       = LOG_FLASH_SECTOR;
    EraseInitStruct.NbSectors    = 1;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    HAL_FLASH_Lock();

    // 状態をリセット
    log_write_address = LOG_FLASH_START_ADDR;
    log_count = 0;
}

/**
 * @brief 保存されているログの数を取得します。
 * @return ログの数
 */
uint16_t Log_GetCount(void) {
    return log_count;
}
