/*
 * log.c
 *
 * Created on: Aug 22, 2025
 * Author: reoch
 */

#include "log.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "Flash.h" //
#include "IMU20649.h" //
#include "Encoder.h" //

static uint32_t log_write_address;
static uint16_t log_count;

// 曲率半径計算用の静的変数
static float integrated_angle = 0.0f;     // 角度積算値[rad]

/**
 * @brief ログ機能を初期化します。
 */
void Log_Init(void) {
    // 書き込みアドレスの初期化
    log_write_address = LOG_FLASH_START_ADDR;
    log_count = 0;
    integrated_angle = 0.0f;
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

    // Flashにログデータを書き込む
    HAL_FLASH_Unlock(); // Flash.cの内部でFLASH_Unlock()が呼ばれているため、ここではHAL関数を使用
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, log_write_address, data.left_encoder_count);
    log_write_address += sizeof(int32_t);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, log_write_address, data.right_encoder_count);
    log_write_address += sizeof(int32_t);
    FLASH_Write_Word_F(log_write_address, data.curvature_radius); //
    log_write_address += sizeof(float);
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
    // Flash.cのFLASH_EreaseSector関数を利用
    FLASH_EreaseSector(LOG_FLASH_SECTOR); //

    // 状態をリセット
    log_write_address = LOG_FLASH_START_ADDR;
    log_count = 0;
    integrated_angle = 0.0f;
}

/**
 * @brief 保存されているログの数を取得します。
 * @return ログの数
 */
uint16_t Log_GetCount(void) {
    return log_count;
}

/**
 * @brief IMUの角速度と走行距離から曲率半径を計算し、Flashメモリに保存します。
 * この関数は、1msごとに呼び出されることを想定しています。
 */
void Log_CalculateAndSave(void) {
    // IMU20649.h/cで定義されているグローバル変数zgを使用
    float angular_velocity_z = (float)zg / 16.4f; // 4000dpsレンジのスケールファクター
    integrated_angle += angular_velocity_z * 0.001f; // TIM6(1ms)ごとに呼び出されると仮定

    if (getDistance10mm() >= 10.0f) {
        LogData_t new_log;

        // 既存のログデータ項目を埋める
        new_log.left_encoder_count = enc_l_total; //
        new_log.right_encoder_count = enc_r_total; //

        if (fabsf(integrated_angle) > 0.001f) {
            new_log.curvature_radius = getDistance10mm() / integrated_angle;
        } else {
            new_log.curvature_radius = 0.0f; // 直進と見なす
        }

        Log_SaveData(new_log);

        // ログ保存後、距離と角度をリセット
        clearDistance10mm(); //
        integrated_angle = 0.0f;
    }
}

/**
 * @brief Flashメモリに保存されているログをシリアル通信で出力します。
 * bayadoが3の時に呼び出されることを想定しています。
 */
void Log_PrintData_To_Serial(void) {
    LogData_t log_entry;
    printf("Flash Log Data (%d entries):\r\n", log_count);
    for (uint16_t i = 0; i < log_count; i++) {
        Log_ReadData(&log_entry, i);
        printf("Entry %d: Left Enc: %ld, Right Enc: %ld, Curvature Radius: %.2f\r\n",
               i,
               log_entry.left_encoder_count,
               log_entry.right_encoder_count,
               log_entry.curvature_radius);
    }
}
