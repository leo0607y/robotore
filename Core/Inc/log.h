/*
 * log.h
 *
 * Created on: Aug 22, 2025
 * Author: reoch
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

#include "main.h"

// ログデータを格納する構造体
typedef struct {
    float left_velocity;
    float right_velocity;
    int32_t left_encoder_count;
    int32_t right_encoder_count;
    float curvature_radius; // 追加：曲率半径
} LogData_t;

// Flashメモリのログ領域設定（STM32F405VGを想定）
// Flashメモリマップを参照し、未使用のセクタを指定してください。
// 例: Sector 5（アドレス 0x08020000）
#define LOG_FLASH_SECTOR        FLASH_SECTOR_5
#define LOG_FLASH_START_ADDR    (0x08020000U)
#define LOG_MAX_ENTRIES         (1000)  // 保存できるログエントリの最大数

// 関数プロトタイプ宣言
void Log_Init(void);
void Log_SaveData(LogData_t data);
void Log_ReadData(LogData_t *data, uint16_t index);
void Log_Erase(void);
uint16_t Log_GetCount(void);
void Log_CalculateAndSave(void);      // 新規：曲率半径を計算して保存
void Log_PrintData_To_Serial(void);   // 新規：シリアル出力

#endif /* INC_LOG_H_ */
