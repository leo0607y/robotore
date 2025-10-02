#ifndef INC_LOG_H_
#define INC_LOG_H_

#include "main.h"
#include "flash2.h"

// ログデータを格納する構造体
typedef struct {
    int16_t left_encoder_count;
    int16_t right_encoder_count;
    float curvature_radius; // 追加：曲率半径
} LogData_t;

// Flashメモリのログ領域設定（STM32F405VGを想定）
// Flashメモリマップを参照し、未使用のセクタを指定してください。
// 例: Sector 5（アドレス 0x08020000）
#define LOG_FLASH_SECTOR        FLASH_SECTOR_5
#define LOG_FLASH_START_ADDR    (0x08020000U)
#define LOG_MAX_ENTRIES         (4000)  // 保存できるログエントリの最大数

// 関数プロトタイプ宣言
void Log_Init(void);
void Log_SaveData(LogData_t data);
void Log_ReadData(LogData_t *data, uint16_t index);
void Log_Erase(void);
uint16_t Log_GetCount(void);
void Log_CalculateAndSave(void);
void Log_PrintData_To_Serial(void);
void WriteData(void);
void Log_Test_Write(void); // 新規追加
void Log_Test_Read_And_Print(void); // 新規追加

#endif /* INC_LOG_H_ */
