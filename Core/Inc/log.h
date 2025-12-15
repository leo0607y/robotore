#ifndef INC_LOG_H_
#define INC_LOG_H_

#include "main.h"
#include "flash2.h"

// ログデータを格納する構造体
typedef struct
{
    int16_t left_encoder_count;
    int16_t right_encoder_count;
    float curvature_radius; // 追加：曲率半径
} LogData_t;

// Flashメモリのログ領域設定（STM32F405RGを想定）
// Flashメモリマップを参照し、未使用のセクタを指定してください。
// Sector 11（アドレス 0x080E0000、128KB）- プログラム領域から最も離れた安全な位置
#define LOG_FLASH_SECTOR FLASH_SECTOR_11
#define LOG_FLASH_START_ADDR (0x080E0000U)
#define LOG_MAX_ENTRIES (4000) // 保存できるログエントリの最大数

// グローバル変数のextern宣言
extern uint16_t dc; // ログエントリ数のカウンタ

// 関数プロトタイプ宣言
void Log_Init(void);
void Log_SaveData(LogData_t data);
void Log_ReadData(LogData_t *data, uint16_t index);
void Log_Erase(void);
uint16_t Log_GetCount(void);
void Log_CalculateAndSave(void);
void Log_PrintData_To_Serial(void);
void WriteData(void);
void Log_Test_Write(void);          // 新規追加
void Log_Test_Read_And_Print(void); // 新規追加

#endif /* INC_LOG_H_ */
