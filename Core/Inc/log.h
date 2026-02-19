#ifndef INC_LOG_H_
#define INC_LOG_H_

#include "main.h"
#include "flash2.h"

// 既存詳細ログ（現在は停止中。互換のため定義は残す）
typedef struct
{
    float distance_from_start_m;
    float speed_m_s;
    float target_speed_m_s;
    int16_t pwm_left;
    int16_t pwm_right;
    int16_t left_encoder_count;
    int16_t right_encoder_count;
    float angle_error_rad; // 10mm区間で積算した角度誤差[rad]
} LogData_t;

// 現在運用中の軽量ログ
typedef struct
{
    float distance_from_start_m;
    float angle_error_rad;
} CompactLogData_t;

// Flashメモリのログ領域設定（STM32F405RGを想定）
// Flashメモリマップを参照し、未使用のセクタを指定してください。
// Sector 11（アドレス 0x080E0000、128KB）- プログラム領域から最も離れた安全な位置
#define LOG_FLASH_SECTOR FLASH_SECTOR_11
#define LOG_FLASH_START_ADDR (0x080E0000U)
#define LOG_MAX_ENTRIES (4000) // 保存できるログエントリの最大数

// グローバル変数のextern宣言
extern uint16_t dc; // ログエントリ数のカウンタ
extern uint16_t compact_dc;

// 関数プロトタイプ宣言
void Log_Init(void);
void Log_Reset(void);
void Log_SaveData(LogData_t data);
void Log_ReadData(LogData_t *data, uint16_t index);
void Log_Erase(void);
uint16_t Log_GetCount(void);
void Log_CalculateAndSave(void);
void Log_PrintRunData_To_Serial(void);
void Log_PrintData_To_Serial(void);
void WriteData(void);
void Log_Test_Write(void);          // 新規追加
void Log_Test_Read_And_Print(void); // 新規追加

#endif /* INC_LOG_H_ */
