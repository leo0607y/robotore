/*
 * log.c
 *
 * Created on: Aug 22, 2025
 * Author: reoch
 */

#include "log.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>	  // rand()関数を使用するために追加
#include "Flash.h"	  //
#include "IMU20649.h" //
#include "Encoder.h"  //
#include "TrackingPart.h"
#include "flash2.h"

#define MIN_COURSE_RADIUS_M (0.10f) // R10cm（コースの最小曲率半径 - 確定値）
#define ANGLE_EPS_RAD (0.002f)      // 10mm区間での角度がこれ未満なら直線扱い

#define ZG_FILTER_ALPHA 0.2037f
static float filtered_angular_velocity = 0.0f;
#define DEG_TO_RAD 0.0174532925f // PI / 180
#define GYRO_SCALE 1.0f           // 角速度補正係数（実測で調整）

static uint32_t log_write_address;
static uint16_t log_count;
static LogData_t data[LOG_MAX_ENTRIES];
static float debug_omega_rad_s[LOG_MAX_ENTRIES];
static float debug_v_m_s[LOG_MAX_ENTRIES];
static float debug_target_m_s[LOG_MAX_ENTRIES];
static float debug_pwm_abs[LOG_MAX_ENTRIES];
uint16_t dc = 0; // extern宣言されているためstaticを削除、型をuint16_tに統一
extern int lion;
extern float zg_offset;
extern bool Start_Flag;

// 曲率半径計算用の静的変数
static float integrated_angle = 0.0f; // 角度積算値[rad]
static bool prev_start_flag = false;
static float segment_time_s = 0.0f;
static float omega_sum_rad = 0.0f;
static uint32_t omega_samples = 0;

/**
 * @brief ログ機能を初期化します。
 */
void Log_Init(void)
{
	// 書き込みアドレスの初期化
	log_write_address = LOG_FLASH_START_ADDR + LOG_FLASH_HEADER_SIZE;
	log_count = 0;
	integrated_angle = 0.0f;
	filtered_angular_velocity = 0.0f;
	dc = 0;
	prev_start_flag = false;
	segment_time_s = 0.0f;
	omega_sum_rad = 0.0f;
	omega_samples = 0;
}

void Log_Reset(void)
{
	// ★2回目以降の走行用★ ログ関連の状態変数をリセット
	integrated_angle = 0.0f;
	filtered_angular_velocity = 0.0f;
	dc = 0;
	prev_start_flag = false;
	segment_time_s = 0.0f;
	omega_sum_rad = 0.0f;
	omega_samples = 0;
}

/**
 * @brief ログデータをFlashメモリに保存します。
 * @param data 保存するログデータ
 */
void Log_SaveData(LogData_t data)
{
	if (log_count >= LOG_MAX_ENTRIES)
	{
		// ログバッファが満杯
		return;
	}

	// Flashにログデータを書き込む
	// 個々の書き込み関数がロック/アンロックを管理するため、ここでは行わない
	FLASH_Write_Word_F(log_write_address, data.curvature_radius);
	log_write_address += sizeof(float);

	// ログエントリ数を更新
	log_count++;
}

/**
 * @brief Flashメモリから指定したインデックスのログデータを読み出します。
 * @param data 読み出したデータの格納先ポインタ
 * @param index 読み出すデータのインデックス
 */
void Log_ReadData(LogData_t *data, uint16_t index)
{
	if (index >= log_count)
	{
		// インデックスが無効
		return;
	}

	uint32_t read_address = LOG_FLASH_START_ADDR + LOG_FLASH_HEADER_SIZE + (index * sizeof(LogData_t));
	memcpy(data, (void *)read_address, sizeof(LogData_t));
}

/**
 * @brief Flashメモリのログ領域を全て消去します。
 */
void Log_Erase(void)
{
	// Flash.cのFLASH_EreaseSector関数を利用
	FLASH_EreaseSector(LOG_FLASH_SECTOR);

	// 状態をリセット
	log_write_address = LOG_FLASH_START_ADDR + LOG_FLASH_HEADER_SIZE;
	log_count = 0;
	integrated_angle = 0.0f;
}

/**
 * @brief 保存されているログの数を取得します。
 * @return ログの数
 */
uint16_t Log_GetCount(void)
{
	return log_count;
}

/**
 * @brief IMUの角速度と走行距離から曲率半径を計算し、Flashメモリに保存します。
 * この関数は、1msごとに呼び出されることを想定しています。
 */
void Log_CalculateAndSave(void)
{
	if (!Start_Flag)
	{
		prev_start_flag = false;
		return;
	}

	if (!prev_start_flag && Start_Flag)
	{
		// Start marker detected: align integration with start position
		integrated_angle = 0.0f;
		filtered_angular_velocity = 0.0f;
		segment_time_s = 0.0f;
		omega_sum_rad = 0.0f;
		omega_samples = 0;
		clearDistance();
	}
	prev_start_flag = Start_Flag;

	// 1. 生の角速度（オフセット補正後）を計算
	float raw_angular_velocity_z = ((float)zg - zg_offset) / GYRO_SENS_LSB_PER_DPS;
	raw_angular_velocity_z *= GYRO_SCALE;

	// 2. IIRフィルタを適用してノイズ除去（filtered_angular_velocityを更新）
	// filtered_angular_velocity = (1 - α) * pre_filtered_value + α * raw_value
	filtered_angular_velocity = (1.0f - ZG_FILTER_ALPHA) * filtered_angular_velocity + ZG_FILTER_ALPHA * raw_angular_velocity_z;

	// 3. 角度積算には平滑化された値を使用
	{
		float omega_rad_s = filtered_angular_velocity * DEG_TO_RAD;
		integrated_angle += omega_rad_s * 0.001f; // 0.001f は dt(1ms)
		segment_time_s += 0.001f;
		omega_sum_rad += omega_rad_s;
		omega_samples++;
	}
																		 //	printf("%f\n", getDistance());

	float segment_distance = getDistance();
	if (segment_distance >= 10.0f / 1000.0f)
	{
		LogData_t new_log;
 		float avg_omega_rad_s = 0.0f;
 		float avg_v_m_s = 0.0f;

		if (dc >= LOG_MAX_ENTRIES)
		{
			return;
		}

		if (omega_samples > 0)
		{
			avg_omega_rad_s = omega_sum_rad / (float)omega_samples;
		}
		if (segment_time_s > 0.0f)
		{
			avg_v_m_s = segment_distance / segment_time_s;
		}

		if (fabsf(integrated_angle) > ANGLE_EPS_RAD)
		{
			float calculated_radius = segment_distance / integrated_angle;
			if (fabsf(calculated_radius) < MIN_COURSE_RADIUS_M)
			{
				calculated_radius = copysignf(MIN_COURSE_RADIUS_M, calculated_radius);
			}
			new_log.curvature_radius = calculated_radius;
		}
		else
		{
			new_log.curvature_radius = 0.0f; // integrated_angle が極小（厳密な直線）
		}

		//        Log_SaveData(new_log);
		data[dc] = new_log;
		debug_omega_rad_s[dc] = avg_omega_rad_s;
		debug_v_m_s[dc] = avg_v_m_s;
		debug_target_m_s[dc] = getTarget();
		debug_pwm_abs[dc] = getDebugPwmAbsMax();

		// ログ保存後、距離と角度をリセット
		clearDistance(); //
		integrated_angle = 0.0f;
		segment_time_s = 0.0f;
		omega_sum_rad = 0.0f;
		omega_samples = 0;
		dc++;
	}
}

/**
 * @brief 走行中にRAMへ保存したomegaとvをシリアル出力します。
 * bayadoが0の時に呼び出されることを想定しています。
 */
void Log_PrintDebug_To_Serial(void)
{
	printf("Debug Log (omega, v) %u entries:\r\n", dc);
	if (dc == 0)
	{
		printf("No debug data in RAM.\r\n");
		return;
	}

	for (uint16_t i = 0; i < dc; i++)
	{
		float dist_m = (float)i * 0.01f;
		printf("Dbg %u: Dist(m): %.3f, Omega(rad/s): %.4f, V(m/s): %.3f, Target: %.3f, PWM: %.0f\r\n",
			   i, dist_m, debug_omega_rad_s[i], debug_v_m_s[i], debug_target_m_s[i], debug_pwm_abs[i]);

		if ((i + 1) % 10 == 0)
		{
			HAL_Delay(50);
		}
	}

	printf("\r\n=== Debug output complete ===\r\n");
}

/**
 * @brief Flashメモリに保存されているログをシリアル通信で出力します。
 * bayadoが3の時に呼び出されることを想定しています。
 */
void Log_PrintData_To_Serial(void)
{
	uint32_t stored_count = *(__IO uint32_t *)LOG_FLASH_START_ADDR;
	printf("Header count: %lu\r\n", stored_count);
	if (stored_count == 0xFFFFFFFFU)
	{
		stored_count = 0;
	}
	if (stored_count > LOG_MAX_ENTRIES)
	{
		stored_count = LOG_MAX_ENTRIES;
	}

	if (stored_count == 0)
	{
		printf("Flash Log Data (0 entries):\r\n");
		printf("No data found in Flash.\r\n");
		return;
	}

	// Flashデータを一度だけ読み込む（ループの外）
	loadFlash(LOG_FLASH_START_ADDR + LOG_FLASH_HEADER_SIZE, (uint8_t *)data, sizeof(LogData_t) * stored_count);

	printf("Flash Log Data (%lu entries):\r\n", stored_count);
	printf("Starting data output...\r\n");
	printf("This will take approximately %lu seconds...\r\n", (stored_count / 10) * 50 / 1000);
	printf("Note: Watchdog is disabled during data output.\r\n");

	for (uint16_t i = 0; i < stored_count; i++)
	{
		float dist_m = (float)i * 0.01f;
		printf("Entry %d: Dist(m): %.3f, Curvature Radius: %.3f\r\n",
			   i, dist_m, data[i].curvature_radius);

		// 10エントリごとに待機してバッファフラッシュ
		if ((i + 1) % 10 == 0)
		{
			HAL_Delay(50); // UARTバッファフラッシュ用
		}

		// 進捗表示（50エントリごと）
		if ((i + 1) % 50 == 0)
		{
			printf("--- Progress: %d/%lu entries ---\r\n", i + 1, stored_count);
		}
	}

	printf("\r\n=== All %lu entries output complete ===\r\n", stored_count);
	printf("No auto-reset will occur.\r\n");
	//	printf("Flash Log Data (%d entries):\r\n", dc);
	//	for (int abc = 0; abc < dc; abc++) {
	//		printf("Entry %d: Dist(m): %.3f, Curvature Radius: %.3f\n", abc,
	//				(float)abc * 0.01f, data[abc].curvature_radius);
	//	}
}
void WriteData()
{
	uint32_t write_address = LOG_FLASH_START_ADDR;

	FLASH_EreaseSector(LOG_FLASH_SECTOR);
	FLASH_Write_Word(write_address, (uint32_t)dc);
	write_address += LOG_FLASH_HEADER_SIZE;

	for (uint16_t i = 0; i < dc; i++)
	{
		FLASH_Write_Word_F(write_address, data[i].curvature_radius);
		write_address += sizeof(float);
	}

	printf("OK\n");
	// lion = 7; // リセット防止のためコメントアウト
}
/**
 * @brief テスト用関数：Flashにランダムな数値を書き込みます。
 */
void Log_Test_Write(void)
{
	uint32_t test_address = LOG_FLASH_START_ADDR;
	srand(HAL_GetTick()); // 乱数のシードを初期化
	printf("Writing random numbers to Flash...\r\n");

	for (int i = 0; i < 10; i++)
	{
		uint32_t random_value = rand();
		FLASH_Write_Word(test_address, random_value);
		test_address += sizeof(uint32_t);
	}
	printf("Writing complete.\r\n");
}

/**
 * @brief テスト用関数：Flashから数値を読み出してシリアルに出力します。
 */
void Log_Test_Read_And_Print(void)
{
	uint32_t test_address = LOG_FLASH_START_ADDR;
	uint32_t read_value;
	printf("Reading from Flash...\r\n");

	for (int i = 0; i < 10; i++)
	{
		read_value = *(__IO uint32_t *)test_address;
		printf("Address 0x%lX: 0x%lX\r\n", test_address, read_value);
		test_address += sizeof(uint32_t);
	}
	printf("Reading complete.\r\n");
}

void Log_Test_WritePattern(uint16_t count)
{
	if (count > LOG_MAX_ENTRIES)
	{
		count = LOG_MAX_ENTRIES;
	}

	for (uint16_t i = 0; i < count; i++)
	{
		data[i].curvature_radius = 0.1f * (float)i;
	}

	dc = count;
	WriteData();
}

void Log_Test_ReadPattern(void)
{
	Log_PrintData_To_Serial();
}
