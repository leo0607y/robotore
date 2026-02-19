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

#define ZG_FILTER_ALPHA 0.28f
static float filtered_angular_velocity = 0.0f;
#define DEG_TO_RAD 0.0174532925f // PI / 180

// 角度再構成チューニング（急ぎ改善用）
#define YAW_DEADBAND_DPS 0.05f
#define YAW_GAIN_MIN 4.20f
#define YAW_GAIN_MAX 6.20f
#define YAW_GAIN_RATE_MIN_DPS 8.0f
#define YAW_GAIN_RATE_MAX_DPS 65.0f
#define YAW_SPIKE_CLAMP_DPS 120.0f
// #define YAW_BIAS_UPDATE_THRESHOLD_DPS 3.0f
// #define YAW_BIAS_ADAPT_ALPHA 0.005f

static uint32_t log_write_address;
static uint16_t log_count;
// static LogData_t data[LOG_MAX_ENTRIES]; // 既存詳細ログは一旦停止（軽量ログのみ運用）
static CompactLogData_t compact_data[LOG_MAX_ENTRIES];
uint16_t dc = 0; // extern宣言されているためstaticを削除、型をuint16_tに統一
uint16_t compact_dc = 0;
extern int lion;
extern float zg_offset;
extern bool Start_Flag;
// extern int16_t mon_rev_l; // 既存詳細ログ停止により未使用
// extern int16_t mon_rev_r; // 既存詳細ログ停止により未使用

// 10mm区間の角度誤差積算用の静的変数
static float integrated_angle = 0.0f; // 角度積算値[rad]
static bool prev_start_flag = false;
static float segment_time_s = 0.0f;
static float logged_distance_from_start_m = 0.0f;
// static float yaw_residual_bias_dps = 0.0f; // 一旦停止（過補正で角度が消えるため）

/**
 * @brief ログ機能を初期化します。
 */
void Log_Init(void)
{
	// 書き込みアドレスの初期化
	log_write_address = LOG_FLASH_START_ADDR;
	log_count = 0;
	integrated_angle = 0.0f;
	filtered_angular_velocity = 0.0f;
	dc = 0;
	prev_start_flag = false;
	segment_time_s = 0.0f;
	logged_distance_from_start_m = 0.0f;
	compact_dc = 0;
	dc = 0;
	// yaw_residual_bias_dps = 0.0f;
}

void Log_Reset(void)
{
	// ★2回目以降の走行用★ ログ関連の状態変数をリセット
	integrated_angle = 0.0f;
	filtered_angular_velocity = 0.0f;
	dc = 0;
	prev_start_flag = false;
	segment_time_s = 0.0f;
	logged_distance_from_start_m = 0.0f;
	compact_dc = 0;
	dc = 0;
	// yaw_residual_bias_dps = 0.0f;
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
	FLASH_Write_Word_S(log_write_address, data.left_encoder_count);
	log_write_address += sizeof(int32_t);
	FLASH_Write_Word_S(log_write_address, data.right_encoder_count);
	log_write_address += sizeof(int32_t);
	FLASH_Write_Word_F(log_write_address, data.angle_error_rad);
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

	uint32_t read_address = LOG_FLASH_START_ADDR + (index * sizeof(LogData_t));
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
	log_write_address = LOG_FLASH_START_ADDR;
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
 * @brief IMUの角速度と走行距離から10mm区間の角度誤差を算出し、RAMログに保存します。
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
		integrated_angle = 0.0f;
		filtered_angular_velocity = 0.0f;
		segment_time_s = 0.0f;
		logged_distance_from_start_m = 0.0f;
		// yaw_residual_bias_dps = 0.0f;
		clearDistance();
	}
	prev_start_flag = Start_Flag;

	// 1. 生の角速度（オフセット補正後）を計算
	float gyro_lsb_per_dps = IMU_GetGyroLsbPerDps();
	float raw_angular_velocity_z = ((float)zg - zg_offset) / gyro_lsb_per_dps;
	if (raw_angular_velocity_z > YAW_SPIKE_CLAMP_DPS)
	{
		raw_angular_velocity_z = YAW_SPIKE_CLAMP_DPS;
	}
	else if (raw_angular_velocity_z < -YAW_SPIKE_CLAMP_DPS)
	{
		raw_angular_velocity_z = -YAW_SPIKE_CLAMP_DPS;
	}

	// 残留バイアスの自動追従は一旦停止（旋回成分まで相殺されるため）
	if (fabsf(raw_angular_velocity_z) < YAW_DEADBAND_DPS)
	{
		raw_angular_velocity_z = 0.0f;
	}

	// 2. IIRフィルタを適用してノイズ除去（filtered_angular_velocityを更新）
	// filtered_angular_velocity = (1 - α) * pre_filtered_value + α * raw_value
	filtered_angular_velocity = (1.0f - ZG_FILTER_ALPHA) * filtered_angular_velocity + ZG_FILTER_ALPHA * raw_angular_velocity_z;

	// 3. 角速度の大きさに応じて積分ゲインを可変化
	float abs_rate_dps = fabsf(filtered_angular_velocity);
	float integration_gain = YAW_GAIN_MAX;
	if (abs_rate_dps >= YAW_GAIN_RATE_MAX_DPS)
	{
		integration_gain = YAW_GAIN_MIN;
	}
	else if (abs_rate_dps > YAW_GAIN_RATE_MIN_DPS)
	{
		float ratio = (abs_rate_dps - YAW_GAIN_RATE_MIN_DPS) / (YAW_GAIN_RATE_MAX_DPS - YAW_GAIN_RATE_MIN_DPS);
		integration_gain = YAW_GAIN_MAX + (YAW_GAIN_MIN - YAW_GAIN_MAX) * ratio;
	}

	if (!is_on_tracking_curve)
	{
		integration_gain *= 1.00f;
	}

	integrated_angle += filtered_angular_velocity * integration_gain * DEG_TO_RAD * 0.001f; // 0.001f は dt(1ms)
	segment_time_s += 0.001f;
																		 //	printf("%f\n", getDistance());

	float segment_distance = getDistance();
	if (segment_distance >= 10.0f / 1000.0f)
	{
		if (compact_dc >= LOG_MAX_ENTRIES)
		{
			return;
		}

		CompactLogData_t new_compact_log;
		new_compact_log.distance_from_start_m = logged_distance_from_start_m + segment_distance;
		new_compact_log.angle_error_rad = integrated_angle;
		compact_data[compact_dc] = new_compact_log;
		compact_dc++;
		dc = compact_dc; // 互換用カウンタ（既存参照向け）

		// ログ保存後、距離と角度をリセット
		logged_distance_from_start_m += segment_distance;
		clearDistance(); //
		integrated_angle = 0.0f;
		segment_time_s = 0.0f;
	}
}

void Log_PrintRunData_To_Serial(void)
{
	printf("Run Compact Log (RAM) %u entries:\r\n", compact_dc);
	if (compact_dc == 0)
	{
		printf("No run data in RAM.\r\n");
		return;
	}

	for (uint16_t i = 0; i < compact_dc; i++)
	{
<<<<<<< HEAD
		printf("Run %u: Dist(m): %.3f, AngleErr(rad): %.9f\r\n",
			   i,
			   compact_data[i].distance_from_start_m,
			   compact_data[i].angle_error_rad);
=======
		printf("Run %u: Dist(m): %.3f, AngleErr(rad): %.6f\r\n",
			   i,
			   data[i].distance_from_start_m,
			   data[i].angle_error_rad);
>>>>>>> 0954dc8 (Refine IMU/log pipeline and add course outline export tools)

		if ((i + 1) % 10 == 0)
		{
			HAL_Delay(50);
		}
	}

	printf("\r\n=== Run compact log output complete ===\r\n");
}

/**
 * @brief Flashメモリに保存されているログをシリアル通信で出力します。
 * bayadoが3の時に呼び出されることを想定しています。
 */
void Log_PrintData_To_Serial(void)
{
	loadFlash(start_adress_sector11, (uint8_t *)compact_data, sizeof(CompactLogData_t) * compact_dc);

<<<<<<< HEAD
	printf("Flash Compact Log Data (%d entries):\r\n", compact_dc);
	printf("Starting data output...\r\n");
	printf("This will take approximately %d seconds...\r\n", (compact_dc / 10) * 50 / 1000);
	printf("Note: Watchdog is disabled during data output.\r\n");
=======
	printf("Flash Log Data (%d entries):\r\n", dc);
>>>>>>> 0954dc8 (Refine IMU/log pipeline and add course outline export tools)

	for (uint16_t i = 0; i < compact_dc; i++)
	{
<<<<<<< HEAD
		printf("Entry %d: Dist: %.3f m, AngleErr(rad): %.9f\r\n",
			   i,
			   compact_data[i].distance_from_start_m,
			   compact_data[i].angle_error_rad);
=======
		printf("Entry %d: Dist: %.3f m, AngleErr(rad): %.6f\r\n",
			   i,
			   data[i].distance_from_start_m,
			   data[i].angle_error_rad);
>>>>>>> 0954dc8 (Refine IMU/log pipeline and add course outline export tools)

		// 10エントリごとに待機してバッファフラッシュ
		if ((i + 1) % 10 == 0)
		{
			HAL_Delay(50); // UARTバッファフラッシュ用
		}

<<<<<<< HEAD
		// 進捗表示（50エントリごと）
		if ((i + 1) % 50 == 0)
		{
			printf("--- Progress: %d/%d entries ---\r\n", i + 1, compact_dc);
		}
=======
>>>>>>> 0954dc8 (Refine IMU/log pipeline and add course outline export tools)
	}

	printf("\r\n=== All %d entries output complete ===\r\n", compact_dc);
	printf("No auto-reset will occur.\r\n");
	//	printf("Flash Log Data (%d entries):\r\n", dc);
	//	for (int abc = 0; abc < dc; abc++) {
	//		printf("Entry %d:Left %d:Right %d:CurrentRadius %.3f\n", abc,
	//				data[abc].left_encoder_count, data[abc].right_encoder_count,
	//				data[abc].curvature_radius);
	//	}
}
void WriteData()
{
	//	for (int abc = 0; abc < dc; abc++) {
	////		FLASH_Write_Word_S(log_write_address, data[abc].left_encoder_count);
	////			log_write_address += sizeof(int16_t);
	////			FLASH_Write_Word_S(log_write_address, data[abc].right_encoder_count);
	////			log_write_address += sizeof(int16_t);
	////			FLASH_Write_Word_F(log_write_address, data[abc].curvature_radius);
	////			log_write_address += sizeof(float);
	//
	//		writeFlash(start_adress_sector11, (uint8_t*) data,
	//				sizeof(LogData_t) * 1000);
	//		printf("%d\n", abc);
	//	}
	eraseFlash(); // セクター11を消去
	writeFlash(start_adress_sector11, (uint8_t *)compact_data, sizeof(CompactLogData_t) * compact_dc);
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
