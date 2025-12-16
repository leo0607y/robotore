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
#include "flash2.h"

#define MIN_COURSE_RADIUS_M (0.09f) // R10cm（コースの最小曲率半径 - 確定値）
// コースの最大半径が不明なため、実用上の上限を100mに設定。
// これを超える値は、走行中のノイズによる「ほぼ直線」と見なす。
#define MAX_PRACTICAL_RADIUS_M (10.0f)

#define ZG_FILTER_ALPHA 0.2037f
static float filtered_angular_velocity = 0.0f;
#define DEG_TO_RAD 0.0174532925f // PI / 180

static uint32_t log_write_address;
static uint16_t log_count;
static LogData_t data[4000];
uint16_t dc = 0; // extern宣言されているためstaticを削除、型をuint16_tに統一
extern int lion;
extern float zg_offset;

// 曲率半径計算用の静的変数
static float integrated_angle = 0.0f; // 角度積算値[rad]

/**
 * @brief ログ機能を初期化します。
 */
void Log_Init(void)
{
	// 書き込みアドレスの初期化
	log_write_address = LOG_FLASH_START_ADDR;
	log_count = 0;
	integrated_angle = 0.0f;
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
 * @brief IMUの角速度と走行距離から曲率半径を計算し、Flashメモリに保存します。
 * この関数は、1msごとに呼び出されることを想定しています。
 */
void Log_CalculateAndSave(void)
{

	// 1. 生の角速度（オフセット補正後）を計算
	float raw_angular_velocity_z = ((float)zg - zg_offset) / 16.4f;

	// 2. IIRフィルタを適用してノイズ除去（filtered_angular_velocityを更新）
	// filtered_angular_velocity = (1 - α) * pre_filtered_value + α * raw_value
	filtered_angular_velocity = (1.0f - ZG_FILTER_ALPHA) * filtered_angular_velocity + ZG_FILTER_ALPHA * raw_angular_velocity_z;

	// 3. 角度積算には平滑化された値を使用
	integrated_angle += filtered_angular_velocity * DEG_TO_RAD * 0.001f; // 0.001f は dt(1ms)
																		 //	printf("%f\n", getDistance());

	if (getDistance() >= 10.0f / 1000.0f)
	{
		LogData_t new_log;

		// 既存のログデータ項目を埋める
		new_log.left_encoder_count = enc_l_total;  //
		new_log.right_encoder_count = enc_r_total; //

		if (fabsf(integrated_angle) > 0.001f)
		{
			float calculated_radius = getDistance() / integrated_angle;
			float abs_calculated_radius = fabsf(calculated_radius); // 絶対値を取得

			// ★★★ 修正: 上限フィルタリングの追加 ★★★
			// |R| < 0.09m（最小半径未満） OR |R| > 10.0m（実用最大半径超）の場合、直線(0.0f)として記録
			if (abs_calculated_radius < MIN_COURSE_RADIUS_M || abs_calculated_radius > MAX_PRACTICAL_RADIUS_M)
			{
				new_log.curvature_radius = 0.0f; // ノイズ/実質的な直線
			}
			else
			{
				// 0.09m <= |R| <= 10.0m の有効なカーブ
				new_log.curvature_radius = calculated_radius;
			}
			// ★★★ 修正はここまで ★★★
		}
		else
		{
			new_log.curvature_radius = 0.0f; // integrated_angle が極小（厳密な直線）
		}

		//        Log_SaveData(new_log);
		data[dc] = new_log;

		// ログ保存後、距離と角度をリセット
		clearDistance(); //
		integrated_angle = 0.0f;
		dc++;
	}
}

/**
 * @brief Flashメモリに保存されているログをシリアル通信で出力します。
 * bayadoが3の時に呼び出されることを想定しています。
 */
void Log_PrintData_To_Serial(void)
{
	//    LogData_t log_entry;
	// Flashデータを一度だけ読み込む（ループの外）
	loadFlash(start_adress_sector11, (uint8_t *)data, sizeof(LogData_t) * dc);

	printf("Flash Log Data (%d entries):\r\n", dc);
	printf("Starting data output...\r\n");
	printf("This will take approximately %d seconds...\r\n", (dc / 10) * 50 / 1000);
	printf("Note: Watchdog is disabled during data output.\r\n");

	for (uint16_t i = 0; i < dc; i++)
	{
		//      Log_ReadData(&log_entry, i); // この行はコメントアウトしたまま
		printf("Entry %d: Left Enc: %d, Right Enc: %d, Curvature Radius: %.2f\r\n",
			   i, data[i].left_encoder_count, data[i].right_encoder_count, // log_entryではなくdata[i]を使用
			   data[i].curvature_radius);

		// 10エントリごとに待機してバッファフラッシュ
		if ((i + 1) % 10 == 0)
		{
			HAL_Delay(50); // UARTバッファフラッシュ用
		}

		// 進捗表示（50エントリごと）
		if ((i + 1) % 50 == 0)
		{
			printf("--- Progress: %d/%d entries ---\r\n", i + 1, dc);
		}
	}

	printf("\r\n=== All %d entries output complete ===\r\n", dc);
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
	eraseFlash();																// セクター11を消去
	writeFlash(start_adress_sector11, (uint8_t *)data, sizeof(LogData_t) * dc); // dc個のデータを書き込み
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
