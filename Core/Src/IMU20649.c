/*
 * IMU20649.c
 *
 *  Created on: Aug 22, 2025
 *      Author: reoch
 */

#include "IMU20649.h"

static uint8_t imu_current_bank = 0x00;

static void IMU_SelectBank(uint8_t bank)
{
	if (imu_current_bank == bank)
	{
		return;
	}

	uint8_t reg = 0x7F;
	CS_RESET;
	HAL_SPI_Transmit(&hspi3, &reg, 1, 100);
	HAL_SPI_Transmit(&hspi3, &bank, 1, 100);
	CS_SET;
	imu_current_bank = bank;
}

static uint8_t imu_read_byte_bank0(uint8_t reg)
{
	uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
	uint8_t rx[2] = { 0 };

	IMU_SelectBank(0x00);
	CS_RESET;
	HAL_SPI_TransmitReceive(&hspi3, tx, rx, 2, 100);
	CS_SET;

	return rx[1];
}

static void imu_read_bytes_bank0(uint8_t start_reg, uint8_t *dst, uint8_t len)
{
	uint8_t tx[16] = {0};
	uint8_t rx[16] = {0};

	if (len == 0 || len > 15)
	{
		return;
	}

	tx[0] = (uint8_t)(start_reg | 0x80);
	for (uint8_t i = 1; i <= len; i++)
	{
		tx[i] = 0x00;
	}

	IMU_SelectBank(0x00);
	CS_RESET;
	HAL_SPI_TransmitReceive(&hspi3, tx, rx, (uint16_t)(len + 1), 100);
	CS_SET;

	for (uint8_t i = 0; i < len; i++)
	{
		dst[i] = rx[i + 1];
	}
}

volatile int16_t xa, ya, za; // 加速度(16bitデータ)
volatile int16_t xg, yg, zg; // 角加速度(16bitデータ)

float zg_offset = 0.0f;

uint8_t read_byte(uint8_t reg)
{
	return imu_read_byte_bank0(reg);
}

void IMU_CalibrateGyro(void)
{
	const uint16_t CALIB_SAMPLES = 1000; // 1000回サンプリング = 1秒間 (1ms間隔でサンプリングを想定)
	int32_t zg_sum = 0;

	// 最初のIMU読み取りが完了するまで待機（IMU_InitでSPI通信が確立されているはず）
	read_gyro_data();

	printf("Gyro Calibration: Start 1 second measurement...\r\n");

	// 1000回サンプリング
	for (uint16_t i = 0; i < CALIB_SAMPLES; i++)
	{
		read_gyro_data(); // IMUから新しいデータを読み込み、グローバル変数 zg (Raw Value) を更新
		zg_sum += zg;
		HAL_Delay(1); // 1ms待機
	}

	// 平均値を計算し、オフセットとして保存
	zg_offset = (float)zg_sum / CALIB_SAMPLES;

	printf("Gyro Calibration: Offset calculated (Raw Value): %.3f\r\n", zg_offset);
}

void write_byte(uint8_t reg, uint8_t val)
{
	uint8_t ret;

	if (reg == 0x7F)
	{
		imu_current_bank = val;
	}

	ret = reg & 0x7F;
	CS_RESET;
	HAL_SPI_Transmit(&hspi3, &ret, 1, 100);
	HAL_SPI_Transmit(&hspi3, &val, 1, 100);
	CS_SET;
}

uint8_t IMU_Init()
{
	static uint8_t initialized = 0;
	uint8_t who_am_i, ret;

	// 既に初期化済みなら再初期化しない
	if (initialized)
	{
		printf("IMU Init: Already initialized, skipping...\r\n");
		return 1;
	}

	imu_current_bank = 0xFF;
	IMU_SelectBank(0x00);

	who_am_i = read_byte(0x00); // IMU動作確認　0xE0が送られてくればおｋ
	printf("IMU WHOAMI: 0x%02X (Expected: 0xE1)\r\n", who_am_i);

	// if ( who_am_i == 0xE1 ) {
	if (who_am_i == 0xE1)
	{ // ICM20649
		ret = 1;
		initialized = 1; // 初期化完了フラグをセット
		printf("IMU Init: SUCCESS\r\n");
		write_byte(0x06, 0x01); // PWR_MGMT_1	スリープﾓｰﾄﾞ解除
		write_byte(0x03, 0x10); // USER_CTRL	諸々機能無効　SPIonly
		write_byte(0x7F, 0x20); // USER_BANK2

		// write_byte(0x01,0x06);	//	レンジ±2000dps DLPF disable
		// write_byte(0x01,0x07);	//range±2000dps DLPF enable DLPFCFG = 0
		// write_byte(0x01,0x0F);	//range±2000dps DLPF enable DLPFCFG = 1
		//		write_byte(0x01,0x17);	//range±2000dps DLPF enable DLPFCFG = 2 ICM20648
		write_byte(0x01, 0x06); // range +/-2000 dps, DLPF disable

		// 2:1 GYRO_FS_SEL[1:0] 00:±250	01:±500 10:±1000 11:±2000
		write_byte(0x14, 0x06); //	レンジ±16g
		// 2:1 ACCEL_FS_SEL[1:0] 00:±2	01:±4 10:±8 11:±16
		write_byte(0x7F, 0x00); // USER_BANK0
	}
	else
	{
		ret = 0;
		printf("IMU Init: FAILED - WHOAMI mismatch\r\n");
	}
	return ret;
}

void read_zg_data()
{
	uint8_t raw[2] = {0};
	imu_read_bytes_bank0(0x37, raw, 2);
	zg = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
}

void read_gyro_data()
{
	uint8_t raw[6] = {0};
	imu_read_bytes_bank0(0x33, raw, 6);
	xg = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
	yg = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
	zg = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
}

void read_xa_data()
{
	uint8_t raw[2] = {0};
	imu_read_bytes_bank0(0x2D, raw, 2);
	xa = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
}

void read_accel_data()
{
	uint8_t raw[6] = {0};
	imu_read_bytes_bank0(0x2D, raw, 6);
	xa = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
	ya = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
	za = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
}
