/*
 * IMU20649.c
 *
 *  Created on: Aug 22, 2025
 *      Author: reoch
 */

#include "IMU20649.h"

volatile int16_t xa, ya, za; // 加速度(16bitデータ)
volatile int16_t xg, yg, zg;	// 角加速度(16bitデータ)

float zg_offset = 0.0f;

uint8_t read_byte(uint8_t reg) {
	uint8_t ret, val;

	ret = reg | 0x80;
	CS_RESET;
	HAL_SPI_Transmit(&hspi3, &ret, 1, 100);
	HAL_SPI_Receive(&hspi3, &val, 1, 100);
	CS_SET;

	return val;
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

void write_byte(uint8_t reg, uint8_t val) {
	uint8_t ret;

	ret = reg & 0x7F;
	CS_RESET;
	HAL_SPI_Transmit(&hspi3, &ret, 1, 100);
	HAL_SPI_Transmit(&hspi3, &val, 1, 100);
	CS_SET;
}

uint8_t IMU_Init() {
	uint8_t who_am_i, ret;

	who_am_i = read_byte(0x00);	//IMU動作確認　0xE0が送られてくればおｋ
	//if ( who_am_i == 0xE1 ) {
	if (who_am_i == 0xE1) { //ICM20649
		ret = 1;
		write_byte(0x06, 0x01);	//PWR_MGMT_1	スリープﾓｰﾄﾞ解除
		write_byte(0x03, 0x10);	//USER_CTRL	諸々機能無効　SPIonly
		write_byte(0x7F, 0x20);	//USER_BANK2

		//write_byte(0x01,0x06);	//	レンジ±2000dps DLPF disable
		//write_byte(0x01,0x07);	//range±2000dps DLPF enable DLPFCFG = 0
		//write_byte(0x01,0x0F);	//range±2000dps DLPF enable DLPFCFG = 1
//		write_byte(0x01,0x17);	//range±2000dps DLPF enable DLPFCFG = 2 ICM20648
		write_byte(0x01, 0x18); //ICM20649_4000dps

		//2:1 GYRO_FS_SEL[1:0] 00:±250	01:±500 10:±1000 11:±2000
		write_byte(0x14, 0x06);	//	レンジ±16g
		//2:1 ACCEL_FS_SEL[1:0] 00:±2	01:±4 10:±8 11:±16
		write_byte(0x7F, 0x00);	//USER_BANK0
	}
	return ret;
}

void read_zg_data() {
	zg = ((int16_t) read_byte(0x37) << 8) | ((int16_t) read_byte(0x38));
}

void read_gyro_data() {
	xg = ((uint16_t) read_byte(0x33) << 8) | ((uint16_t) read_byte(0x34));
	yg = ((uint16_t) read_byte(0x35) << 8) | ((uint16_t) read_byte(0x36));
	zg = ((uint16_t) read_byte(0x37) << 8) | ((uint16_t) read_byte(0x38));
}

void read_xa_data() {
	xa = ((int16_t) read_byte(0x2D) << 8) | ((int16_t) read_byte(0x2E));
}

void read_accel_data() {
	xa = ((uint16_t) read_byte(0x2D) << 8) | ((uint16_t) read_byte(0x2E));
	ya = ((uint16_t) read_byte(0x2F) << 8) | ((uint16_t) read_byte(0x30));
	za = ((uint16_t) read_byte(0x31) << 8) | ((uint16_t) read_byte(0x32));
}

