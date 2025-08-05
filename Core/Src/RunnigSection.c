#include "RunningSection.h"

// static float velocity_table[5000];
// static int16_t acceleration_table[5000];

#define own_ms_CP 180 // 1m/sに必要なCpunterPeriod

uint16_t velocity_table_idx;
uint16_t mode;

bool Start_Flag = false;
bool Stop_Flag = false;
uint8_t Marker_State = 0;       // 0: idle, 1: start passed, 2: goal candidate
uint32_t RightDetectedTime = 0;

static int16_t Fan;

float ref_distance;
extern int bayado;



void Fan_Ctrl(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Fan);
}

void FanMotor(int16_t suction)
{
	Fan = abs(suction);

	if(suction >= 4199) suction = 4199;
}

void S_Sensor() {
	// センサ読み取り（白いライン上でtrue）
	bool side_sensor_r = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET); // R: ライン上
	bool side_sensor_l = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET); // L: ライン上

	static bool prev_side_sensor_r = false;
	static uint32_t start_passed_time = 0;

	if (!Start_Flag) {
		// スタート判定は右センサの立ち上がりだけで判定
		bool rising_edge_r = (!prev_side_sensor_r && side_sensor_r);
		if (rising_edge_r) {
			Start_Flag = true;
			Marker_State = 1;
			start_passed_time = HAL_GetTick();  // スタート時刻を保存
		}
	} else if (Start_Flag && !Stop_Flag) {
		bool rising_edge_r = (!prev_side_sensor_r && side_sensor_r);
		uint32_t current_time = HAL_GetTick();

		if (Marker_State == 1 && rising_edge_r
				&& (current_time - start_passed_time) > 1000) {
			RightDetectedTime = current_time;
			Marker_State = 2;
		}

		if (Marker_State == 2 && side_sensor_l) {
			uint32_t dt = current_time - RightDetectedTime;
			if (dt <= 100) {
				Marker_State = 1;  // 交差ラインなのでキャンセル
			}
		}

		if (Marker_State == 2) {
			uint32_t dt = current_time - RightDetectedTime;
			if (dt > 100) {
				Stop_Flag = true;
				bayado = 7;
				setMotor(0, 0);
				Marker_State = 0;
				Start_Flag = false;
				Stop_Flag = false;

			}
		}
	}

	prev_side_sensor_r = side_sensor_r;

}

