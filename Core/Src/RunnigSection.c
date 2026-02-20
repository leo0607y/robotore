#include "RunningSection.h"
#include "TrackingPart.h"
#include "log.h"
#include "stdio.h"

// static float velocity_table[5000];
// static int16_t acceleration_table[5000];

#define own_ms_CP 180 // 1m/sに必要なCpunterPeriod
#define CROSS_CENTER_SENSOR_START 2
#define CROSS_CENTER_SENSOR_END 9
#define CROSS_CENTER_SENSOR_THRESHOLD 500
#define CROSS_GUARD_TIME_MS 300U
#define SHARP_CURVE_DIFF_THRESHOLD 300.0f

uint16_t velocity_table_idx;
uint16_t mode;

bool Start_Flag = false;
bool Stop_Flag = false;
uint8_t Marker_State = 0; // 0: idle, 1: start passed, 2: goal candidate
uint32_t RightDetectedTime = 0;

static int16_t Fan;

float ref_distance;
extern int lion, bayado;

// S_Sensorの静的変数（グローバルスコープに移動してリセット可能にする）
static bool prev_side_sensor_r_global = false;
static uint32_t start_passed_time_global = 0;
static uint32_t cross_ignore_until_ms = 0;

void Reset_S_Sensor_State(void)
{
	prev_side_sensor_r_global = false;
	start_passed_time_global = 0;
	cross_ignore_until_ms = 0;
}

void Fan_Ctrl(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Fan);
}

void FanMotor(int16_t suction)
{
	Fan = abs(suction);

	if (suction >= 4199)
		suction = 4199;
}

void S_Sensor()
{
	// センサ読み取り（白いライン上でtrue）
	bool side_sensor_r = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET); // R: ライン上
	bool side_sensor_l = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET); // L: ライン上

	if (!Start_Flag)
	{
		// スタート判定は右センサの立ち上がりだけで判定
		bool rising_edge_r = (!prev_side_sensor_r_global && side_sensor_r);
		if (rising_edge_r)
		{
			Start_Flag = true;
			Marker_State = 1;
			start_passed_time_global = HAL_GetTick(); // スタート時刻を保存
		}
	}
	else if (Start_Flag && !Stop_Flag && !is_on_tracking_curve)
	{
		bool rising_edge_r = (!prev_side_sensor_r_global && side_sensor_r);
		uint32_t current_time = HAL_GetTick();

		// 急カーブ中（diff大）はゴール判定を無視
		if (fabsf(diff) >= SHARP_CURVE_DIFF_THRESHOLD)
		{
			if (Marker_State == 2)
			{
				Marker_State = 1;
			}
			prev_side_sensor_r_global = side_sensor_r;
			return;
		}

		// 中心8センサが同時にライン上（低値）を検出したらクロスと判定し、一定時間ゴール判定を無視
		bool center8_all_on_line = true;
		for (int i = CROSS_CENTER_SENSOR_START; i <= CROSS_CENTER_SENSOR_END; i++)
		{
			if (sensor[i] > CROSS_CENTER_SENSOR_THRESHOLD)
			{
				center8_all_on_line = false;
				break;
			}
		}

		if (center8_all_on_line)
		{
			cross_ignore_until_ms = current_time + CROSS_GUARD_TIME_MS;
			if (Marker_State == 2)
			{
				Marker_State = 1; // ゴール候補中でもクロス優先でキャンセル
			}
		}

		bool cross_guard_active = (current_time < cross_ignore_until_ms);
		if (cross_guard_active)
		{
			prev_side_sensor_r_global = side_sensor_r;
			return;
		}

		if (Marker_State == 1 && rising_edge_r && (current_time - start_passed_time_global) > 1000)
		{
			RightDetectedTime = current_time;
			Marker_State = 2;
		}

		if (Marker_State == 2 && side_sensor_l)
		{
			uint32_t dt = current_time - RightDetectedTime;
			if (dt <= 100)
			{
				Marker_State = 1; // 交差ラインなのでキャンセル
			}
		}

		if (Marker_State == 2)
		{
			uint32_t dt = current_time - RightDetectedTime;
			if (dt > 100)
			{
				// ★★★ 緊急停止：最優先でモーターを停止 ★★★

				bayado = 6;
				trace_flag = 0;
				setMotor(0, 0);
				// トレースフラグも即座にOFF

				Stop_Flag = true;
				lion = 6;
				Marker_State = 0;
				Start_Flag = false;
				Stop_Flag = false;

				// Flash書き込みは停止後に実行
				printf("Goal detected! Motor stopped. Saving logs...\r\n");
				WriteData();
			}
		}
	}

	prev_side_sensor_r_global = side_sensor_r;
}
