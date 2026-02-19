#include "RunningSection.h"
#include "TrackingPart.h"
#include "log.h"
#include "stdio.h"

#define GOAL_DIAG_PRINT 0
#define CROSS_ALL_SENSOR_THRESHOLD 500
#define CROSS_GUARD_DISTANCE_MM 80.0f
#define CROSS_CENTER_SENSOR_START 3
#define CROSS_CENTER_SENSOR_END 8

// static float velocity_table[5000];
// static int16_t acceleration_table[5000];

#define own_ms_CP 180 // 1m/sに必要なCpunterPeriod

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
static bool prev_all_sensors_high_global = false;
static float cross_guard_until_distance_mm_global = 0.0f;

void Reset_S_Sensor_State(void)
{
	prev_side_sensor_r_global = false;
	start_passed_time_global = 0;
	prev_all_sensors_high_global = false;
	cross_guard_until_distance_mm_global = 0.0f;
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
	uint32_t current_time = HAL_GetTick();
	float current_distance_mm = getTotalDistance();

	bool all_sensors_high = true;
	for (int i = CROSS_CENTER_SENSOR_START; i <= CROSS_CENTER_SENSOR_END; i++)
	{
		if (sensor[i] <= CROSS_ALL_SENSOR_THRESHOLD)
		{
			all_sensors_high = false;
			break;
		}
	}

	bool rising_edge_all_high = (!prev_all_sensors_high_global && all_sensors_high);
	if (Start_Flag && !Stop_Flag && rising_edge_all_high)
	{
		cross_guard_until_distance_mm_global = current_distance_mm + CROSS_GUARD_DISTANCE_MM;
#if GOAL_DIAG_PRINT
		printf("[MARKER] CROSS_PREP allSensorHigh dist=%.1fmm guard_until=%.1fmm\r\n",
				current_distance_mm, cross_guard_until_distance_mm_global);
#endif
	}

	if (!Start_Flag)
	{
		// スタート判定は右センサの立ち上がりだけで判定
		bool rising_edge_r = (!prev_side_sensor_r_global && side_sensor_r);
		if (rising_edge_r)
		{
			Start_Flag = true;
			Marker_State = 1;
			start_passed_time_global = current_time; // スタート時刻を保存
#if GOAL_DIAG_PRINT
			printf("[MARKER] START riseR t=%lums dist=%.1fmm\r\n", current_time, current_distance_mm);
#endif
		}
	}
	else if (Start_Flag && !Stop_Flag && is_on_tracking_curve)
	{
		bool rising_edge_r = (!prev_side_sensor_r_global && side_sensor_r);
		if (rising_edge_r)
		{
#if GOAL_DIAG_PRINT
			printf("[MARKER] IGNORE riseR(on_curve) t=%lums dist=%.1fmm\r\n", current_time, current_distance_mm);
#endif
		}
	}
	else if (Start_Flag && !Stop_Flag && !is_on_tracking_curve)
	{
		bool rising_edge_r = (!prev_side_sensor_r_global && side_sensor_r);
		bool in_cross_guard = (current_distance_mm <= cross_guard_until_distance_mm_global);

		if (Marker_State == 1 && rising_edge_r && (current_time - start_passed_time_global) > 1000)
		{
			if (in_cross_guard)
			{
#if GOAL_DIAG_PRINT
				printf("[MARKER] GOAL_IGNORE riseR_in_cross_guard t=%lums dist=%.1fmm guard_until=%.1fmm\r\n",
						current_time, current_distance_mm, cross_guard_until_distance_mm_global);
#endif
			}
			else
			{
				RightDetectedTime = current_time;
				Marker_State = 2;
#if GOAL_DIAG_PRINT
				printf("[MARKER] GOAL_CAND riseR t=%lums dist=%.1fmm dt_start=%lums\r\n",
						current_time, current_distance_mm, current_time - start_passed_time_global);
#endif
			}
		}

		if (Marker_State == 2 && side_sensor_l)
		{
			uint32_t dt = current_time - RightDetectedTime;
			if (dt <= 150)
			{
				Marker_State = 1; // 交差ラインなのでキャンセル
#if GOAL_DIAG_PRINT
				printf("[MARKER] GOAL_CANCEL left_within_150ms t=%lums dist=%.1fmm dt=%lums\r\n",
						current_time, current_distance_mm, dt);
#endif
			}
		}

		if (Marker_State == 2)
		{
			if (in_cross_guard)
			{
				Marker_State = 1;
#if GOAL_DIAG_PRINT
				printf("[MARKER] GOAL_CANCEL cross_guard_active t=%lums dist=%.1fmm guard_until=%.1fmm\r\n",
						current_time, current_distance_mm, cross_guard_until_distance_mm_global);
#endif
			}

			uint32_t dt = current_time - RightDetectedTime;
			if (Marker_State == 2 && dt > 150)
			{
				// ★★★ 緊急停止：最優先でモーターを停止 ★★★

#if GOAL_DIAG_PRINT
				printf("[MARKER] GOAL_COMMIT stop_cmd t=%lums dist=%.1fmm dt_from_right=%lums\r\n",
						current_time, current_distance_mm, dt);
#endif

				bayado = 6;
				trace_flag = 0;
				setMotor(0, 0);
				LED(LED_OFF);
				// トレースフラグも即座にOFF

				Stop_Flag = true;
				lion = 6;
				Marker_State = 0;
				Start_Flag = false;
				Stop_Flag = false;

				// Flash書き込みは停止後に実行
				// printf("Goal detected! Motor stopped. Saving logs...\r\n");
				WriteData();
			}
		}
	}

	prev_side_sensor_r_global = side_sensor_r;
	prev_all_sensors_high_global = all_sensors_high;
}
