#include "TrackingPart.h"
#include "Encoder.h"
#include <stdio.h>
#include <math.h>

#define DELTA_T 0.001
#define STRAIGHT_DIFF_THRESHOLD 100.0f
#define TRACKING_TERM_MAX 1.2f
#define CURVE_TARGET_GAIN 0.25f
#define CENTER_SPEED_K 0.6f

static float clampf(float value, float min_value, float max_value)
{
	if (value < min_value)
		return min_value;
	if (value > max_value)
		return max_value;
	return value;
}

int8_t trace_flag;
static uint8_t i_clear_flag;
bool is_on_tracking_curve = false;
float tracking_term; // static修飾子を削除
static bool Unable_to_run_flag;
static float mo_l_Deb;
static float mo_r_Deb;
static float pre_diff;
float mon_velo_term;
static float Target = 0; // baseSpeedを初期化

extern bool Start_Flag;
extern bool Stop_Flag;
extern uint8_t Marker_State;

// mainのlion変数をextern宣言
extern int bayado;
extern int lion;
float diff = 0;

static float integral_left = 0.0;
static float integral_right = 0.0;
static float debug_pwm_left = 0.0f;
static float debug_pwm_right = 0.0f;

volatile uint16_t all_sensor = 0;

void getCurrentVelocity(float *current_speed_left, float *current_speed_right)
{
	int16_t enc_l = 0, enc_r = 0;
	getEncoderCnt(&enc_l, &enc_r);

	*current_speed_left = DISTANCE_PER_CNT * (float)enc_l / DELTA_T;
	*current_speed_right = DISTANCE_PER_CNT * (float)enc_r / DELTA_T;
}

void ControlLineTracking(void)
{
	float p, d;
	static float i;
	//	float kp = 0.0072;
	//	float kd = 0.00013;
	float kp = 0.005;	 // 2.4m/s
	float kd = 0.000002; // 2.4m/s
	//	float kp = 0.018;	// 2.6m/s
	//	float kd = 0.00008; // 2.6m/s

	//    float diff = 0;
	if (trace_flag == 1)
	{
		if (i_clear_flag == 1)
		{
			i = 0;
			i_clear_flag = 0;
		}
		//		diff = ((sensor[0] * 2 + sensor[1] * 1.8 + sensor[2] * 1.6
		//				+ sensor[3] * 1.4 + sensor[4] * 1.2 + sensor[5]) / 5)
		//				- ((sensor[6] + sensor[7] * 1.2 + sensor[8] * 1.4
		//						+ sensor[9] * 1.6 + sensor[10] * 1.8 + sensor[11] * 2)
		//						/ 5);
		diff = ((sensor[0] * 1.0 + sensor[1] * 1.0 + sensor[2] * 1.0 + sensor[3] * 1.0 + sensor[4] * 1.0 + sensor[5]) / 5) - ((sensor[6] + sensor[7] * 1.0 + sensor[8] * 1.0 + sensor[9] * 1.0 + sensor[10] * 1.0 + sensor[11] * 1.0) / 5);
		float abs_diff = fabsf(diff); // floatの絶対値を取得

		// |diff| >= 100.0f の場合を「カーブ走行中（ゴール判定を無視）」と見なす
		if (abs_diff >= STRAIGHT_DIFF_THRESHOLD)
		{
			is_on_tracking_curve = true; // diffが大きいためカーブ走行中と判定
		}
		else
		{
			is_on_tracking_curve = false; // diffが小さいため直線走行中と判定
		}
		p = kp * diff;
		d = kd * (diff - pre_diff) / DELTA_T;
		tracking_term = p + d + i;

		// カーブ時は左右速度差を増幅して大回りに（R10対策）
		if (abs_diff >= STRAIGHT_DIFF_THRESHOLD)
		{
			tracking_term *= 1.3; // カーブ時は30%増幅
		}

		pre_diff = diff;
	}
}

void TraceFlip(void)
{
	if (trace_flag == 1)
	{
		// float velo_ctrl_term = getVelocityControlTerm();
		//        float limit = MAX_COUNTER_PERIOD * 0.8;

		//        if (velo_ctrl_term >= limit)
		//            velo_ctrl_term = limit;
		//        else if (velo_ctrl_term <= -limit)
		//            velo_ctrl_term = -limit;

		//        float exceeded = 0;
		//        if (velo_ctrl_term + tracking_term >= MAX_COUNTER_PERIOD)
		//        {
		//            exceeded = (velo_ctrl_term + tracking_term) - MAX_COUNTER_PERIOD;
		//        }
		//        else if (velo_ctrl_term - tracking_term <= -MAX_COUNTER_PERIOD)
		//        {
		//            exceeded = -MAX_COUNTER_PERIOD - (velo_ctrl_term - tracking_term);
		//        }
		//        velo_ctrl_term -= exceeded;
		//        tracking_term += exceeded;

		// 車両移動速度取得
		float current_speed_left = 0.0;
		float current_speed_right = 0.0;
		getCurrentVelocity(&current_speed_left, &current_speed_right);
		float TargetSpeed = getTarget();
		float center_speed = (current_speed_left + current_speed_right) * 0.5f;

		// 理想的な中心速度制御：
		// 各モーターの目標速度を設定し、平均がTargetになるようにする
		// 左目標 = Target - tracking_term
		// 右目標 = Target + tracking_term
		// → (左目標 + 右目標) / 2 = Target が保証される

		float tracking_term_limited = clampf(tracking_term, -TRACKING_TERM_MAX, TRACKING_TERM_MAX);
		float curve_ratio = fabsf(tracking_term_limited) / TRACKING_TERM_MAX;
		float TargetSpeed_adj = TargetSpeed * (1.0f + CURVE_TARGET_GAIN * curve_ratio);
		float center_bias = CENTER_SPEED_K * (TargetSpeed_adj - center_speed);

		float target_speed_left = TargetSpeed_adj - tracking_term_limited + center_bias;
		float target_speed_right = TargetSpeed_adj + tracking_term_limited + center_bias;

		// 各モーターを目標速度に追従させるPI制御
		float speed_pi_output_left = SpeedControl(target_speed_left,
												  current_speed_left, &integral_left);
		float speed_pi_output_right = SpeedControl(target_speed_right,
												   current_speed_right, &integral_right);

		debug_pwm_left = speed_pi_output_left;
		debug_pwm_right = speed_pi_output_right;

		setMotor(speed_pi_output_left, speed_pi_output_right);
	}
	else
	{
		debug_pwm_left = 0.0f;
		debug_pwm_right = 0.0f;
		setMotor(0, 0);
	}
}

void ResetAllTrackingVariables(void)
{
	// ★重要★ 2回目以降の走行のため、全ての状態変数を初期化
	integral_left = 0.0f;
	integral_right = 0.0f;
	pre_diff = 0.0f;
	tracking_term = 0.0f;
	diff = 0.0f;
	Unable_to_run_flag = false;
	is_on_tracking_curve = false;
	trace_flag = 0;
	i_clear_flag = 0;
}

void startTracking(void)
{
	trace_flag = 1;
	i_clear_flag = 1;
}

void stopTracking(void)
{
	trace_flag = 0;
	tracking_term = 0;
	setMotor(0, 0);
}

void CourseOut(void)
{
	if (trace_flag == 1)
	{
		//	uint16_t all_sensor;
		static uint16_t unable_cnt;
		static uint16_t unable_cnt_all_high;

		// 条件1: 平均値が900超
		all_sensor = (sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5] + sensor[6] + sensor[7] + sensor[8] + sensor[9] + sensor[10] + sensor[11]) / 12;
		if (all_sensor > 920)
		{
			unable_cnt++;
		}
		else
		{
			unable_cnt = 0;
		}

		// 条件2: すべてのセンサが500超（1個でも500以下ならリセット）
		bool all_sensors_high = true;
		for (int i = 0; i < 12; i++)
		{
			if (sensor[i] <= 500)
			{
				all_sensors_high = false;
				break;
			}
		}

		if (all_sensors_high)
		{
			unable_cnt_all_high++;
		}
		else
		{
			unable_cnt_all_high = 0;
		}

		// どちらかの条件が50ms継続したらコースアウト
		if (unable_cnt >= 80 || unable_cnt_all_high >= 70)
		{
			Unable_to_run_flag = true;
			Marker_State = 0;
			Start_Flag = false;
			Stop_Flag = false;
			bayado = 6; // mainにあるlionと同じ変数に戻す

			// どちらの条件で停止したかをlionで区別
			if (unable_cnt >= 80)
			{
				lion = 1; // 条件1：平均値900超で停止
			}
			else
			{
				lion = 0; // 条件2：全センサ500超で停止
			}

			setMotor(0, 0);
			trace_flag = 0;
		}
		else
		{
			Unable_to_run_flag = false;
		}
	}
}

void debugmotor(float mon_deb_l, float mon_deb_r)
{
	mo_l_Deb = mon_deb_l;
	mo_r_Deb = mon_deb_r;
}

bool getUnableToRunFlag(void)
{
	return Unable_to_run_flag;
}

void setTarget(float speed)
{
	Target = speed; // baseSpeedを設定する関数
}

float getTarget(void)
{
	return Target; // baseSpeedを取得する関数
}

float getDebugPwmAbsMax(void)
{
	float left_abs = fabsf(debug_pwm_left);
	float right_abs = fabsf(debug_pwm_right);
	return (left_abs > right_abs) ? left_abs : right_abs;
}
