#include "TrackingPart.h"
#include "Encoder.h"
#include <stdio.h>

#define DELTA_T 0.001

static int8_t trace_flag;
static uint8_t i_clear_flag;
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
float diff = 0;

static float integral_left = 0.0;
static float integral_right = 0.0;

void getCurrentVelocity(float *current_speed_left, float *current_speed_right) {
	int16_t enc_l = 0, enc_r = 0;
	getEncoderCnt(&enc_l, &enc_r);

	*current_speed_left = DISTANCE_PER_CNT * (float) enc_l / DELTA_T;
	*current_speed_right = DISTANCE_PER_CNT * (float) enc_r / DELTA_T;
}

void ControlLineTracking(void) {
	float p, d;
	static float i;
	//	float kp = 0.0072;
	//	float kd = 0.00013;
	//	float kp = 0.0039;//2.4m/s
	//	float kd = 0.00003;//2.4m/s
	float kp = 0.008;	// 2.6m/s
	float kd = 0.00003; // 2.6m/s

	//    float diff = 0;
	if (trace_flag == 1) {
		if (i_clear_flag == 1) {
			i = 0;
			i_clear_flag = 0;
		}
		//		diff = ((sensor[0] * 2 + sensor[1] * 1.8 + sensor[2] * 1.6
		//				+ sensor[3] * 1.4 + sensor[4] * 1.2 + sensor[5]) / 5)
		//				- ((sensor[6] + sensor[7] * 1.2 + sensor[8] * 1.4
		//						+ sensor[9] * 1.6 + sensor[10] * 1.8 + sensor[11] * 2)
		//						/ 5);
		diff = ((sensor[0] * 1.0 + sensor[1] * 1.0 + sensor[2] * 1.0
				+ sensor[3] * 1.0 + sensor[4] * 1.0 + sensor[5]) / 5)
				- ((sensor[6] + sensor[7] * 1.0 + sensor[8] * 1.0
						+ sensor[9] * 1.0 + sensor[10] * 1.0 + sensor[11] * 1.0)
						/ 5);
		p = kp * diff;
		d = kd * (diff - pre_diff) / DELTA_T;
		tracking_term = p + d + i;

		pre_diff = diff;
	}
}

void TraceFlip(void) {
	if (trace_flag == 1) {
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
		// 速度制御
		float speed_pi_output_left = SpeedControl(TargetSpeed - tracking_term,
				current_speed_left, &integral_left);
		float speed_pi_output_right = SpeedControl(TargetSpeed + tracking_term,
				current_speed_right, &integral_right);

		//		float speed_pi_output_left = SpeedControl(0.0, current_speed_left, &integral_left);
		//				float speed_pi_output_right = SpeedControl(0.0, current_speed_right, &integral_right);

		//        mon_velo_term = velo_ctrl_term;

		setMotor(speed_pi_output_left, speed_pi_output_right);
	} else {
		setMotor(0, 0);
	}
}

void startTracking(void) {
	trace_flag = 1;
	i_clear_flag = 1;
}

void stopTracking(void) {
	trace_flag = 0;
	tracking_term = 0;
	setMotor(0, 0);
}

void CourseOut(void) {
	volatile static uint16_t all_sensor;
//	uint16_t all_sensor;
	static uint16_t unable_cnt;
	all_sensor = (sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4]
			+ sensor[5] + sensor[6] + sensor[7] + sensor[8] + sensor[9]
			+ sensor[10] + sensor[11]) / 12;
	if (all_sensor > 900) {
		unable_cnt++;
	} else {
		unable_cnt = 0;
	}

	if (unable_cnt >= 40) {
		Unable_to_run_flag = true;
		Marker_State = 0;
		Start_Flag = false;
		Stop_Flag = false;
		bayado = 6; // mainにあるlionと同じ変数に戻す
		setMotor(0, 0);
	} else {
		Unable_to_run_flag = false;
	}
}


void debugmotor(float mon_deb_l, float mon_deb_r) {
	mo_l_Deb = mon_deb_l;
	mo_r_Deb = mon_deb_r;
}

bool getUnableToRunFlag(void) {
	return Unable_to_run_flag;
}

void setTarget(float speed) {
	Target = speed; // baseSpeedを設定する関数
}

float getTarget(void) {
	return Target; // baseSpeedを取得する関数
}
