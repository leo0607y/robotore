/*
 * SpeedControl.c
 *
 *  Created on: Jul 22, 2025
 *      Author: reoch
 */

#define DELTA_T 0.001

// PIゲイン調整：応答性と安定性のバランス
// KP: 150（目標速度への到達時間を短縮）
// KI: 20（定常偏差を素早く解消、PWMオーバーフロー対策は±1599クリッピングで担保）
#define KP 150.0f
#define KI 20.0f

float SpeedControl(float target_velocity, float current_velocity, float *integral)
{
	float e = target_velocity - current_velocity;
	*integral += e * DELTA_T; // I制御

	// 積分ワインドアップ防止（より広い範囲を許容）
	if (*integral > 10.0f)
		*integral = 10.0f;
	if (*integral < -10.0f)
		*integral = -10.0f;

	float pwm = e * KP + *integral * KI;

	// PWM値のクリッピング（オーバーフロー防止）
	// MAX_COUNTER_PERIOD = 1599 の範囲内に制限
	if (pwm > 1599.0f)
		pwm = 1599.0f;
	if (pwm < -1599.0f)
		pwm = -1599.0f;

	return pwm;
}
