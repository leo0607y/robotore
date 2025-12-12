/*
 * SpeedControl.c
 *
 *  Created on: Jul 22, 2025
 *      Author: reoch
 */

#define DELTA_T 0.001

// PIゲイン調整：安定性重視（フリーズ防止）
// KP: 300→100（応答性は維持しつつオーバーシュート抑制）
// KI: 50→10（積分項の暴走を防止、PWMオーバーフロー対策）
#define KP 100.0f
#define KI 10.0f

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
