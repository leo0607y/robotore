/*
 * SpeedControl.c
 *
 *  Created on: Jul 22, 2025
 *      Author: reoch
 */

#define DELTA_T 0.001

// PIゲイン調整：高速応答重視（PWMクリッピングで安全性確保）
// KP: 250（素早い目標速度到達）
// KI: 40（定常偏差を短時間で解消）
// 安全対策：PWM±1599クリッピング + Watchdog(1.5秒)で保護
#define KP 250.0f
#define KI 40.0f

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
