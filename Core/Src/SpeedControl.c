/*
 * SpeedControl.c
 *
 *  Created on: Jul 22, 2025
 *      Author: reoch
 */

#define DELTA_T 0.001

// PIゲイン: 超高速応答設定（0.2秒以内の目標到達を目指す）
// KP: 500（超高速応答）
// KI: 100（非常に強力な積分、定常偏差を素早く解消）
// 積分上限: ±30.0（最大PWM = 500*0.3 + 100*30 = 3150 → クリッピングで1599）
// 安全対策: PWM±1599クリッピング + Watchdog(1.5秒)で完全保護
#define KP 500.0f
#define KI 100.0f

float SpeedControl(float target_velocity, float current_velocity, float *integral)
{
	float e = target_velocity - current_velocity;
	*integral += e * DELTA_T; // I制御

	// 積分ワインドアップ防止（±30で固定）
	if (*integral > 30.0f)
		*integral = 30.0f;
	if (*integral < -30.0f)
		*integral = -30.0f;

	float pwm = e * KP + *integral * KI;

	// PWM値のクリッピング（オーバーフロー防止）
	// MAX_COUNTER_PERIOD = 1599 の範囲内に制限
	if (pwm > 1599.0f)
		pwm = 1599.0f;
	if (pwm < -1599.0f)
		pwm = -1599.0f;

	return pwm;
}
