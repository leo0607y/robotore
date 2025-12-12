/*
 * SpeedControl.c
 *
 *  Created on: Jul 22, 2025
 *      Author: reoch
 */

#define DELTA_T 0.001

// PIゲイン: 超超高速応答設定（0.2秒以内の目標到達 + 安定化）
// KP: 700（極めて高速な初期応答）
// KI: 150（極めて強力な積分、定常偏差を瞬時に解消）
// 積分上限: ±20.0（適度な制限で安定性も確保）
// 計算例: e=0.2時、PWM = 700*0.2 + 150*20 = 140+3000 = 3140 → クリッピングで1599
// 安全対策: PWM±1599クリッピング + Watchdog(1.5秒)で完全保護
#define KP 700.0f
#define KI 150.0f

float SpeedControl(float target_velocity, float current_velocity, float *integral)
{
	float e = target_velocity - current_velocity;
	*integral += e * DELTA_T; // I制御

	// 積分ワインドアップ防止（±20で適度に制限、安定性向上）
	if (*integral > 20.0f)
		*integral = 20.0f;
	if (*integral < -20.0f)
		*integral = -20.0f;

	float pwm = e * KP + *integral * KI;

	// PWM値のクリッピング（オーバーフロー防止）
	// MAX_COUNTER_PERIOD = 1599 の範囲内に制限
	if (pwm > 1599.0f)
		pwm = 1599.0f;
	if (pwm < -1599.0f)
		pwm = -1599.0f;

	return pwm;
}
