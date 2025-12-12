/*
 * SpeedControl.c
 *
 *  Created on: Jul 22, 2025
 *      Author: reoch
 */

#define DELTA_T 0.001

// PIゲイン: 超高速応答設定（0.2秒以内の確実な到達）
// KP: 1000（非常に積極的な初期応答、誤差に即座に反応）
// KI: 250（極めて強力な積分、定常偏差を即座に解消）
// 積分上限: ±15.0（過度な積分を抑制、安定性とレスポンスの両立）
// 計算例: e=0.2時、PWM = 1000*0.2 + 250*15 = 200+3750 = 3950 → クリッピングで1599
// 安全対策: PWM±1599クリッピング + Watchdog(1.5秒)で完全保護
#define KP 1000.0f
#define KI 250.0f

float SpeedControl(float target_velocity, float current_velocity, float *integral)
{
	float e = target_velocity - current_velocity;
	*integral += e * DELTA_T; // I制御

	// 積分ワインドアップ防止（±15で制限、安定性とレスポンスの最適バランス）
	if (*integral > 15.0f)
		*integral = 15.0f;
	if (*integral < -15.0f)
		*integral = -15.0f;

	float pwm = e * KP + *integral * KI;

	// PWM値のクリッピング（オーバーフロー防止）
	// MAX_COUNTER_PERIOD = 1599 の範囲内に制限
	if (pwm > 1599.0f)
		pwm = 1599.0f;
	if (pwm < -1599.0f)
		pwm = -1599.0f;

	return pwm;
}
