/*
 * SpeedControl.c
 *
 *  Created on: Jul 22, 2025
 *      Author: reoch
 */

#define DELTA_T 0.001

// PIゲイン: 元の高応答設定に戻す（PWMクリッピングで安全確保）
// KP: 300（最速応答）
// KI: 50（強力な定常偏差解消）
// 積分上限: ±30.0（十分なPWM出力を確保）
// 安全対策: PWM±1599クリッピング + Watchdog(1.5秒)
#define KP 300.0f
#define KI 50.0f

float SpeedControl(float target_velocity, float current_velocity, float *integral)
{
	float e = target_velocity - current_velocity;
	*integral += e * DELTA_T; // I制御

	// 積分ワインドアップ防止（積分上限を拡大: ±10→±30）
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
