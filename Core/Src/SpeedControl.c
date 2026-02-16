/*
 * SpeedControl.c
 *
 *  Created on: Jul 22, 2025
 *      Author: reoch
 */

#define DELTA_T 0.001

// PIゲイン: 発振を抑えるためPを下げ、Iは小さく入れて到達性を補う
// KP: 応答性を維持しつつ発振抑制
// KI: 目標到達性のため少量
// 安全対策: PWM±1599クリッピング + Watchdog(1.5秒)で完全保護
#define KP 1000.0f
#define KI 800.0f
#define PWM_BIAS 0.0f

float SpeedControl(float target_velocity, float current_velocity, float *integral)
{
	float e = target_velocity - current_velocity;
	*integral += e * DELTA_T; // I制御

	// 積分ワインドアップ防止（±10で制限）
	if (*integral > 10.0f)
		*integral = 10.0f;
	if (*integral < -10.0f)
		*integral = -10.0f;

	float pwm = e * KP + *integral * KI;
	if (pwm > 0.0f)
	{
		pwm += PWM_BIAS;
	}
	else if (pwm < 0.0f)
	{
		pwm -= PWM_BIAS;
	}

	// PWM値のクリッピング（オーバーフロー防止）
	// MAX_COUNTER_PERIOD = 1599 の範囲内に制限
	if (pwm > 1599.0f)
		pwm = 1599.0f;
	if (pwm < -1599.0f)
		pwm = -1599.0f;

	return pwm;
}
