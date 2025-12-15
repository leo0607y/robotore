/*
 * SpeedControl.c
 *
 *  Created on: Jul 22, 2025
 *      Author: reoch
 */

#define DELTA_T 0.001

// PIゲイン: Pは適切、Iを大幅に下げて発振完全除去
// KP: 730（応答性と安定性のバランスが取れている）
// KI: 70（100でもPWM552/534と発振継続、70まで大幅低減）
// 積分上限: ±15.0（過度な積分を抑制、安定性とレスポンスの両立）
// 計算例: e=0.2時、PWM = 730*0.2 + 70*15 = 146+1050 = 1196
// 安全対策: PWM±1599クリッピング + Watchdog(1.5秒)で完全保護
#define KP 900.0f
#define KI 200.0f

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
