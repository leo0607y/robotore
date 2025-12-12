/*
 * SpeedControl.c
 *
 *  Created on: Jul 22, 2025
 *      Author: reoch
 */

#define DELTA_T 0.001

// 積分ゲインを大幅増加：定常偏差をゼロに近づける
// 負荷時（地面走行）でも目標速度を維持するために必要
#define KP 300.0f
#define KI 50.0f  // 5.0 → 50.0 (10倍)

float SpeedControl(float target_velocity, float current_velocity, float *integral)
{
	float e = target_velocity - current_velocity;
	*integral += e * DELTA_T; // I制御

	// 積分ワインドアップ防止（より広い範囲を許容）
	if (*integral > 10.0f)
		*integral = 10.0f;
	if (*integral < -10.0f)
		*integral = -10.0f;

	return e * KP + *integral * KI;
}
