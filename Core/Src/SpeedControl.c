/*
 * SpeedControl.c
 *
 *  Created on: Jul 22, 2025
 *      Author: reoch
 */

#define DELTA_T 0.001

#define KP 300.0f
#define KI 5.0f

float SpeedControl(float target_velocity, float current_velocity, float *integral)
{
	float e = target_velocity - current_velocity;
	*integral += e * DELTA_T; // I制御

	// 積分ワインドアップ防止
	if (*integral > 500.0f)
		*integral = 500.0f;
	if (*integral < -500.0f)
		*integral = -500.0f;

	return e * KP + *integral * KI;
}
