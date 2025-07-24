/*
 * SpeedControl.c
 *
 *  Created on: Jul 22, 2025
 *      Author: reoch
 */

#define DELTA_T 0.001

#define KP 1800.0
#define KI 20.0

float SpeedControl(float target_velocity, float current_velocity, float *integral) {
	float e = target_velocity - current_velocity;
	*integral += e * DELTA_T; // I制御
	return e * KP + *integral * KI;
}
