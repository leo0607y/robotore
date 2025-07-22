/*
 * SpeedControl.h
 *
 *  Created on: Jul 22, 2025
 *      Author: reoch
 */

#ifndef INC_SPEEDCONTROL_H_
#define INC_SPEEDCONTROL_H_

#include "stm32f4xx_hal.h"

float SpeedControl(float target_velocity, float current_velocity, float *integral);

#endif /* INC_SPEEDCONTROL_H_ */
