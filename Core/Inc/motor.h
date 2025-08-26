#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f4xx_hal.h"

#define MAX_COUNTER_PERIOD 1599
#define MIN_COUNTER_PERIOD -1599

#define STOP_COUNTER_PERIOD 0

void Motor_Init(void);
void motorCtrlFlip(void);
void droneMotorCtrlFlip(void);
void setMotor(int16_t, int16_t);

#endif /* INC_MOTOR_H_ */
