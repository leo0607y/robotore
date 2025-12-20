#ifndef INC_RUNNINGSECTION_H_
#define INC_RUNNINGSECTION_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "TrackingSensor.h"
#include "FullColorLED.h"
#include "TrackingPart.h"
#include "Encoder.h"
#include "math.h"
#include "tim.h"
#include <stdlib.h>

// void updateSideSensorStatus();
void Fan_Ctrl();
void FanMotor(int16_t);
void S_Sensor();
void Reset_S_Sensor_State(void);

#endif
