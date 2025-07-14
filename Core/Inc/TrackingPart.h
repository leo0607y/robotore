#ifndef INC_TRACKINGPART_H_
#define INC_TRACKINGPART_H_

#define COURSEOUT 20

#include "stm32f4xx_hal.h"
#include "motor.h"
#include "SpeedControl.h"
#include "TrackingSensor.h"
#include "main.h"

void ControlLineTracking(void);
void TraceFlip(void);

float getLineFollowingTerm(void);

void startTracking(void);
void stopTracking(void);

void CourseOut(void);
bool getUnableToRunFlag(void);

void debugmotor(float, float);

#endif /* INC_TRACKINGPART_H_ */