#ifndef INC_TRACKINGPART_H_
#define INC_TRACKINGPART_H_

#define COURSEOUT 40

#include "stm32f4xx_hal.h"
#include "motor.h"
#include <SpeedControl.h>
#include "TrackingSensor.h"
#include "main.h"

extern int8_t trace_flag; // 追加
extern bool is_on_tracking_curve;

void ControlLineTracking(void);
void TraceFlip(void);

float getLineFollowingTerm(void);

void startTracking(void);
void stopTracking(void);

void CourseOut(void);
bool getUnableToRunFlag(void);

void debugmotor(float, float);

extern float tracking_term;

void setTarget(float speed);
float getTarget(void);

#endif /* INC_TRACKINGPART_H_ */
