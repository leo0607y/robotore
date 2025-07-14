#ifndef SPEEDCONTROL_H
#define SPEEDCONTROL_H

#include "stm32f4xx_hal.h"
#include "Encoder.h"
#include "motor.h"

void calculateVelocityControlFlip(void);
float getVelocityControlTerm(void);

float getCurrentVelocity(void);
float getTargetVelocity(void);
float getpidplus(void);
float getTargetAcceleration(void);
float setvariablespeed(void);
void setTargetVelocity(float);
void setTargetAcceleration(float);

void startVelocityControl(void);
void stopVelocityControl(void);

void setClearFlagOfVelocityControlI();

void setrunmode(uint16_t);

extern uint16_t mode;

#endif // SPEEDCONTROL_H
