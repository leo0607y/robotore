#ifndef INC_RUNNINGSECTION_H_
#define INC_RUNNINGSECTION_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "TrackingSensor.h"
#include "FullColorLED.h"
#include "TrackingPart.h"
#include "Encoder.h"
#include "math.h"

void updateSideSensorStatus();
bool getSideSensorStatusL();
bool getSideSensorStatusR();
bool getgoalStatus();
void SideMarker();
void startstop(void);
void updateSideSensorStatus();
void S_Sensor();

void setRunMode(uint16_t);
bool isCrossLine();
// bool isContinuousCurvature();
bool isTargetDistance(float);

void running();
void runningFlip();
void runningInit();

void saveLog();
void startLogging();
void stopLogging();
void startVelocityUpdate();
void stopVelocityUpdate();

void createVelocityTable();
float radius2Velocity(float);
void addDecelerationDistanceMergin(float *, int16_t);
void addAccelerationDistanceMergin(float *, int16_t);
void decelerateProcessing(const float, const float *);
// void accelerateProcessing(const float, const float *);
void updateTargetVelocity();
void correctionTotalDistanceFromCrossLine();
void correctionTotalDistanceFromSideLine();
void CreateAcceleration(const float *);
void SaveVelocityTable();

void setVelocityRange(float, float);
void setAccDec(float, float);
void setStraightRadius(float);

#endif
