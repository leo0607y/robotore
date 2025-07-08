#ifndef INC_TRACKINGSENSOR_H_
#define INC_TRACKINGSENSOR_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "Buttan.h"
#include "main.h"
#include "FullColorLED.h"

#define ADC_NUM 12 // ADCのチャンネル数

float coefficient[ADC_NUM];
float offset[ADC_NUM];

int16_t sensor[ADC_NUM];

void ADC_Init(void);
void StorageBuffer(void);
void Sensor_Update(void);
void Sensor_Calibration(void);

#endif