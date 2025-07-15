#ifndef INC_TRACKINGSENSOR_H_
#define INC_TRACKINGSENSOR_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "Buttan.h"
#include "main.h"
#include "FullColorLED.h"

#define ADC_NUM 12 // ADCのチャンネル数

extern float coefficient[ADC_NUM];
extern float offset[ADC_NUM];
extern int16_t sensor[ADC_NUM];
extern uint32_t adc_values[ADC_NUM]; // ADC値のextern宣言を追加

void ADC_Init(void);
void StorageBuffer(void);
void Sensor_Update(void);
void Calibration(void);

#endif
