#include "TrackingSensor.h"

ADC_HandleTypeDef hadc1;

float values_max[ADC_NUM];
float values_min[ADC_NUM] = {1000};

static uint16_t adc_values[ADC_NUM];

static int16_t 0_buffer [10];
static int16_t 1_buffer [10];
static int16_t 2_buffer [10];
static int16_t 3_buffer [10];
static int16_t 4_buffer [10];
static int16_t 5_buffer [10];
static int16_t 6_buffer [10];
static int16_t 7_buffer [10];
static int16_t 8_buffer [10];
static int16_t 9_buffer [10];
static int16_t 10_buffer [10];
static int16_t 11_buffer [10];

static uint16_t L_index = 0;

void ADC_Init()
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, ADC_NUM);
}

void StorageBuffer(void)
{
    0_buffer [L_index] = ((adc_values[0] - offset[0]) / coefficient[0]) * 1000;
    1_buffer [L_index] = ((adc_values[1] - offset[1]) / coefficient[1]) * 1000;
    2_buffer [L_index] = ((adc_values[2] - offset[2]) / coefficient[2]) * 1000;
    3_buffer [L_index] = ((adc_values[3] - offset[3]) / coefficient[3]) * 1000;
    4_buffer [L_index] = ((adc_values[4] - offset[4]) / coefficient[4]) * 1000;
    5_buffer [L_index] = ((adc_values[5] - offset[5]) / coefficient[5]) * 1000;
    6_buffer [L_index] = ((adc_values[6] - offset[6]) / coefficient[6]) * 1000;
    7_buffer [L_index] = ((adc_values[7] - offset[7]) / coefficient[7]) * 1000;
    8_buffer [L_index] = ((adc_values[8] - offset[8]) / coefficient[8]) * 1000;
    9_buffer [L_index] = ((adc_values[9] - offset[9]) / coefficient[9]) * 1000;
    10_buffer [L_index] = ((adc_values[10] - offset[10]) / coefficient[10]) * 1000;
    11_buffer [L_index] = ((adc_values[11] - offset[11]) / coefficient[11]) * 1000;
    L_index++;
}

void Sensor_Update(void)
{
    sensor[0] = ((adc_values[0] - offset[0]) / coefficient[0]) * 1000;
    sensor[1] = ((adc_values[1] - offset[1]) / coefficient[1]) * 1000;
    sensor[2] = ((adc_values[2] - offset[2]) / coefficient[2]) * 1000;
    sensor[3] = ((adc_values[3] - offset[3]) / coefficient[3]) * 1000;
    sensor[4] = ((adc_values[4] - offset[4]) / coefficient[4]) * 1000;
    sensor[5] = ((adc_values[5] - offset[5]) / coefficient[5]) * 1000;
    sensor[6] = ((adc_values[6] - offset[6]) / coefficient[6]) * 1000;
    sensor[7] = ((adc_values[7] - offset[7]) / coefficient[7]) * 1000;
    sensor[8] = ((adc_values[8] - offset[8]) / coefficient[8]) * 1000;
    sensor[9] = ((adc_values[9] - offset[9]) / coefficient[9]) * 1000;
    sensor[10] = ((adc_values[10] - offset[10]) / coefficient[10]) * 1000;
    sensor[11] = ((adc_values[11] - offset[11]) / coefficient[11]) * 1000;

    for (int j = 0; j <= 11; j++)
    {
        if (sensor[j] >= 1000)
            sensor[j] = 1000;
        if (sensor[j] <= 0)
            sensor[j] = 0;
    }
    L_index = 0;
}

void Calibration(void)
{
    float val_max_buffer[ADC_NUM] = {0};
    float val_min_buffer[ADC_NUM] = {1000};
    for (uint16_t i = 0; i < ADC_NUM; i++)
    {
        values_max[i] = 0;
        values_min[i] = 1500;
        val_max_buffer[i] = 0;
        val_min_buffer[i] = 1500;
    }
    while (Status('L') == 1)
    {
        for (uint16_t i = 0; i < ADC_NUM; i++)
        {
            val_max_buffer[i] = adc_values[i];
            val_min_buffer[i] = adc_values[i];

            if (val_max_buffer[i] > values_max[i])
            {
                values_max[i] = val_max_buffer[i];
            }
            if (val_min_buffer[i] < values_min[i])
            {
                values_min[i] = val_min_buffer[i];
            }
        }
    }
    for (uint16_t i = 0; i < ADC_NUM; i++)
    {
        coefficient[i] = (values_max[i] - values_min[i]);
    }
    for (uint16_t i = 0; i < ADC_NUM; i++)
    {
        offset[i] = values_min[i];
    }
}
