#include "TrackingSensor.h"
#include "adc.h"
#include <stdio.h>
#include <string.h>

// ADC_HandleTypeDef hadc1;

float values_max[ADC_NUM];
float values_min[ADC_NUM] = {1000};

extern uint32_t adc_values[ADC_NUM]; // adc_valuesの定義を削除し、extern宣言のみを使用

static int16_t buffer_0[10];
static int16_t buffer_1[10];
static int16_t buffer_2[10];
static int16_t buffer_3[10];
static int16_t buffer_4[10];
static int16_t buffer_5[10];
static int16_t buffer_6[10];
static int16_t buffer_7[10];
static int16_t buffer_8[10];
static int16_t buffer_9[10];
static int16_t buffer_10[10];
static int16_t buffer_11[10];

static uint16_t L_index = 0;

float coefficient[ADC_NUM];
float offset[ADC_NUM];
int16_t sensor[ADC_NUM];

UART_HandleTypeDef huart2;

extern DMA_HandleTypeDef hdma_adc1; // DMAハンドルのextern宣言を追加

void ADC_Init()
{
    // __HAL_RCC_DMA2_CLK_ENABLE();                                // DMAクロック有効化
    // HAL_DMA_DeInit(&hdma_adc1);                                 // DMA初期化解除
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, ADC_NUM); // ADC DMA開始
}

void StorageBuffer(void)
{
    buffer_0[L_index] = ((adc_values[0] - offset[0]) / coefficient[0]) * 1000;
    buffer_1[L_index] = ((adc_values[1] - offset[1]) / coefficient[1]) * 1000;
    buffer_2[L_index] = ((adc_values[2] - offset[2]) / coefficient[2]) * 1000;
    buffer_3[L_index] = ((adc_values[3] - offset[3]) / coefficient[3]) * 1000;
    buffer_4[L_index] = ((adc_values[4] - offset[4]) / coefficient[4]) * 1000;
    buffer_5[L_index] = ((adc_values[5] - offset[5]) / coefficient[5]) * 1000;
    buffer_6[L_index] = ((adc_values[6] - offset[6]) / coefficient[6]) * 1000;
    buffer_7[L_index] = ((adc_values[7] - offset[7]) / coefficient[7]) * 1000;
    buffer_8[L_index] = ((adc_values[8] - offset[8]) / coefficient[8]) * 1000;
    buffer_9[L_index] = ((adc_values[9] - offset[9]) / coefficient[9]) * 1000;
    buffer_10[L_index] = ((adc_values[10] - offset[10]) / coefficient[10]) * 1000;
    buffer_11[L_index] = ((adc_values[11] - offset[11]) / coefficient[11]) * 1000;
    L_index++;
}

void Sensor_Update(void)
{
    for (int j = 0; j <= 11; j++)
    {
        if (adc_values[j] >= values_max[j])
            adc_values[j] = values_max[j];
        if (adc_values[j] <= values_min[j])
            adc_values[j] = values_min[j];
    }

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

    //    for (int j = 0; j <= 11; j++)
    //    {
    //        if (sensor[j] >= 1000)
    //            sensor[j] = 1000;
    //        if (sensor[j] <= 0)
    //            sensor[j] = 0;
    //    }
    L_index = 0;
}

void Calibration(void)
{
    printf("Calibration started\n"); // キャリブレーション開始を通知

    float val_max_buffer[ADC_NUM] = {0};
    float val_min_buffer[ADC_NUM] = {1000};
    for (uint16_t i = 0; i < ADC_NUM; i++)
    {
        values_max[i] = 0;
        values_min[i] = 1500;
        val_max_buffer[i] = 0;
        val_min_buffer[i] = 1500;
    }
    while (StatusL('L') == 0)
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

        // キャリブレーション中のボタン状態をUARTで出力
        //        char debugMessage[50];
        //        snprintf(debugMessage, sizeof(debugMessage), "Status('L') = %d\r\n", Status('L'));
        //        HAL_UART_Transmit(&huart2, (uint8_t*)debugMessage, strlen(debugMessage), HAL_MAX_DELAY);
    }
    for (uint16_t i = 0; i < ADC_NUM; i++)
    {
        coefficient[i] = (values_max[i] - values_min[i]);
    }
    for (uint16_t i = 0; i < ADC_NUM; i++)
    {
        offset[i] = values_min[i];
    }
    printf("Calibration completed\n"); // キャリブレーション終了を通知
}
