#include "Buttan.h"

uint16_t StatusL(uint8_t buttan)
{
    uint16_t ret = 0;
    if (buttan == 'L' && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 0)
    {
        ret = 1;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // PA15のLEDを点灯
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // PB14のLEDを消灯
    }
    return ret;
}

uint16_t StatusR(uint8_t buttan)
{
    uint16_t ret = 0;
    if (buttan == 'R' && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 0)
    {
        ret = 2;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // PA15のLEDを点灯
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // PA15のLEDを消灯
    }
    return ret;
}
