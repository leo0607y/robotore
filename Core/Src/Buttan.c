#include "Buttan.h"

uint16_t Status(uint8_t buttan)
{
    uint16_t ret = 0;
    if (buttan == 'R' && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 0)
    {
        ret = 1;
    }
    else if (buttan == 'L' && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 0)
    {
        ret = 2;
    }
    return ret;
}
