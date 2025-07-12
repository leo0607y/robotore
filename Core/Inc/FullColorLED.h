#ifndef INC_LED_H_
#define INC_LED_H_

#include "stm32f4xx_hal.h"

typedef enum
{
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_WHITE,
    LED_YELLOW,
    LED_CYAN,
    LED_MAGENTA,
    LED_OFF
} led_color_t;

void LED(led_color_t c);

#endif /* INC_LED_H_ */
