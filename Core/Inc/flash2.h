/*
 * flash2.h
 *
 *  Created on: Sep 25, 2025
 *      Author: reoch
 */

#ifndef INC_FLASH2_H_
#define INC_FLASH2_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdint.h>
void eraseFlash( void );

void writeFlash(uint32_t address, uint8_t *data, uint32_t size  );

void loadFlash(uint32_t address, uint8_t *data, uint32_t size );


#endif /* INC_FLASH2_H_ */
