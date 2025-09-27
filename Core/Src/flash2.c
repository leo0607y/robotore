/*
 * flash2.c
 *
 *  Created on: Sep 25, 2025
 *      Author: reoch
 */
#include "flash2.h"

const uint32_t start_address = 0x80E0000; //sentor11 start address
const uint32_t end_adress = 0x80FFFFF; // sector11 end address

/*
 *@brief erase sector11
*/
void eraseFlash( void )
{
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_SECTORS;	// select sector
	erase.Sector = FLASH_SECTOR_11;		       // set selector11
	erase.NbSectors = 1;		// set to erase one sector
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;	// set voltage range (2.7 to 3.6V)

	uint32_t pageError = 0;


	HAL_FLASHEx_Erase(&erase, &pageError);	// erase sector
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data write data
 * @param uint32_t size write data size
*/
void writeFlash(uint32_t address, uint8_t *data, uint32_t size  )
{
	 HAL_FLASH_Unlock();
	  eraseFlash();

	  uint32_t *source_data = (uint32_t *)data; // uint32_tポインタにキャスト
	  uint32_t num_words = size / sizeof(uint32_t); // 書き込むワード数を計算

	  for (uint32_t i = 0; i < num_words; i++) {
	    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + (i * sizeof(uint32_t)), source_data[i]);
	  }

	  HAL_FLASH_Lock();
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data read data
 * @param uint32_t size read data size
*/
void loadFlash(uint32_t address, uint8_t *data, uint32_t size )
{
	memcpy(data, (uint8_t*)address, size); // copy data
}
