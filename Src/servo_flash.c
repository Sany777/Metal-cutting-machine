#include <servo_flash.h>

#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"



/**
 * @brief Function implementing read memory
 * @param argument: pointer buf, expected data size
 * @retval None
 */
void readFlash( uint8_t *buf, uint32_t dataSize )
{
	__disable_irq();
	uint32_t Address = FLASH_START_ADDR;
	for (uint32_t i = 0; i < dataSize; i++, Address++) {
		buf[i] = *(__IO uint8_t*) Address;
	}
	__enable_irq();
}


/**
 * @brief Function implementing write data to flash
 * @param argument: data buf, data size
 * @retval HAL_StatusTypeDef
 */
bool writeToFlash( uint8_t *dataIn, uint32_t dataSize )
{
	__disable_irq();
	FLASH_EraseInitTypeDef EraseInitStruct;
	HAL_FLASH_Unlock();
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FLASH_SECTOR_5;
	EraseInitStruct.NbSectors = 1;
	uint32_t SectorError;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
		return false;
	}

	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_INSTRUCTION_CACHE_RESET();

	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();

	uint32_t Address = FLASH_START_ADDR;
	for (uint32_t i = 0; i < dataSize; i++, Address++) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address, dataIn[i]) != HAL_OK) {
			return false;
		}
	}
	HAL_FLASH_Lock();
	__enable_irq();
	return true;
}
