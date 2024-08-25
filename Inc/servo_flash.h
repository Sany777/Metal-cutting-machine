#ifndef _FLASH_H
#define _FLASH_H

#include "main.h"
#include "stdbool.h"


#ifdef __cplusplus
extern "C" {
#endif

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
//#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
//#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */

#define FLASH_START_ADDR  			ADDR_FLASH_SECTOR_5   /* Start @ of user Flash area */
#define FLASH_END_ADDR				(FLASH_START_ADDR+(1024*64) -1)


bool writeToFlash(uint8_t *dataIn, uint32_t dataSize);
void readFlash( uint8_t *buf, uint32_t dataSize );





#ifdef __cplusplus
}
#endif






#endif
