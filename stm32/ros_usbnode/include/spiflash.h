#ifndef __SPIFLASH_H
#define __SPIFLASH_H

#include <stdint.h>

void SPIFLASH_Config(void);
uint8_t SPIFLASH_TestDevice(void);

uint8_t FLASH25D_Busy(void);
uint8_t FLASH25D_ReadStatusReg(void);
void FLASH25D_Read(uint8_t *buffer, uint32_t addr, uint32_t size);
void FLASH25D_Write_NoCheck(uint8_t *buffer, uint32_t addr, uint32_t size);
void FLASH25D_Erase_Sector(uint32_t addr);
void FLASH25D_WriteEnable(void);

#endif  /* __SPIFLASH_H */