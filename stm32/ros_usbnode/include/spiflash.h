#ifndef __SPIFLASH_H
#define __SPIFLASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


void SPIFLASH_Config(void);
void SPIFLASH_IncBootCounter(void);
uint8_t SPIFLASH_TestDevice(void);
void SPIFLASH_WriteCfgValue(const char *name, uint8_t type, uint8_t len, uint8_t *data);
uint8_t SPIFLASH_ReadCfgValue(const char *name, uint8_t *type, uint8_t *data);
int SPIFLASH_ListDir(const char *path);
double SPIFLASH_ReadDouble(char *name);

uint8_t FLASH25D_Busy(void);
uint8_t FLASH25D_ReadStatusReg(void);
void FLASH25D_Read(uint8_t *buffer, uint32_t addr, uint32_t size);
void FLASH25D_Write_NoCheck(uint8_t *buffer, uint32_t addr, uint32_t size);
void FLASH25D_Erase_Sector(uint32_t addr);
void FLASH25D_WriteEnable(void);

#ifdef __cplusplus
}
#endif

#endif  /* __SPIFLASH_H */