/**
  ******************************************************************************
  * @file    spiflash.c
  * @author  Georg Swoboda <cn@warp.at>
  * @brief   LittleFS integration for Mowgli
  ******************************************************************************
  * @attention
  *
  * https://uimeter.com/2018-04-12-Try-LittleFS-on-STM32-and-SPI-Flash/
  ******************************************************************************
  */

#include "spiflash.h"
#include "stm32f1xx_hal.h"
#include "board.h"
#include "main.h"
#include "littlefs/lfs.h"
#include "littlefs/lfs_util.h"

// #define SPIFLASH_DEBUG 1

lfs_t lfs;
lfs_file_t file;
struct lfs_config cfg;

uint8_t lfs_read_buf[256];
uint8_t lfs_prog_buf[256];
uint8_t lfs_lookahead_buf[16];	// 128/8=16
uint8_t lfs_file_buf[256];


/*********************************************************************************************************
 * SPI Flash Driver for 25D40/20 SPI Flash
 *********************************************************************************************************/

uint8_t FLASH25D_Busy(void)
{
    return(FLASH25D_ReadStatusReg() & 0x1);
}

uint8_t FLASH25D_ReadStatusReg(void)
{
    uint8_t tx_buf[8], rx_buf[8];    //Buffers     
    tx_buf[0] = 0x5; // READ STATUS REG
    
    HAL_GPIO_WritePin(FLASH_SPICS_PORT, FLASH_nCS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&SPI3_Handle, tx_buf, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&SPI3_Handle, rx_buf, 1, HAL_MAX_DELAY);   //Returns the right value             
    HAL_GPIO_WritePin(FLASH_SPICS_PORT, FLASH_nCS_PIN, GPIO_PIN_SET);
    return(rx_buf[0]);
}

void FLASH25D_WriteEnable(void)
{
    uint8_t tx_buf[8];  //Buffers     
    tx_buf[0] = 0x6; // WRITE ENABLE
    
    HAL_GPIO_WritePin(FLASH_SPICS_PORT, FLASH_nCS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&SPI3_Handle, tx_buf, 1, HAL_MAX_DELAY);    
    HAL_GPIO_WritePin(FLASH_SPICS_PORT, FLASH_nCS_PIN, GPIO_PIN_SET);

#ifdef SPIFLASH_DEBUG
   debug_printf("WEL Bit: %x\r\n", (FLASH25D_ReadStatusReg()&2)>>1);    
   debug_printf("BP2 BP1 Bp0 Bits:  %x %x %x\r\n", (FLASH25D_ReadStatusReg()&16)>>4, (FLASH25D_ReadStatusReg()&8)>>3, (FLASH25D_ReadStatusReg()&4)>>2);    
#endif   
}

void FLASH25D_Read(uint8_t *buffer, uint32_t addr, uint32_t size)
{
    uint8_t tx_buf[8];    //Buffers     
    tx_buf[0] = 0x3; // READ
    tx_buf[1] = (addr>>16)&0xFF;
    tx_buf[2] = (addr>>8)&0xFF;
    tx_buf[3] = addr&0xFF;;

#ifdef SPIFLASH_DEBUG
   debug_printf("FLASH25D_Read: addr=0x%x len=%d\r\n", addr, size);       
#endif  

    HAL_GPIO_WritePin(FLASH_SPICS_PORT, FLASH_nCS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&SPI3_Handle, tx_buf, 4, HAL_MAX_DELAY);    
    HAL_SPI_Receive(&SPI3_Handle, buffer, size, HAL_MAX_DELAY);   //Returns the right value                         
    HAL_GPIO_WritePin(FLASH_SPICS_PORT, FLASH_nCS_PIN, GPIO_PIN_SET);
}

void FLASH25D_Write_NoCheck(uint8_t *buffer, uint32_t addr, uint32_t size)
{
    uint8_t tx_buf[8];    //Buffers    
    tx_buf[0] = 0x2; // PAGE PROGRAMM 
    tx_buf[1] = (addr>>16)&0xFF;
    tx_buf[2] = (addr>>8)&0xFF;
    tx_buf[3] = addr&0xFF;;

#ifdef SPIFLASH_DEBUG
   debug_printf("FLASH25D_Write_NoCheck: addr=0x%x len=%d\r\n", addr, size);       
#endif  

    HAL_GPIO_WritePin(FLASH_SPICS_PORT, FLASH_nCS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&SPI3_Handle, tx_buf, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&SPI3_Handle, buffer, size, HAL_MAX_DELAY);            
    HAL_GPIO_WritePin(FLASH_SPICS_PORT, FLASH_nCS_PIN, GPIO_PIN_SET);
    while (FLASH25D_Busy()) { };
}

void FLASH25D_Erase_Sector(uint32_t addr)
{
    uint8_t tx_buf[8];    //Buffers 

    tx_buf[0] = 0x20; // SECTOR ERASE (4KB)
    tx_buf[1] = (addr>>16)&0xFF;
    tx_buf[2] = (addr>>8)&0xFF;
    tx_buf[3] = addr&0xFF;;

#ifdef SPIFLASH_DEBUG
    debug_printf("FLASH25D_Erase_Sector addr=0x%x\r\n", addr);
#endif
    HAL_GPIO_WritePin(FLASH_SPICS_PORT, FLASH_nCS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&SPI3_Handle, tx_buf, 4, HAL_MAX_DELAY);    
    HAL_GPIO_WritePin(FLASH_SPICS_PORT, FLASH_nCS_PIN, GPIO_PIN_SET);    
    while (FLASH25D_Busy()) { };
}

/********************************************************************************************************
 * LittleFS Configuration
 ********************************************************************************************************/

int block_device_read(const struct lfs_config *c, lfs_block_t block,
	lfs_off_t off, void *buffer, lfs_size_t size)
{        
	FLASH25D_Read((uint8_t*)buffer, (block * c->block_size + off), size);
	return 0;
}

int block_device_prog(const struct lfs_config *c, lfs_block_t block,
	lfs_off_t off, const void *buffer, lfs_size_t size)
{  
    FLASH25D_WriteEnable();
	FLASH25D_Write_NoCheck((uint8_t*)buffer, (block * c->block_size + off), size);	
	return 0;
}

int block_device_erase(const struct lfs_config *c, lfs_block_t block)
{   
    FLASH25D_WriteEnable();
	FLASH25D_Erase_Sector(block * c->block_size);	
	return 0;
}

int block_device_sync(const struct lfs_config *c)
{  
	return 0;
}

/**
 * @brief Initialize LittleFS and increase bootcounter 
 */
void SPIFLASH_Config(void)
{
	// block device operations
	cfg.read  = block_device_read;
	cfg.prog  = block_device_prog;
	cfg.erase = block_device_erase;
	cfg.sync  = block_device_sync;

	// block device configuration
	cfg.read_size = 256;
	cfg.prog_size = 256;
	cfg.block_size = 4096;
	cfg.block_count = 128;
	cfg.lookahead_size = 256;
    cfg.cache_size = 256;
    cfg.block_cycles = 500;    


    SPI3_Init();
    debug_printf("   >> LittleFS: mounting ... ");
    int err = lfs_mount(&lfs, &cfg);
    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        debug_printf("ERROR: no valid filesystem found, formatting ...");
        lfs_format(&lfs, &cfg);        
        err = lfs_mount(&lfs, &cfg);
    }
    

    if (!err) // filesystem or, or reformat of filesystem is ok
    {
        debug_printf("OK\r\n");
        // read current count
        uint32_t boot_count = 0;
        lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));

        // update boot count
        boot_count += 1;
        lfs_file_rewind(&lfs, &file);
        lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));

        // remember the storage is not updated until the file is closed successfully
        lfs_file_close(&lfs, &file);

        // release any resources we were using
        lfs_unmount(&lfs);

        // print the boot count
        debug_printf("   >> boot_count: %d\r\n", boot_count);
    }   
    SPI3_DeInit();
}


/**
 * @brief Init and test SPI Flash attached to SPI3
 * @retval 1 if device found, 0 otherwise
 */
uint8_t SPIFLASH_TestDevice(void)
{
    uint8_t tx_buf[8], rx_buf[8];    //Buffers for the first read.
    tx_buf[0] = 0x9F; // ID Code

    SPI3_Init();
    HAL_GPIO_WritePin(FLASH_SPICS_PORT, FLASH_nCS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&SPI3_Handle, tx_buf, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&SPI3_Handle, rx_buf, 3, HAL_MAX_DELAY);   //Returns the right value
    HAL_GPIO_WritePin(FLASH_SPICS_PORT, FLASH_nCS_PIN, GPIO_PIN_SET);

    debug_printf(" * SPI3 SPIFlash ID: 0x%02x 0x%02x 0x%02x\r\n", rx_buf[0], rx_buf[1], rx_buf[2]);
    SPI3_DeInit();
    return(rx_buf[0]>0); // manufactorer id present
}