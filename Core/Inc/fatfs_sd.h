#ifndef __FATFS_SD_H
#define __FATFS_SD_H

#include "stm32l4xx_hal.h" // Matches your Nucleo L4
#include "diskio.h"

// Define your hardware handle and CS layout
extern SPI_HandleTypeDef hspi1;
#define HSPI_SDCARD       hspi1
#define SD_CS_PORT        GPIOA
#define SD_CS_PIN         GPIO_PIN_10

/* Function prototypes to bind with user_diskio.c */
DSTATUS SD_disk_initialize(BYTE pdrv);
DSTATUS SD_disk_status(BYTE pdrv);
DRESULT SD_disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void* buff);

#endif
