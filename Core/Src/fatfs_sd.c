#include "fatfs_sd.h"

static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType = 0;

#define CMD0   (0)
#define CMD1   (1)
#define CMD8   (8)
#define CMD9   (9)
#define CMD10  (10)
#define CMD12  (12)
#define CMD16  (16)
#define CMD17  (17)
#define CMD18  (18)
#define CMD23  (23)
#define CMD24  (24)
#define CMD25  (25)
#define CMD55  (55)
#define CMD58  (58)
#define ACMD41 (0x80 + 41)

#define CT_MMC  0x01
#define CT_SD1  0x02
#define CT_SD2  0x04
#define CT_SDC  (CT_SD1 | CT_SD2)
#define CT_BLOCK 0x08

static void CS_LOW(void)
{
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
}

static void CS_HIGH(void)
{
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
}

static uint8_t SPI_TxRx(uint8_t data)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&HSPI_SDCARD, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

static void SPI_Clock(void)
{
    SPI_TxRx(0xFF);
}

static uint8_t SD_WaitReady(uint32_t timeout)
{
    uint32_t tick = HAL_GetTick();
    uint8_t res;

    do {
        res = SPI_TxRx(0xFF);
        if (res == 0xFF) return 1;
    } while ((HAL_GetTick() - tick) < timeout);

    return 0;
}

static void SD_Deselect(void)
{
    CS_HIGH();
    SPI_Clock();
}

static uint8_t SD_Select(void)
{
    CS_LOW();
    SPI_Clock();

    if (SD_WaitReady(500)) return 1;

    SD_Deselect();
    return 0;
}

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
    uint8_t n, res;

    if (cmd & 0x80) {
        cmd &= 0x7F;
        res = SD_SendCmd(CMD55, 0);
        if (res > 1) return res;
    }

    SD_Deselect();

    if (!SD_Select()) return 0xFF;

    SPI_TxRx(0x40 | cmd);
    SPI_TxRx((uint8_t)(arg >> 24));
    SPI_TxRx((uint8_t)(arg >> 16));
    SPI_TxRx((uint8_t)(arg >> 8));
    SPI_TxRx((uint8_t)arg);

    n = 0x01;
    if (cmd == CMD0) n = 0x95;
    if (cmd == CMD8) n = 0x87;

    SPI_TxRx(n);

    if (cmd == CMD12) SPI_Clock();

    n = 10;
    do {
        res = SPI_TxRx(0xFF);
    } while ((res & 0x80) && --n);

    return res;
}

static uint8_t SD_RxDataBlock(uint8_t *buff, UINT btr)
{
    uint8_t token;
    uint32_t tick = HAL_GetTick();

    do {
        token = SPI_TxRx(0xFF);
        if (token == 0xFE) break;
    } while ((HAL_GetTick() - tick) < 200);

    if (token != 0xFE) return 0;

    do {
        *buff++ = SPI_TxRx(0xFF);
    } while (--btr);

    SPI_TxRx(0xFF);
    SPI_TxRx(0xFF);

    return 1;
}

static uint8_t SD_TxDataBlock(const uint8_t *buff, uint8_t token)
{
    uint8_t resp;

    if (!SD_WaitReady(500)) return 0;

    SPI_TxRx(token);

    if (token != 0xFD) {
        for (UINT i = 0; i < 512; i++) {
            SPI_TxRx(buff[i]);
        }

        SPI_TxRx(0xFF);
        SPI_TxRx(0xFF);

        resp = SPI_TxRx(0xFF);

        if ((resp & 0x1F) != 0x05) return 0;
    }

    return 1;
}

DSTATUS SD_disk_initialize(BYTE pdrv)
{
    uint8_t n, cmd, ty, ocr[4];

    if (pdrv) return STA_NOINIT;

    CS_HIGH();
    HAL_Delay(300);

    // Extra clocks with CS high for fresh SD power-up
    for (n = 0; n < 30; n++) {
        SPI_Clock();
    }

    HAL_Delay(10);

    // Force deselect again
    CS_HIGH();
    SPI_Clock();
    ty = 0;

    if (SD_SendCmd(CMD0, 0) == 1) {
        if (SD_SendCmd(CMD8, 0x1AA) == 1) {
            for (n = 0; n < 4; n++) {
                ocr[n] = SPI_TxRx(0xFF);
            }

            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
                uint32_t tick = HAL_GetTick();

                while ((HAL_GetTick() - tick) < 1000) {
                    if (SD_SendCmd(ACMD41, 1UL << 30) == 0) break;
                }

                if ((HAL_GetTick() - tick) < 1000 && SD_SendCmd(CMD58, 0) == 0) {
                    for (n = 0; n < 4; n++) {
                        ocr[n] = SPI_TxRx(0xFF);
                    }

                    ty = (ocr[0] & 0x40) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
                }
            }
        } else {
            if (SD_SendCmd(ACMD41, 0) <= 1) {
                ty = CT_SD1;
                cmd = ACMD41;
            } else {
                ty = CT_MMC;
                cmd = CMD1;
            }

            uint32_t tick = HAL_GetTick();

            while ((HAL_GetTick() - tick) < 1000) {
                if (SD_SendCmd(cmd, 0) == 0) break;
            }

            if (!((HAL_GetTick() - tick) < 1000) || SD_SendCmd(CMD16, 512) != 0) {
                ty = 0;
            }
        }
    }

    CardType = ty;
    SD_Deselect();

    if (ty) {
        Stat &= ~STA_NOINIT;
    } else {
        Stat = STA_NOINIT;
    }

    return Stat;
}

DSTATUS SD_disk_status(BYTE pdrv)
{
    if (pdrv) return STA_NOINIT;
    return Stat;
}

DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & CT_BLOCK)) {
        sector *= 512;
    }

    if (count == 1) {
        if (SD_SendCmd(CMD17, sector) == 0) {
            if (SD_RxDataBlock(buff, 512)) {
                count = 0;
            }
        }
    } else {
        if (SD_SendCmd(CMD18, sector) == 0) {
            do {
                if (!SD_RxDataBlock(buff, 512)) break;
                buff += 512;
            } while (--count);

            SD_SendCmd(CMD12, 0);
        }
    }

    SD_Deselect();

    return count ? RES_ERROR : RES_OK;
}

DRESULT SD_disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
#if _USE_WRITE == 1
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & CT_BLOCK)) {
        sector *= 512;
    }

    if (count == 1) {
        if (SD_SendCmd(CMD24, sector) == 0) {
            if (SD_TxDataBlock(buff, 0xFE)) {
                count = 0;
            }
        }
    } else {
        if (CardType & CT_SDC) {
            SD_SendCmd(ACMD41, count);
        }

        if (SD_SendCmd(CMD25, sector) == 0) {
            do {
                if (!SD_TxDataBlock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);

            if (!SD_TxDataBlock(0, 0xFD)) {
                count = 1;
            }
        }
    }

    SD_Deselect();

    return count ? RES_ERROR : RES_OK;
#else
    return RES_WRPRT;
#endif
}

DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    DRESULT res;
    BYTE n, csd[16];
    DWORD csize;

    if (pdrv) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    res = RES_ERROR;

    switch (cmd) {
        case CTRL_SYNC:
            if (SD_Select()) res = RES_OK;
            SD_Deselect();
            break;

        case GET_SECTOR_SIZE:
            *(WORD*)buff = 512;
            res = RES_OK;
            break;

        case GET_BLOCK_SIZE:
            *(DWORD*)buff = 128;
            res = RES_OK;
            break;

        case GET_SECTOR_COUNT:
            if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16)) {
                if ((csd[0] >> 6) == 1) {
                    csize = csd[9] + ((WORD)csd[8] << 8) + 1;
                    *(DWORD*)buff = csize << 10;
                } else {
                    n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                    csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                    *(DWORD*)buff = csize << (n - 9);
                }
                res = RES_OK;
            }
            SD_Deselect();
            break;

        default:
            res = RES_PARERR;
    }

    return res;
}
