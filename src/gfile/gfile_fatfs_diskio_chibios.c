/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/

#include "../../gfx.h"
#if HAL_USE_QSPI
#include "qspiFlash.h"
#endif

#if GFX_USE_GFILE && GFILE_NEED_FATFS && GFX_USE_OS_CHIBIOS && !GFILE_FATFS_EXTERNAL_LIB

#include "gfile_fatfs_wrapper.h"

#if HAL_USE_MMC_SPI && HAL_USE_SDC
#error "cannot specify both MMC_SPI and SDC drivers"
#endif

#if HAL_USE_QSPI
extern QSPIDriver QSPID1;
typedef struct {
  /**
    * буфер для одного сектора в 64 кБайта
    */
    uint8_t first[32768];
    uint8_t second[32768];
    uint8_t temp[256]; //без него нормально не считывает
  /**
   * номер сектора
   */
  int16_t numberSector;
} SectorBuffer;

SectorBuffer mainBuf;
#elif HAL_USE_MMC_SPI
extern MMCDriver MMCD1;
#elif HAL_USE_SDC
extern SDCDriver SDCD1;
#else
#error "MMC_SPI or SDC driver must be specified"
#endif

#if HAL_USE_RTC
#include "chrtclib.h"
extern RTCDriver RTCD1;
#endif

/*-----------------------------------------------------------------------*/
/* Correspondence between physical drive number and physical drive.      */

#define MMC     0
#define SDC     0
#define QSPI    0


/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */

DSTATUS disk_initialize (
    BYTE drv                /* Physical drive nmuber (0..) */
)
{
  DSTATUS stat;

  switch (drv) {
#if HAL_USE_MMC_SPI
  case MMC:
    stat = 0;
    /* It is initialized externally, just reads the status.*/
    if (blkGetDriverState(&MMCD1) != BLK_READY)
      stat |= STA_NOINIT;
    if (mmcIsWriteProtected(&MMCD1))
      stat |=  STA_PROTECT;
    return stat;
#elif HAL_USE_QSPI
  case QSPI:
    mainBuf.numberSector = -1;
    stat = 0;
    startQSPIConfiguredForQuad();
    return stat;
#else
  case SDC:
    stat = 0;
    /* It is initialized externally, just reads the status.*/
    if (blkGetDriverState(&SDCD1) != BLK_READY)
      stat |= STA_NOINIT;
    if (sdcIsWriteProtected(&SDCD1))
      stat |=  STA_PROTECT;
    return stat;
#endif
  }
  return STA_NODISK;
}



/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
    BYTE drv        /* Physical drive nmuber (0..) */
)
{
  DSTATUS stat;

  switch (drv) {
#if HAL_USE_MMC_SPI
  case MMC:
    stat = 0;
    /* It is initialized externally, just reads the status.*/
    if (blkGetDriverState(&MMCD1) != BLK_READY)
      stat |= STA_NOINIT;
    if (mmcIsWriteProtected(&MMCD1))
      stat |= STA_PROTECT;
    return stat;
#elif HAL_USE_QSPI
  case QSPI:
    stat = 0;
      /* It is initialized externally, just reads the status.*/
    if (getDriverState() != QSPI_READY) {
      stat |= STA_NOINIT;
    }
    return stat;
#else
  case SDC:
    stat = 0;
    /* It is initialized externally, just reads the status.*/
    if (blkGetDriverState(&SDCD1) != BLK_READY)
      stat |= STA_NOINIT;
    if (sdcIsWriteProtected(&SDCD1))
      stat |= STA_PROTECT;
    return stat;
#endif
  }
  return STA_NODISK;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */

DRESULT disk_read (
    BYTE drv,        /* Physical drive nmuber (0..) */
    BYTE *buff,        /* Data buffer to store read data */
    DWORD sector,    /* Sector address (LBA) */
    UINT count        /* Number of sectors to read (1..255) */
)
{
  switch (drv) {
#if HAL_USE_MMC_SPI
  case MMC:
    if (blkGetDriverState(&MMCD1) != BLK_READY)
      return RES_NOTRDY;
    if (mmcStartSequentialRead(&MMCD1, sector))
      return RES_ERROR;
    while (count > 0) {
      if (mmcSequentialRead(&MMCD1, buff))
        return RES_ERROR;
      buff += MMCSD_BLOCK_SIZE;
      count--;
    }
    if (mmcStopSequentialRead(&MMCD1))
        return RES_ERROR;
    return RES_OK;
#elif HAL_USE_QSPI
  case QSPI:
    quadIndirectReadMode(buff, count*512, sector*512);
    return RES_OK;
#else
  case SDC:
    if (blkGetDriverState(&SDCD1) != BLK_READY)
      return RES_NOTRDY;
    if (sdcRead(&SDCD1, sector, buff, count))
      return RES_ERROR;
    return RES_OK;
#endif
  }
  return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */

#if _READONLY == 0
DRESULT disk_write (
    BYTE drv,            /* Physical drive nmuber (0..) */
    const BYTE *buff,    /* Data to be written */
    DWORD sector,        /* Sector address (LBA) */
    UINT count            /* Number of sectors to write (1..255) */
)
{
  uint32_t bSize = 0;
  UINT tempCount = count;
  switch (drv) {
#if HAL_USE_MMC_SPI
  case MMC:
    if (blkGetDriverState(&MMCD1) != BLK_READY)
        return RES_NOTRDY;
    if (mmcIsWriteProtected(&MMCD1))
        return RES_WRPRT;
    if (mmcStartSequentialWrite(&MMCD1, sector))
        return RES_ERROR;
    while (count > 0) {
        if (mmcSequentialWrite(&MMCD1, buff))
            return RES_ERROR;
        buff += MMCSD_BLOCK_SIZE;
        count--;
    }
    if (mmcStopSequentialWrite(&MMCD1))
        return RES_ERROR;
    return RES_OK;
#elif HAL_USE_QSPI
  case QSPI:
//    __disable_irq();
    while(count > 0) {
      if(mainBuf.numberSector == sector*512 >> 16) {
          setBufferWithSector(sector, bSize, buff);
      } else if (mainBuf.numberSector == -1) {
          readExternalFlash(sector);
          setBufferWithSector(sector, bSize, buff);
      } else {
          writeBufferToFlash();
          readExternalFlash(sector);
          setBufferWithSector(sector, bSize, buff);
      }
      count--;
      sector++;
      bSize++;
    }

    writeBufferToFlash();
    mainBuf.numberSector = -1;
//    __enable_irq();
    return RES_OK;
#else
  case SDC:
    if (blkGetDriverState(&SDCD1) != BLK_READY)
      return RES_NOTRDY;
    if (sdcWrite(&SDCD1, sector, buff, count))
      return RES_ERROR;
    return RES_OK;
#endif
  }
  return RES_PARERR;
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
    BYTE drv,        /* Physical drive nmuber (0..) */
    BYTE ctrl,        /* Control code */
    void *buff        /* Buffer to send/receive control data */
)
{
  switch (drv) {
#if HAL_USE_MMC_SPI
  case MMC:
    switch (ctrl) {
    case CTRL_SYNC:
        return RES_OK;
    case GET_SECTOR_SIZE:
        *((WORD *)buff) = MMCSD_BLOCK_SIZE;
        return RES_OK;
#if _USE_ERASE
    case CTRL_ERASE_SECTOR:
        mmcErase(&MMCD1, *((DWORD *)buff), *((DWORD *)buff + 1));
        return RES_OK;
#endif
    default:
        return RES_PARERR;
    }
#elif HAL_USE_QSPI
    case QSPI:
      switch (ctrl) {
        case CTRL_SYNC:
          return RES_OK;
        case GET_SECTOR_COUNT:
          *((DWORD *)buff) = 32768;
          return RES_OK;
        case GET_BLOCK_SIZE:
          *((DWORD *)buff) = 128; /* 512b blocks in one erase block */
          return RES_OK;
      }
#else
  case SDC:
    switch (ctrl) {
    case CTRL_SYNC:
        return RES_OK;
    case GET_SECTOR_COUNT:
        *((DWORD *)buff) = mmcsdGetCardCapacity(&SDCD1);
        return RES_OK;
    case GET_SECTOR_SIZE:
        *((WORD *)buff) = MMCSD_BLOCK_SIZE;
        return RES_OK;
    case GET_BLOCK_SIZE:
        *((DWORD *)buff) = 256; /* 512b blocks in one erase block */
        return RES_OK;
#if _USE_ERASE
    case CTRL_ERASE_SECTOR:
        sdcErase(&SDCD1, *((DWORD *)buff), *((DWORD *)buff + 1));
        return RES_OK;
#endif
    default:
        return RES_PARERR;
    }
#endif
  }
  return RES_PARERR;
}

DWORD get_fattime(void) {
#if HAL_USE_RTC
    return rtcGetTimeFat(&RTCD1);
#else
    return ((uint32_t)0 | (1 << 16)) | (1 << 21); /* wrong but valid time */
#endif
}

void writeBufferToFlash() {
    volatile uint16_t i = 0;
    while(readStatusRegisterProgressBitQuadMode()) {}
    sectorErase(mainBuf.numberSector << 16);
    while(readStatusRegisterProgressBitQuadMode()) {}

    for(i; i < 256; i++) {
      if(i*256 > 32767) {
        quadIndirectWriteMode(mainBuf.second + i*256 - 32768, 256,
                              (mainBuf.numberSector << 16) + i*256);
        while(readStatusRegisterProgressBitQuadMode()) {}

      } else {
        quadIndirectWriteMode(mainBuf.first + i*256, 256,
                              (mainBuf.numberSector << 16) + i*256);
        while(readStatusRegisterProgressBitQuadMode()) {}

      }
    }
}

void setBufferWithSector(DWORD sector, uint32_t bSize, const BYTE *buff) {
  uint32_t baseAddr = (sector*512) & 0xFFFF;
  for(uint16_t i = 0; i < 512; i++) {
    if(i > 32767) {
      mainBuf.second[baseAddr + i] = buff[bSize*512 + i];
    } else {
      mainBuf.first[baseAddr + i] = buff[bSize*512 + i];
    }
  }
}

void readExternalFlash(DWORD sector) {
    mainBuf.numberSector = sector*512 >> 16;
    quadIndirectReadMode(mainBuf.first, 32768, mainBuf.numberSector << 16);
    quadIndirectReadMode(mainBuf.second, 32768, (mainBuf.numberSector << 16) + 32768);
}

#endif // GFX_USE_GFILE && GFILE_NEED_FATFS && GFX_USE_OS_CHIBIOS && !GFILE_FATFS_EXTERNAL_LIB


