/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* ************************************************************ */
/* DataFlash_REVOMINI Log library                                 */
/* ************************************************************ */
#ifndef __DATAFLASH_REVOMINI_H__
#define __DATAFLASH_REVOMINI_H__

#include <AP_HAL.h>
#include "DataFlash.h"

// flash size
#define DF_FLASH_SIZE 8192 // pages * 256 byte
#define DF_eeNumPages 256  // 256 * 256 byte = 65536 byte of Parameters
#define DF_LAST_PAGE 7680  // 0x1e00
#define DF_eeStartPage DF_LAST_PAGE

class DataFlash_REVOMINI : public DataFlash_Block
{
private:
    void              WaitReady();
    uint8_t           ReadStatusReg();
    uint8_t           ReadStatus();
    uint16_t          PageSize();
    void              Flash_Jedec_WriteEnable();
    void 	      Flash_Jedec_EraseSector(uint32_t chip_offset);
    void	      BlockWrite(uint32_t IntPageAdr, const void *pHeader, uint8_t hdr_size, const void *pBuffer, uint16_t size);
    bool              BlockRead(uint32_t IntPageAdr, void *pBuffer, uint16_t size);
    
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    // take a semaphore safely
    bool	      _sem_take(uint8_t timeout);

public:
    void        Init();
    void        ReadManufacturerID();
    bool        CardInserted();
};

#endif
