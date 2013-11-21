/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* ************************************************************ */
/* DataFlash_REVOMINI Log library                                 */
/* ************************************************************ */
#ifndef __DATAFLASH_REVOMINI_H__
#define __DATAFLASH_REVOMINI_H__

#include <AP_HAL.h>
#include "DataFlash.h"

// flash size
#define DF_LAST_PAGE 0x1f00

class DataFlash_REVOMINI : public DataFlash_Block
{
private:
    void              WaitReady();
    uint8_t           ReadStatusReg();
//    uint8_t           ReadStatus();
    uint16_t          PageSize();
    void              Flash_Jedec_WriteEnable();
    void 	      Flash_Jedec_EraseSector(uint32_t chip_offset);
    void              BufferToPage (uint32_t IntPageAdr);
    void	      BlockWrite(uint32_t BufferIdx, const void *pHeader, uint8_t hdr_size, const void *pBuffer, uint16_t size);
    bool              BlockRead(uint32_t IntPageAdr, void *pBuffer, uint16_t size);
    
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    // take a semaphore safely
    bool	      _sem_take(uint8_t timeout);

public:
    void        Init();
    void        ReadManufacturerID();
    bool        CardInserted();
    uint8_t     ReadStatus();
};

#endif
