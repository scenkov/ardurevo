/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <AP_HAL.h>
#include "DataFlash_REVOMINI.h"
#include <wirish.h>

extern const AP_HAL::HAL& hal;

#define ENABLE_FASTSERIAL_DEBUG

#ifdef ENABLE_FASTSERIAL_DEBUG
 #define serialDebug(fmt, args...)  do {hal.console->printf_P(PSTR( __FUNCTION__ ":%d:" fmt "\n"), __LINE__, ##args); } while(0)
#else
 # define serialDebug(fmt, args...)
#endif

#define DF_RESET BOARD_SPI3_CS_DF_PIN // RESET (PB3)

//Micron M25P16 Serial Flash Embedded Memory 16 Mb, 3V
#define JEDEC_WRITE_ENABLE           0x06
#define JEDEC_WRITE_DISABLE          0x04
#define JEDEC_READ_STATUS            0x05
#define JEDEC_WRITE_STATUS           0x01
#define JEDEC_READ_DATA              0x03
#define JEDEC_FAST_READ              0x0b
#define JEDEC_DEVICE_ID              0x9F
#define JEDEC_PAGE_WRITE             0x02

#define JEDEC_STATUS_BUSY            0x01
#define JEDEC_STATUS_WRITEPROTECT    0x02
#define JEDEC_STATUS_BP0             0x04
#define JEDEC_STATUS_BP1             0x08
#define JEDEC_STATUS_BP2             0x10
#define JEDEC_STATUS_TP              0x20
#define JEDEC_STATUS_SEC             0x40
#define JEDEC_STATUS_SRP0            0x80

#define expect_memorytype            0x20
#define expect_capacity              0x15
#define sector_erase                 0xD8

uint8_t BlockBuffer[256];

/*
  try to take a semaphore safely from both in a timer and outside
 */
bool DataFlash_REVOMINI::_sem_take(uint8_t timeout)
{
    if (hal.scheduler->in_timerprocess()) {
        return _spi_sem->take_nonblocking();
    }
    return _spi_sem->take(timeout);
}

void DataFlash_REVOMINI::Init(const struct LogStructure *structure, uint8_t num_types)
{
    DataFlash_Class::Init(structure, num_types);
    // init to zero
    df_NumPages = 0;

    hal.gpio->pinMode(DF_RESET,HAL_GPIO_OUTPUT);
    // Reset the chip
    hal.gpio->write(DF_RESET,0);
    hal.scheduler->delay(1);
    hal.gpio->write(DF_RESET,1);

    _spi = hal.spi->device(AP_HAL::SPIDevice_Dataflash);
    if (_spi == NULL) {
        hal.scheduler->panic(
                PSTR("PANIC: DataFlash SPIDeviceDriver not found"));
        return; /* never reached */
    }

    _spi_sem = _spi->get_semaphore();
    if (_spi_sem == NULL) {
        hal.scheduler->panic(
                PSTR("PANIC: DataFlash SPIDeviceDriver semaphore is null"));
        return; /* never reached */
    }

    df_PageSize = PageSize();

    // the last page is reserved for config information
    df_NumPages = DF_LAST_PAGE - 1;
}

// This function is mainly to test the device
void DataFlash_REVOMINI::ReadManufacturerID()
{
    if (!_sem_take(5))
        return;
    // activate dataflash command decoder
    _spi->cs_assert();

    // Read manufacturer and ID command...
    _spi->transfer(JEDEC_DEVICE_ID); //

    df_manufacturer = _spi->transfer(0xff);
    df_device = _spi->transfer(0xff); //memorytype
    df_device = (df_device << 8) | _spi->transfer(0xff); //capacity
    _spi->transfer(0xff);

    // release SPI bus for use by other sensors
    _spi->cs_release();

    _spi_sem->give();
}

// This function return 1 if Card is inserted on SD slot
bool DataFlash_REVOMINI::CardInserted()
{
    return true;
}

// Read the status register
// Assumes _spi_sem handled by caller
uint8_t DataFlash_REVOMINI::ReadStatusReg()
{
    uint8_t tmp;

    // activate dataflash command decoder
    _spi->cs_assert();

    // Read status command
    _spi->transfer(JEDEC_READ_STATUS);
    tmp = _spi->transfer(0x00); // We only want to extract the READY/BUSY bit

    // release SPI bus for use by other sensors
    _spi->cs_release();

    return tmp;
}

// Read the status of the DataFlash
// Assumes _spi_sem handled by caller.
inline
uint8_t DataFlash_REVOMINI::ReadStatus()
{
  // We only want to extract the READY/BUSY bit
    int32_t status = ReadStatusReg();
    if (status < 0)
	    return -1;
    return status & JEDEC_STATUS_BUSY;
}

inline
uint16_t DataFlash_REVOMINI::PageSize()
{
    return 256;
}

// Wait until DataFlash is in ready state...
// Assumes _spi_sem handled by caller.
void DataFlash_REVOMINI::WaitReady()
{
    while(ReadStatus() != 0);
}

/**
 * @brief Execute the write enable instruction and returns the status
 * @returns 0 if successful, -1 if unable to claim bus
 */
void DataFlash_REVOMINI::Flash_Jedec_WriteEnable(void)
{
    // activate dataflash command decoder
    _spi->cs_assert();

    _spi->transfer(JEDEC_WRITE_ENABLE);

    _spi->cs_release();
}

void DataFlash_REVOMINI::BufferToPage (uint32_t IntPageAdr)
{
    if (!_sem_take(1))
        return;
    uint8_t *pData = BlockBuffer;

    uint8_t cmd[4];
    cmd[0] = JEDEC_PAGE_WRITE;
    cmd[1] = (IntPageAdr >> 16) & 0xff;
    cmd[2] = (IntPageAdr >>  8) & 0xff;
    cmd[3] = (IntPageAdr >>  0) & 0xff;

    Flash_Jedec_WriteEnable();

    _spi->cs_assert();

    _spi->transfer(cmd, sizeof(cmd));

    _spi->transfer((uint8_t *)pData, sizeof(BlockBuffer));

    // release SPI bus for use by other sensors
    _spi->cs_release();
    WaitReady();
    _spi_sem->give();
}

// Write block of data to temporary buffer
void DataFlash_REVOMINI::BlockWrite (uint32_t BufferIdx, const void *pHeader, uint8_t hdr_size, const void *pBuffer, uint16_t size)
{
    uint8_t *pData = BlockBuffer;

    pData += BufferIdx;
    if (hdr_size != 0) {
	memcpy( pData, (const uint8_t *)pHeader, hdr_size);
	pData += hdr_size;
    }
    memcpy( pData, (const uint8_t *)pBuffer, size);
}

bool DataFlash_REVOMINI::BlockRead (uint32_t IntPageAdr, void *pBuffer, uint16_t size)
{
    if (!_sem_take(1))
        return false;

    // activate dataflash command decoder
    _spi->cs_assert();

    uint8_t cmd[4];
    cmd[0] = JEDEC_READ_DATA;
    cmd[1] = (IntPageAdr >> 16) & 0xff;
    cmd[2] = (IntPageAdr >>  8) & 0xff;
    cmd[3] = (IntPageAdr >>  0) & 0xff;

    _spi->transfer(cmd, sizeof(cmd));

    uint8_t *pData = (uint8_t *)pBuffer;
    while (size--) {
        *pData++ = _spi->transfer(0x00);
    }
    // release SPI bus for use by other sensors
    _spi->cs_release();

    _spi_sem->give();
    return true;
}

/**
 * @brief Erase a sector on the flash chip
 * @param[in] chip_offset Sector number of flash to erase
 */

void DataFlash_REVOMINI::Flash_Jedec_EraseSector(uint32_t chip_offset)
{
    uint8_t cmd[4];
    cmd[0] = sector_erase;
    cmd[1] = (chip_offset >> 16) & 0xff;
    cmd[2] = (chip_offset >>  8) & 0xff;
    cmd[3] = (chip_offset >>  0) & 0xff;

    Flash_Jedec_WriteEnable();

    _spi->cs_assert();

    _spi->transfer(cmd, sizeof(cmd));

    _spi->cs_release();

}

// *** END OF INTERNAL FUNCTIONS ***

