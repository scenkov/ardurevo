/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef __DATAFLASH_MP32_H__
#define __DATAFLASH_MP32_H__

#include "DataFlash.h"

//#define DATAFLASH_MP32_DEBUG_ENABLE

class DataFlash_MP32 : public DataFlash_Class
{
  private:
	HardwareSPI * _SPIx;

	//Methods
	unsigned char BufferRead (unsigned char BufferNum, uint16_t IntPageAdr);
	void BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data);
	void BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait);
	void PageToBuffer(unsigned char BufferNum, uint16_t PageAdr);
	void WaitReady();
	unsigned char ReadStatusReg();
	unsigned char ReadStatus();
	uint16_t PageSize();
	void PageErase (uint16_t PageAdr);
	void BlockErase (uint16_t BlockAdr);
    void                    ChipErase(void (*delay_cb)(unsigned long));

  public:

	DataFlash_MP32(HardwareSPI *spi_dev, FastSerial *ser_port);
	void Init(void);
	void ReadManufacturerID();
	bool CardInserted(void);
};

#endif // __DATAFLASH_MP32_H__
