/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
	DataFlash_MP32.cpp - DataFlash log library for AT45DB161
	Code by Jordi Munoz and Jose Julio. DIYDrones.com
	This code works with boards based on ATMega168/328 and ATMega1280/2560 using SPI port

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Dataflash library for AT45DB161D flash memory
	Memory organization : 4096 pages of 512 bytes or 528 bytes

	Maximun write bandwidth : 512 bytes in 14ms
	This code is written so the master never has to wait to write the data on the eeprom

	Methods:
		Init() : Library initialization (SPI initialization)
		StartWrite(page) : Start a write session. page=start page.
		WriteByte(data) : Write a byte
		WriteInt(data) :  Write an integer (2 bytes)
		WriteLong(data) : Write a long (4 bytes)
		StartRead(page) : Start a read on (page)
		GetWritePage() : Returns the last page written to
		GetPage() : Returns the last page read
		ReadByte()
		ReadInt()
		ReadLong()

	Properties:

*/

#include <stdint.h>
#include "DataFlash.h"

static FastSerial *serPort;

// flash size
#define DF_LAST_PAGE 4096

#ifdef DATAFLASH_MP32_DEBUG_ENABLE
#pragma message "*** DATAFLASH_MP32 Debug Enabled ***"
#define debug(fmt, args...) do { if (serPort != NULL) { serPort->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__ , ##args); delay(100); } } while(0)
#else
#define debug(fmt, args...)
#endif
#define notify(fmt, args...) do { if (serPort != NULL) { serPort->printf(fmt, ##args); delay(100); } } while(0)


// AT45DB161D Commands (from Datasheet)
#define DF_TRANSFER_PAGE_TO_BUFFER_1   0x53
#define DF_TRANSFER_PAGE_TO_BUFFER_2   0x55
#define DF_STATUS_REGISTER_READ   0xD7
#define DF_READ_MANUFACTURER_AND_DEVICE_ID   0x9F
#define DF_PAGE_READ   0xD2
#define DF_BUFFER_1_READ   0xD4
#define DF_BUFFER_2_READ   0xD6
#define DF_BUFFER_1_WRITE   0x84
#define DF_BUFFER_2_WRITE   0x87
#define DF_BUFFER_1_TO_PAGE_WITH_ERASE   0x83
#define DF_BUFFER_2_TO_PAGE_WITH_ERASE   0x86
#define DF_PAGE_ERASE   0x81
#define DF_BLOCK_ERASE   0x50
#define DF_SECTOR_ERASE   0x7C
#define DF_CHIP_ERASE_0   0xC7
#define DF_CHIP_ERASE_1   0x94
#define DF_CHIP_ERASE_2   0x80
#define DF_CHIP_ERASE_3   0x9A

DataFlash_MP32::DataFlash_MP32(HardwareSPI *spi_dev, FastSerial *ser_port) :
	DataFlash_Class(),
	_SPIx(spi_dev)
{
	serPort = ser_port;
}

// *** INTERNAL FUNCTIONS ***

void dataflash_CS_inactive()
{
  digitalWrite(BOARD_SPI1_CS_DF_PIN,HIGH); //disable device
}

void dataflash_CS_active()
{
  //delay_us(1);
  digitalWrite(BOARD_SPI1_CS_DF_PIN,LOW); //enable device
  }

// Public Methods //////////////////////////////////////////////////////////////
void DataFlash_MP32::Init(void)
{
  pinMode(BOARD_SPI1_CS_DF_PIN,OUTPUT);
  pinMode(BOARD_SPI1_CS_BR_PIN,OUTPUT);
  digitalWrite(BOARD_SPI1_CS_BR_PIN,HIGH); //disable barometer

  dataflash_CS_inactive();     //disable device

  // get page size: 512 or 528
  df_PageSize=PageSize();

  // the last page is reserved for config information
  df_NumPages = DF_LAST_PAGE - 1;
}

// This function is mainly to test the device
void DataFlash_MP32::ReadManufacturerID()
{
  dataflash_CS_active();     // activate dataflash command decoder

  // Read manufacturer and ID command...
  _SPIx->transfer(DF_READ_MANUFACTURER_AND_DEVICE_ID);

  df_manufacturer = _SPIx->transfer(0xff);
  df_device = _SPIx->transfer(0xff);
  df_device = (df_device<<8) | _SPIx->transfer(0xff);
  _SPIx->transfer(0xff);

  dataflash_CS_inactive();    // Reset dataflash command decoder
}


bool DataFlash_MP32::CardInserted(void)
{
    return true;
}

// Read the status register
byte DataFlash_MP32::ReadStatusReg()
{
  byte tmp;

  dataflash_CS_active();     // activate dataflash command decoder

  // Read status command
  _SPIx->transfer(DF_STATUS_REGISTER_READ);
  tmp = _SPIx->transfer(0x00);  // We only want to extract the READY/BUSY bit

  dataflash_CS_inactive();    // Reset dataflash command decoder

  return tmp;
}

// Read the status of the DataFlash
inline
byte DataFlash_MP32::ReadStatus()
{
  return(ReadStatusReg()&0x80);  // We only want to extract the READY/BUSY bit
}


inline
uint16_t DataFlash_MP32::PageSize()
{
  return(528-((ReadStatusReg()&0x01)<<4));  // if first bit 1 trhen 512 else 528 bytes
}


// Wait until DataFlash is in ready state...
void DataFlash_MP32::WaitReady()
{
  while(!ReadStatus());
}

void DataFlash_MP32::PageToBuffer(unsigned char BufferNum, uint16_t PageAdr)
{
  dataflash_CS_active();     // activate dataflash command decoder

  if (BufferNum==1)
    _SPIx->transfer(DF_TRANSFER_PAGE_TO_BUFFER_1);
  else
    _SPIx->transfer(DF_TRANSFER_PAGE_TO_BUFFER_2);

  if(df_PageSize==512){
    _SPIx->transfer((unsigned char)(PageAdr >> 7));
    _SPIx->transfer((unsigned char)(PageAdr << 1));
  }else{
    _SPIx->transfer((unsigned char)(PageAdr >> 6));
    _SPIx->transfer((unsigned char)(PageAdr << 2));
  }
  _SPIx->transfer(0x00);	// don´t care bytes

  dataflash_CS_inactive();	//initiate the transfer
  dataflash_CS_active();

  while(!ReadStatus());  //monitor the status register, wait until busy-flag is high

  dataflash_CS_inactive();

}

void DataFlash_MP32::BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait)
{
  dataflash_CS_active();     // activate dataflash command decoder

  if (BufferNum==1)
    _SPIx->transfer(DF_BUFFER_1_TO_PAGE_WITH_ERASE);
  else
    _SPIx->transfer(DF_BUFFER_2_TO_PAGE_WITH_ERASE);

  if(df_PageSize==512){
    _SPIx->transfer((unsigned char)(PageAdr >> 7));
    _SPIx->transfer((unsigned char)(PageAdr << 1));
  }else{
    _SPIx->transfer((unsigned char)(PageAdr >> 6));
    _SPIx->transfer((unsigned char)(PageAdr << 2));
  }
  _SPIx->transfer(0x00);	// don´t care bytes

  dataflash_CS_inactive();	//initiate the transfer
  dataflash_CS_active();

  // Check if we need to wait to write the buffer to memory or we can continue...
  if (wait)
	while(!ReadStatus());  //monitor the status register, wait until busy-flag is high

  dataflash_CS_inactive();	//deactivate dataflash command decoder
}

void DataFlash_MP32::BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data)
{
  dataflash_CS_active();     // activate dataflash command decoder

  if (BufferNum==1)
    _SPIx->transfer(DF_BUFFER_1_WRITE);
  else
    _SPIx->transfer(DF_BUFFER_2_WRITE);
  _SPIx->transfer(0x00);				 //don't cares
  _SPIx->transfer((unsigned char)(IntPageAdr>>8));  //upper part of internal buffer address
  _SPIx->transfer((unsigned char)(IntPageAdr));	 //lower part of internal buffer address
  _SPIx->transfer(Data);				 //write data byte

  dataflash_CS_inactive();   // disable dataflash command decoder
}

unsigned char DataFlash_MP32::BufferRead (unsigned char BufferNum, uint16_t IntPageAdr)
{
  byte tmp;

  dataflash_CS_active();     // activate dataflash command decoder

  if (BufferNum==1)
    _SPIx->transfer(DF_BUFFER_1_READ);
  else
    _SPIx->transfer(DF_BUFFER_2_READ);
  _SPIx->transfer(0x00);				 //don't cares
  _SPIx->transfer((unsigned char)(IntPageAdr>>8));  //upper part of internal buffer address
  _SPIx->transfer((unsigned char)(IntPageAdr));	 //lower part of internal buffer address
  _SPIx->transfer(0x00);                            //don't cares
  tmp = _SPIx->transfer(0x00);		         //read data byte

  dataflash_CS_inactive();   // deactivate dataflash command decoder

  return (tmp);
}
// *** END OF INTERNAL FUNCTIONS ***

void DataFlash_MP32::PageErase (uint16_t PageAdr)
{
  dataflash_CS_active();     // activate dataflash command decoder
  _SPIx->transfer(DF_PAGE_ERASE);   // Command

  if(df_PageSize==512){
    _SPIx->transfer((unsigned char)(PageAdr >> 7));
    _SPIx->transfer((unsigned char)(PageAdr << 1));
  }else{
    _SPIx->transfer((unsigned char)(PageAdr >> 6));
    _SPIx->transfer((unsigned char)(PageAdr << 2));
  }

  _SPIx->transfer(0x00);	           // "dont cares"
  dataflash_CS_inactive();               //initiate flash page erase
  dataflash_CS_active();
  while(!ReadStatus());

  dataflash_CS_inactive();   // deactivate dataflash command decoder
}

void DataFlash_MP32::BlockErase (uint16_t BlockAdr)
{
  dataflash_CS_active();     // activate dataflash command decoder
  _SPIx->transfer(DF_BLOCK_ERASE);   // Command

  if (df_PageSize==512) {
      _SPIx->transfer((unsigned char)(BlockAdr >> 3));
      _SPIx->transfer((unsigned char)(BlockAdr << 5));
  } else {
      _SPIx->transfer((unsigned char)(BlockAdr >> 4));
      _SPIx->transfer((unsigned char)(BlockAdr << 4));
  }

  _SPIx->transfer(0x00);	           // "dont cares"
  dataflash_CS_inactive();               //initiate flash page erase
  dataflash_CS_active();
  while(!ReadStatus());

  dataflash_CS_inactive();   // deactivate dataflash command decoder
}



void DataFlash_MP32::ChipErase (void (*delay_cb)(unsigned long))
{

  dataflash_CS_active();     // activate dataflash command decoder
  // opcodes for chip erase
  _SPIx->transfer(DF_CHIP_ERASE_0);
  _SPIx->transfer(DF_CHIP_ERASE_1);
  _SPIx->transfer(DF_CHIP_ERASE_2);
  _SPIx->transfer(DF_CHIP_ERASE_3);

  dataflash_CS_inactive();               //initiate flash page erase
  dataflash_CS_active();
    while (!ReadStatus()) {
        delay_cb(1);
    }

  dataflash_CS_inactive();   // deactivate dataflash command decoder
}

