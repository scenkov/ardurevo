
#include <string.h>
#include "Storage.h"
#include <AP_HAL.h>
#include <i2c.h>

extern const AP_HAL::HAL& hal;

using namespace VRBRAIN;
__IO uint32_t  timeout = I2C_TIMEOUT;

#define countof(a) (sizeof(a) / sizeof(*(a)))

void VRBRAINStorage::init(void*)
{
    Status = 0x0000;
}

uint8_t VRBRAINStorage::read_byte(uint16_t loc){

	uint32_t ret;
	uint8_t buf[1];
	uint16_t numbytes = 1;

	//buf = (uint8_t)read(addr16);
	ret = sEE_ReadBuffer(buf, loc, &numbytes);
	if(ret == 1){
	    hal.console->println_P("i2c timeout read byte");
	    return 0;
	}

	return buf[0];

}

uint16_t VRBRAINStorage::read_word(uint16_t loc){
    uint16_t val;
    read_block(&val, loc,sizeof(val));
    return val;
}

uint32_t VRBRAINStorage::read_dword(uint16_t loc){
	uint32_t val = 0;
	this->read_block(&val, loc, 4);
    return val;
}

void VRBRAINStorage::read_block(void* dst, uint16_t src, size_t n) {

	uint8_t * buff = (uint8_t*)dst;
	uint16_t numbytes = (uint16_t)n;

	uint32_t ret = sEE_ReadBuffer(buff, src, &numbytes);
	if(ret == 1){
	    hal.console->println_P("i2c timeout read block");
		return;
	}
	timeout = I2C_TIMEOUT;
	while(numbytes > 0)
	    {
	    if ((timeout--) == 0)
		{
		return;
		};
	    }

}

void VRBRAINStorage::write_block(uint16_t dst,const void* src, size_t n)
{
	uint8_t * buff = (uint8_t *)src;

	uint32_t ret = sEE_WriteBuffer(buff,dst,(uint16_t)n);
	if(ret == 1){
	    hal.console->println_P("i2c timeout write block");
	    return;
	}
	sEE_WaitEepromStandbyState();
}
void VRBRAINStorage::write_word(uint16_t loc, uint16_t value)
{
	write_block(loc,&value, sizeof(value));
	return;
}

void VRBRAINStorage::write_dword(uint16_t loc, uint32_t value)
{
	uint32_t val = value;
	this->write_block(loc,&val, 4);

}

void VRBRAINStorage::write_byte(uint16_t loc, uint8_t value)
{
	uint8_t numbytes = 1;
	uint8_t buff[1] = {value};

	uint32_t ret = sEE_WriteBuffer(buff, loc, numbytes);

	if(ret == 1){
	    hal.console->println_P("i2c timeout write byte");
	}
	sEE_WaitEepromStandbyState();
}



uint16_t VRBRAINStorage::format(void)
{
	for (unsigned int i = 0; i < EEPROM_PAGE_SIZE; i++)
	{
		write(EEPROM_START_ADDRESS + i, 0xFF);
	}

	return 0x0000;
}

uint16_t VRBRAINStorage::read(uint16_t Address)
{
	uint16_t data;
	read(Address, &data);
	return data;
}

uint16_t VRBRAINStorage::read(uint16_t Address, uint16_t *Data)
{
	uint8_t rdata[10];

	int8_t xret = 0; //hal.i2c->read((uint8_t)EEPROM_ADDRESS, (uint16_t)Address, (uint8_t)1, (uint8_t *)rdata);
	*Data = (uint16_t)rdata[0];

	if (xret != 0)
		return 0x0001;
	else
		return 0x0000;

}

uint16_t VRBRAINStorage::write(uint16_t Address, uint8_t Data)
{
	int8_t xret = 0;// hal.i2c->write((uint8_t)EEPROM_ADDRESS, (uint16_t)Address, (uint8_t)Data);
	hal.scheduler->delay(5);

	if (xret != 0)
		return 0x0001;
	else
		return 0x0000;
}

