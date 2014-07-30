
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
}

uint8_t VRBRAINStorage::read_byte(uint16_t loc){

	uint32_t ret;
	uint8_t buf[1];
	uint16_t numbytes = 1;

	//sEE_WaitEepromStandbyState();

	//buf = (uint8_t)read(addr16);
	ret = sEE_ReadBuffer(_dev, buf, loc, &numbytes);
	if(ret == 1){
	    //hal.console->println_P("i2c timeout read byte");
	    return 0;
	}
	//while(numbytes > 0);

	return buf[0];

}

uint16_t VRBRAINStorage::read_word(uint16_t loc){
    uint16_t val = 0;
    read_block(&val, loc, sizeof(val));
    return val;
}

uint32_t VRBRAINStorage::read_dword(uint16_t loc){
    uint32_t val = 0;
    read_block(&val, loc, sizeof(val));
    return val;
}

void VRBRAINStorage::read_block(void* dst, uint16_t src, size_t n) {

	uint8_t * buff = (uint8_t*)dst;
	uint16_t numbytes = (uint16_t)n;

	//sEE_WaitEepromStandbyState();

	uint32_t ret = sEE_ReadBuffer(_dev, buff, src, &numbytes);

	if(ret == 1){
	    hal.gpio->write(20, 1);
	    return;
	}

	uint32_t time =  ((uint32_t)(10 * 0x1000));
	while(numbytes > 0){
	    if ((time--) == 0)
		{
		hal.gpio->write(20, 1);
		return;
		}
	}

}

void VRBRAINStorage::write_block(uint16_t dst,const void* src, size_t n)
{
	uint8_t * buff = (uint8_t *)src;

	//sEE_WaitEepromStandbyState();

	uint32_t ret = sEE_WriteBuffer(_dev, buff,dst,(uint16_t)n);
	if(ret == 1){
	    //hal.console->println_P("i2c timeout write block");
	    return;
	}
}
void VRBRAINStorage::write_word(uint16_t loc, uint16_t value)
{
	uint16_t val = value;
	this->write_block(loc,&val, sizeof(val));
}

void VRBRAINStorage::write_dword(uint16_t loc, uint32_t value)
{
	uint32_t val = value;
	this->write_block(loc,&val, sizeof(val));

}

void VRBRAINStorage::write_byte(uint16_t loc, uint8_t value)
{
	uint8_t numbytes = 1;
	uint8_t buff[1];

	//sEE_WaitEepromStandbyState();

	buff[0]= read_byte(loc);

	if(buff[0] != value){
	    buff[0] = value;
	    uint32_t ret = sEE_WriteBuffer(_dev, buff, loc, numbytes);
	    if(ret == 1){
		//hal.console->println_P("i2c timeout write byte");
	    }
	}
}


