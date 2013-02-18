
#include <string.h>
#include "Storage.h"
#include <AP_Hal.h>

extern const AP_HAL::HAL& hal;

using namespace VRBRAIN;


void VRBRAINStorage::init(void*)
{
    Status = 0x0000;
}

uint8_t VRBRAINStorage::read_byte(uint16_t loc){
	uint16_t addr16 = (uint16_t)(uint32_t)loc;
	return (uint8_t) read(addr16);
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
    //memset(dst, 0, n);
    //serPort->println("enter read block");
	uint8_t * buff = (uint8_t *)dst;
	uint16_t addr16 = src;
	for (uint16_t i = 0; i < (uint16_t)n; i++)
	{
		buff[i] = (uint8_t)read(addr16 + i);
		//serPort->printf("%u : %u\n", i, buff[i]);
	}
}

void VRBRAINStorage::write_byte(uint16_t loc, uint8_t value)
{
	uint16_t addr16 = (uint16_t)(uint32_t)loc;
	write(addr16, (uint16_t) value );
}

void VRBRAINStorage::write_word(uint16_t loc, uint16_t value)
{
	uint16_t val = value;
	write_block(&val, loc, sizeof(val));
}

void VRBRAINStorage::write_dword(uint16_t loc, uint32_t value)
{
	uint32_t val = value;
	write_block(&val, loc, sizeof(val));
}

void VRBRAINStorage::write_block(uint16_t loc, void* src, size_t n)
{
	uint8_t * buff = (uint8_t *)src;
	uint16_t addr16 = loc;

	for (uint16_t i = 0; i < (uint16_t)n; i++)
	{
		write(addr16 + i, (uint16_t) buff[i] );
	}
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

	int8_t xret = hal.i2c->read((uint8_t)EEPROM_ADDRESS, (uint16_t)Address, (uint8_t)1, (uint8_t *)rdata);
	*Data = (uint16_t)rdata[0];

	if (xret != 0)
		return 0x0001;
	else
		return 0x0000;

}

uint16_t VRBRAINStorage::write(uint16_t Address, uint16_t Data)
{
	int8_t xret = hal.i2c->write((uint8_t)EEPROM_ADDRESS, (uint16_t)Address, (uint8_t)Data);
	hal.scheduler->delay(5);

	if (xret != 0)
		return 0x0001;
	else
		return 0x0000;
}

