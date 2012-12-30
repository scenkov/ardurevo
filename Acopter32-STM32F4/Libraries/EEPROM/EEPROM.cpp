#include "EEPROM.h"

static FastSerial *serPort;

#ifdef EEPROM_DEBUG_ENABLE
#pragma message "*** EEPROM Debug Enabled ***"
#define debug(fmt, args...) do { if (serPort != NULL) { serPort->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__ , ##args); delay(100); } } while(0)
#else
#define debug(fmt, args...)
#endif
#define notify(fmt, args...) do { if (serPort != NULL) { serPort->printf(fmt, ##args); delay(100); } } while(0)

#if EEPROM_TYPE_ENABLE == EEPROM_FLASH

#elif EEPROM_TYPE_ENABLE == EEPROM_I2C

EEPROMClass::EEPROMClass(void)
{
}

uint16_t EEPROMClass::init(HardwareI2C *i2c_d, FastSerial *ser_port)
{
	_I2Cx = i2c_d;
	serPort = ser_port;

	Status = 0x0000;
	return Status;
}


uint16_t EEPROMClass::format(void)
{
	for (unsigned int i = 0; i < EEPROM_PAGE_SIZE; i++) 
	{
		write(EEPROM_START_ADDRESS + i, 0xFF);
	}

	return 0x0000;
}

uint16_t EEPROMClass::read(uint16_t Address)
{
	uint16_t data;
	read(Address, &data);
	return data;
}

uint16_t EEPROMClass::read(uint16_t Address, uint16_t *Data)
{
	uint8_t rdata[10];

	int8_t xret = _I2Cx->read(EEPROM_ADDRESS, (uint16_t)Address, (uint8_t)1, (uint8_t *)rdata);
	*Data = (uint16_t)rdata[0];

	if (xret != 0)
		return 0x0001;
	else
		return 0x0000;

}

uint16_t EEPROMClass::write(uint16_t Address, uint16_t Data)
{
	int8_t xret = _I2Cx->write(EEPROM_ADDRESS, (uint16_t)Address, (uint8_t)Data);
	delay(5);

	if (xret != 0)
		return 0x0001;
	else
		return 0x0000;
}

EEPROMClass EEPROM;

#endif

//static functions - access to utilities to emulate EEPROM
void eeprom_read_block (void *pointer_ram, const void *pointer_eeprom, size_t n)
{
    //serPort->println("enter read block");
	uint8_t * buff = (uint8_t *)pointer_ram;
	uint16_t addr16 = (uint16_t)(uint32_t)pointer_eeprom;
	for (uint16_t i = 0; i < (uint16_t)n; i++) 
	{
		buff[i] = (uint8_t)EEPROM.read(addr16 + i);
		//serPort->printf("%u : %u\n", i, buff[i]);
	}

}

void eeprom_write_block (const void *pointer_ram, void *pointer_eeprom, size_t n)
{
#if EEPROM_TYPE_ENABLE == EEPROM_FLASH
#elif EEPROM_TYPE_ENABLE == EEPROM_I2C
	uint8_t * buff = (uint8_t *)pointer_ram;
	uint16_t addr16 = (uint16_t)(uint32_t)pointer_eeprom;

	for (uint16_t i = 0; i < (uint16_t)n; i++) 
	{
		EEPROM.write(addr16 + i, (uint16_t) buff[i] );
	}
#endif
}

uint8_t eeprom_read_byte (const uint8_t *addr)
{
	uint16_t addr16 = (uint16_t)(uint32_t)addr;
	return (uint8_t) EEPROM.read(addr16);
}

uint16_t eeprom_write_byte (uint8_t *addr, uint8_t value)
{
	uint16_t addr16 = (uint16_t)(uint32_t)addr;
	return EEPROM.write(addr16, (uint16_t) value );
}

uint16_t eeprom_read_word (const uint16_t *addr)
{
	uint16_t val = 0;
	eeprom_read_block(&val, addr, sizeof(val));
	return val;
}

void eeprom_write_word (uint16_t *addr, uint16_t value)
{
	uint16_t val = value;
	eeprom_write_block(&val, addr, sizeof(val));
}

uint32_t eeprom_read_dword (const uint32_t *addr)
{
	uint32_t val = 0;
	eeprom_read_block(&val, addr, sizeof(val));
	return val;
}

void eeprom_write_dword (uint32_t *addr, uint32_t value)
{
	uint32_t val = value;
	eeprom_write_block(&val, addr, sizeof(val));
}
