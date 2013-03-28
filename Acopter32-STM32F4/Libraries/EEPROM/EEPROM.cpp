#include "EEPROM.h"
#include <HardwareI2C.h>
#include <i2c.h>

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

#define I2C_TIMEOUT         ((uint32_t)0x1000)
__IO uint32_t  timeout = I2C_TIMEOUT;

HardwareI2C *EEPROMClass::_I2Cx;

EEPROMClass::EEPROMClass()
{

}

EEPROMClass::EEPROMClass(HardwareI2C *i2c_d, FastSerial *ser_port)
{
	_I2Cx = i2c_d;
	serPort = ser_port;
	this->init(_I2Cx,serPort);
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
	_I2Cx->WaitEepromStandbyState(EEPROM_ADDRESS);

	if (xret != 0)
		return 0x0001;
	else
		return 0x0000;
}

#endif

//static functions - access to utilities to emulate EEPROM
void EEPROMClass::eeprom_read_block (void *pointer_ram, const void *pointer_eeprom, size_t n)
{
	uint8_t * buff = (uint8_t*)pointer_ram;
	uint16_t addr16 = (uint16_t)(uint32_t)pointer_eeprom;
	uint16_t numbytes = (uint16_t)n;

	uint32_t ret = sEE_ReadBuffer(buff, addr16, &numbytes);
	if(ret == 1){
	    serPort->println_P("i2c read block error");
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

void EEPROMClass::eeprom_write_block (const void *pointer_ram, void *pointer_eeprom, size_t n)
{


	uint8_t * buff = (uint8_t *)pointer_ram;
	uint16_t addr16 = (uint16_t)(uint32_t)pointer_eeprom;
	uint32_t ret = sEE_WriteBuffer(buff,addr16,(uint16_t)n);
	if(ret == 1){
	    serPort->println_P("i2c write block error");
	    return;
	}
	sEE_WaitEepromStandbyState();
}

uint8_t EEPROMClass::eeprom_read_byte (const uint8_t *addr)
{
	uint16_t addr16 = (uint16_t)(uint32_t)addr;

	uint32_t ret;
	uint8_t buf[1];
	uint16_t numbytes = 1;

	ret = sEE_ReadBuffer(buf, addr16, &numbytes);
	if(ret == 1){
	    serPort->println_P("i2c read byte error");
	    return 0;
	}
	return buf[0];

}

uint16_t EEPROMClass::eeprom_write_byte (uint8_t *addr, uint8_t value)
{
	uint16_t addr16 = (uint16_t)(uint32_t)addr;

	uint8_t numbytes = 1;
	uint8_t buff[1] = {value};

	uint32_t ret = sEE_WritePage(buff, addr16, &numbytes);
	if(ret == 1){
	    serPort->println_P("i2c timeout write byte");
	}
	timeout = I2C_TIMEOUT;

	sEE_WaitEepromStandbyState();
	return (uint16_t)ret;
}

uint16_t EEPROMClass::eeprom_read_word (const uint16_t *addr)
{
	uint16_t val = 0;
	eeprom_read_block(&val, addr, sizeof(val));
	return val;
}

void EEPROMClass::eeprom_write_word (uint16_t *addr, uint16_t value)
{
	uint16_t val = value;
	eeprom_write_block(&val, addr, sizeof(val));
}

uint32_t EEPROMClass::eeprom_read_dword (const uint32_t *addr)
{
	uint32_t val = 0;
	eeprom_read_block(&val, addr, sizeof(val));
	return val;
}

void EEPROMClass::eeprom_write_dword (uint32_t *addr, uint32_t value)
{
	uint32_t val = value;
	eeprom_write_block(&val, addr, sizeof(val));
}

