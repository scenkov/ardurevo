/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * I2CDriver.cpp --- AP_HAL_VRBRAIN I2C driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.R Written by Roberto Navoni  <info@virtualrobotix.com>, 11 January 2013
 */

#include <AP_HAL.h>
//#include <wirish.h>
#include "I2CDriver.h"
#include <i2c.h>

extern const AP_HAL::HAL& hal;

#define I2CDELAY 50

using namespace VRBRAIN;

void VRBRAINI2CDriver::begin() {
    i2c_init(this->_dev, 0, I2C_400KHz_SPEED);
    hal.scheduler->delay(I2CDELAY);
}
void VRBRAINI2CDriver::end() {}
void VRBRAINI2CDriver::setTimeout(uint16_t ms) {}
void VRBRAINI2CDriver::setHighSpeed(bool active) {}

uint8_t VRBRAINI2CDriver::write(uint8_t address, uint8_t len, uint8_t* tx_buffer)
{
	//if(i2c_is_busy())
//	    hal.console->printf_P(PSTR("I2C Busy write1\n"));

	uint32_t ret = i2c_write(this->_dev, address, tx_buffer, len);
	sEE_WaitEepromStandbyState(this->_dev, address);

	if(ret != 0)
	hal.console->printf_P(PSTR("Failed I2C write1: Event=0x%08X\n"),ret);

	return ret;
}

int8_t VRBRAINI2CDriver::write(uint8_t address, uint16_t registerAddress, uint8_t databyte)
{
	uint8_t ibuff[3];

	ibuff[0] = (uint8_t)(registerAddress >> 8);
	ibuff[1] = (uint8_t)(registerAddress & 0xFF);
	ibuff[2] = (uint8_t)databyte;

	//if(i2c_is_busy())
	//    hal.console->printf_P(PSTR("I2C Busy write\n"));

	uint8_t ret = i2c_write(this->_dev, address, ibuff, 3);
	sEE_WaitEepromStandbyState(this->_dev, address);

	if(ret != 0)
	hal.console->printf_P(PSTR("Failed I2C write: Event=0x%08X\n"),ret);


	return ret;
}

uint8_t VRBRAINI2CDriver::writeRegister(uint8_t address, uint8_t registerAddress, uint8_t databyte)
{
	//uint8_t ret = i2c_8bitaddr_write(this->i2c_d, address, registerAddress, databyte);
	//uint8_t ret = i2c_write(this->i2c_d, address, registerAddress, databyte);

	uint8_t ibuff[2];

	ibuff[0] = registerAddress;
	ibuff[1] = databyte;

	//if(i2c_is_busy())
	//    hal.console->printf_P(PSTR("I2C Busy writeRegister\n"));

	uint8_t ret = i2c_write(this->_dev, address, ibuff, 2);
	sEE_WaitEepromStandbyState(this->_dev, address);

	if(ret != 0)
	hal.console->printf_P(PSTR("Failed I2C writeRegister: Event=0x%08X\n"),ret);


	return ret;
}

uint8_t VRBRAINI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{return 0;}


uint8_t VRBRAINI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{

	//if(i2c_is_busy())
	//    hal.console->printf_P(PSTR("I2C Busy read1\n"));

	uint8_t ret = i2c_read(this->_dev, addr, NULL, 0, data, len);
	sEE_WaitEepromStandbyState(this->_dev, addr);
	if(ret != 0)
	hal.console->printf_P(PSTR("Failed I2C read1: Event=0x%08X\n"),ret);


	return ret;

}
int8_t VRBRAINI2CDriver::read(uint8_t address, uint16_t registerAddress, uint8_t numberBytes, uint8_t *dataBuffer)
{
	uint8_t ibuff[2];

	//preparing I2C buffer
	ibuff[0]=(uint8_t)(registerAddress >> 8);
	ibuff[1]=(uint8_t)(registerAddress & 0xFF);

	//if(i2c_is_busy())
	//    hal.console->printf_P(PSTR("I2C Busy read\n"));


	uint8_t ret = i2c_read(this->_dev, address, ibuff, 2, dataBuffer, numberBytes);
	//uint8_t ret = i2c_buffer_read(this->i2c_d, address, registerAddress, numberBytes, dataBuffer);
	sEE_WaitEepromStandbyState(this->_dev, address);
	if(ret != 0)
	hal.console->printf_P(PSTR("Failed I2C read: Event=0x%08X\n"),ret);


	return ret;
}
uint8_t VRBRAINI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
	uint8_t ibuff[1];

	ibuff[0] = reg;

	//if(i2c_is_busy())
	//    hal.console->printf_P(PSTR("I2C Busy readRegister\n"));


	uint8_t ret = i2c_read(this->_dev, addr, ibuff, 1, data, 1);
	sEE_WaitEepromStandbyState(this->_dev, addr);

	if(ret != 0)
	hal.console->printf_P(PSTR("Failed I2C readRegister: Event=0x%08X\n"),ret);


	return ret;
}
uint8_t VRBRAINI2CDriver::readRegisters(uint8_t addr, uint8_t reg, uint8_t numberBytes, uint8_t* data)
{
	uint8_t ibuff[1];

	ibuff[0] = reg;

	//if(i2c_is_busy())
	//    hal.console->printf_P(PSTR("I2C Busy readRegisters \n"));

	uint8_t ret = i2c_read(this->_dev, addr, ibuff, 1, data, numberBytes);
	//uint8_t ret = i2c_8bitaddr_buffer_read(this->i2c_d, address, registerAddress, numberBytes, dataBuffer);
	//uint8_t ret = i2c_buffer_read(this->i2c_d, address, registerAddress, numberBytes, dataBuffer);
	sEE_WaitEepromStandbyState(this->_dev, addr);

	if(ret != 0)
	hal.console->printf_P(PSTR("Failed I2C readRegisters: Event=0x%08X\n"),ret);


	return ret;
}

uint8_t VRBRAINI2CDriver::lockup_count() {return 0;}
