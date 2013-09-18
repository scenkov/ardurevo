/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * I2CDriver.cpp --- AP_HAL_VRBRAIN I2C driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.R Written by Roberto Navoni  <info@virtualrobotix.com>, 11 January 2013
 */

#include <AP_HAL.h>
#include "I2CDriver.h"
#include <i2c.h>

extern const AP_HAL::HAL& hal;

#define I2CDELAY 50

using namespace VRBRAIN;

__IO uint32_t  i2ctimeout = I2C_TIMEOUT;

void VRBRAINI2CDriver::begin() {
    i2c_init(this->_dev, 0, I2C_400KHz_SPEED);
    hal.scheduler->delay(I2CDELAY);

}
void VRBRAINI2CDriver::end() {}

void VRBRAINI2CDriver::setHighSpeed(bool active) {}

uint8_t VRBRAINI2CDriver::write(uint8_t address, uint8_t len, uint8_t* tx_buffer)
{

	uint8_t numbytes = len;

	uint32_t ret = i2c_write(this->_dev, address, tx_buffer, &numbytes);

	if(ret == 1){
	    _lockup_count ++;  //hal.console->printf_P(PSTR("Failed I2C write1: Event=0x%08X\n"),ret);
	    return 1;
	}

	return ret;
}


uint8_t VRBRAINI2CDriver::writeRegister(uint8_t address, uint8_t registerAddress, uint8_t databyte)
{
	uint8_t ibuff[2];

	ibuff[0] = registerAddress;
	ibuff[1] = databyte;
	uint8_t numbytes = 2;

	uint8_t ret = i2c_write(this->_dev, address, ibuff, &numbytes);

	if(ret == 1){
	     _lockup_count ++;
	}

	return ret;
}

uint8_t VRBRAINI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{return 0;}


uint8_t VRBRAINI2CDriver::read(uint8_t addr, uint8_t numberBytes, uint8_t* data)
{
	uint8_t ret = i2c_read(this->_dev, addr, NULL, 0, data, &numberBytes);

	if(ret == 1){
	    _lockup_count ++; //hal.console->printf_P(PSTR("Failed I2C read1: Event=0x%08X\n"),ret);
	    return ret;
	}
	return ret;
}

uint8_t VRBRAINI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
	uint8_t ibuff[1];

	ibuff[0] = reg;
	uint8_t numberBytes = 1;

	uint8_t ret = i2c_read(this->_dev, addr, ibuff, 1, data, &numberBytes);

	if(ret == 1){
	    _lockup_count ++; //hal.console->println_P("i2c timeout read register");
	    return 1;
	}

	return ret;
}
uint8_t VRBRAINI2CDriver::readRegisters(uint8_t addr, uint8_t reg, uint8_t numberBytes, uint8_t* data)
{
	uint8_t ibuff[1];

	ibuff[0] = reg;
	uint8_t numbytes = numberBytes;

	uint8_t ret = i2c_read(this->_dev, addr, ibuff, 1, data, &numbytes);

	if(ret == 1){
	    _lockup_count ++;
	    return 1;
	}

	uint32_t time = i2ctimeout;
	while(numbytes > 0){
	    if ((time--) == 0)
		{
		_lockup_count ++;
		return 1;
		}
	}

	return ret;
}

