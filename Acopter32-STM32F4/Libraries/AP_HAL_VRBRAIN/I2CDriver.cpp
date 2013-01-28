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

using namespace VRBRAIN;

extern const AP_HAL::HAL& hal;

VRBRAINI2CDriver::VRBRAINI2CDriver(i2c_dev *dev_num):
	dev(dev_num),
	_semaphore(0)
{
    begin();
}

void VRBRAINI2CDriver::begin() {
	i2c_init(this->dev, 0, I2C_400KHz_SPEED);
	hal.scheduler->delay(50);
}
void VRBRAINI2CDriver::end() {}
void VRBRAINI2CDriver::setTimeout(uint16_t ms) {}
void VRBRAINI2CDriver::setHighSpeed(bool active) {}

uint8_t VRBRAINI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
    uint8_t ret = i2c_write(this->dev, addr, data, len);
    return ret;
}

uint8_t VRBRAINI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t ibuff[2];

    ibuff[0] = reg;
    ibuff[1] = val;

    uint8_t ret = i2c_write(this->dev, addr, ibuff, 2);
    return ret;
}

uint8_t VRBRAINI2CDriver::writeRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data)
{
    return 0;
}

uint8_t VRBRAINI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{

	uint8_t ret = i2c_read(this->dev, addr, NULL, 0, data, len);
	return ret;

}
uint8_t VRBRAINI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
	uint8_t ibuff[1];

	ibuff[0] = (uint8_t)reg;

	uint8_t ret = i2c_read(this->dev, addr, ibuff, 1, data, 1);
	return ret;
}
uint8_t VRBRAINI2CDriver::readRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data)
{
	uint8_t ibuff[1];

	ibuff[0] = (uint8_t)reg;

	uint8_t ret = i2c_read(this->dev, addr, ibuff, 1, data, len);
	return ret;
}

uint8_t VRBRAINI2CDriver::lockup_count() {return 0;}
