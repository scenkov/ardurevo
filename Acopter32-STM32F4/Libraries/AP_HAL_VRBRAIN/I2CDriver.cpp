/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * I2CDriver.cpp --- AP_HAL_VRBRAIN I2C driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.R Written by Roberto Navoni  <info@virtualrobotix.com>, 11 January 2013
 */

#include <AP_HAL.h>
#include <hwf4/i2c.h>
#include <hwf4/gpio.h>

#include "I2CDriver.h"

using namespace VRBRAIN;

// For now, we are assuming all devices are on a single bus.  This
// will need to be refactored at some point to use a device manager
// like SPI if we need support for multiple busses.
#define I2C_BUS i2c2
#define I2C_SDA pin_b11
#define I2C_SCL pin_b10

void VRBRAINI2CDriver::begin()
{
  i2c_init(I2C_BUS, I2C_SDA, I2C_SCL);
  _semaphore.init();
}

// XXX hwf4 doesn't support de-initialization
void VRBRAINI2CDriver::end()
{
}

// XXX hwf4 doesn't support non-blocking I2C
void VRBRAINI2CDriver::setTimeout(uint16_t ms)
{
}

// XXX hwf4 always uses standard speed
void VRBRAINI2CDriver::setHighSpeed(bool active)
{
}

AP_HAL::Semaphore* VRBRAINI2CDriver::get_semaphore()
{
  return &_semaphore;
}

uint8_t VRBRAINI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
  return i2c_transfer(I2C_BUS, addr, data, len, NULL, 0) ? 0 : 1;
}

uint8_t VRBRAINI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
  return i2c_write_reg(I2C_BUS, addr, reg, val) ? 0 : 1;
}

uint8_t VRBRAINI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                                        uint8_t len, uint8_t* data)
{
  return i2c_write_regs(I2C_BUS, addr, reg, data, len) ? 0 : 1;
}

uint8_t VRBRAINI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
  return i2c_transfer(I2C_BUS, addr, NULL, 0, data, len) ? 0 : 1;
}

uint8_t VRBRAINI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
  return i2c_transfer(I2C_BUS, addr, &reg, 1, data, 1) ? 0 : 1;
}

uint8_t VRBRAINI2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                       uint8_t len, uint8_t* data)
{
  return i2c_transfer(I2C_BUS, addr, &reg, 1, data, len) ? 0 : 1;
}

uint8_t VRBRAINI2CDriver::lockup_count()
{
  return 0;
}
