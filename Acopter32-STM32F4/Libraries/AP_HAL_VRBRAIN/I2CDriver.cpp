/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * I2CDriver.cpp --- AP_HAL_VRBRAIN I2C driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.R Written by Roberto Navoni  <info@virtualrobotix.com>, 11 January 2013
 */

#include <AP_HAL.h>
#include <wirish.h>
#include "I2CDriver.h"

using namespace VRBRAIN;

void VRBRAINI2CDriver::begin() {}
void VRBRAINI2CDriver::end() {}
void VRBRAINI2CDriver::setTimeout(uint16_t ms) {}
void VRBRAINI2CDriver::setHighSpeed(bool active) {}

uint8_t VRBRAINI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{return 0;} 
uint8_t VRBRAINI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{return 0;}
uint8_t VRBRAINI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{return 0;}

uint8_t VRBRAINI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{return 0;}
uint8_t VRBRAINI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{return 0;}
uint8_t VRBRAINI2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                              uint8_t len, uint8_t* data)
{return 0;}

uint8_t VRBRAINI2CDriver::lockup_count() {return 0;}
