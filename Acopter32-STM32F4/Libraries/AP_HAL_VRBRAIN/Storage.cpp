/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * Storage.cpp --- AP_HAL_VRBRAIN storage driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by Roberto Navoni  <info@virtualrobotix.com>, 11 January 2013
 */

#include <string.h>
//#include <hwf4/eeprom.h>

#include "Storage.h"

using namespace VRBRAIN;

//#define EEPROM_I2C_ADDR 0x50

// Note: These functions write multi-byte integers to the EEPROM in
// the native byte order, and so the format will depend on the
// endianness of the machine.

VRBRAINStorage::VRBRAINStorage()
{
}

void VRBRAINStorage::init(void*)
{
  //eeprom_init(i2c2, EEPROM_I2C_ADDR);
}

uint8_t VRBRAINStorage::read_byte(uint16_t loc)
{
  uint8_t result = 0;
  //eeprom_read_byte(loc, &result);
  return result;
}

uint16_t VRBRAINStorage::read_word(uint16_t loc)
{
  uint16_t result = 0;
  //eeprom_read(loc, (uint8_t*)&result, sizeof(result));
  return result;
}

uint32_t VRBRAINStorage::read_dword(uint16_t loc)
{
  uint32_t result = 0;
  //eeprom_read(loc, (uint8_t*)&result, sizeof(result));
  return result;
}

void VRBRAINStorage::read_block(void* dst, uint16_t src, size_t n)
{
  //eeprom_read(src, (uint8_t*)dst, n);
}

void VRBRAINStorage::write_byte(uint16_t loc, uint8_t value)
{
  //eeprom_write_byte(loc, value);
}

void VRBRAINStorage::write_word(uint16_t loc, uint16_t value)
{
  //eeprom_write(loc, (uint8_t*)&value, sizeof(value));
}

void VRBRAINStorage::write_dword(uint16_t loc, uint32_t value)
{
  //eeprom_write(loc, (uint8_t*)&value, sizeof(value));
}

void VRBRAINStorage::write_block(uint16_t loc, void* src, size_t n)
{
  //eeprom_write(loc, (const uint8_t *)src, n);
}
