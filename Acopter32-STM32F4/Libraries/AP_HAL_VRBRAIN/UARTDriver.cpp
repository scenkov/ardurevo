/*
 * UARTDriver.cpp --- AP_HAL_VRBRAIN UART driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#include "UARTDriver.h"

using namespace VRBRAIN;

VRBRAINUARTDriver::VRBRAINUARTDriver() {}

void VRBRAINUARTDriver::begin(uint32_t b) {}
void VRBRAINUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void VRBRAINUARTDriver::end() {}
void VRBRAINUARTDriver::flush() {}
bool VRBRAINUARTDriver::is_initialized() { return false; }
void VRBRAINUARTDriver::set_blocking_writes(bool blocking) {}
bool VRBRAINUARTDriver::tx_pending() { return false; }

/* VRBRAIN implementations of BetterStream virtual methods */
void VRBRAINUARTDriver::print_P(const prog_char_t *pstr) {}
void VRBRAINUARTDriver::println_P(const prog_char_t *pstr) {}
void VRBRAINUARTDriver::printf(const char *pstr, ...) {}
void VRBRAINUARTDriver::_printf_P(const prog_char *pstr, ...) {}
void VRBRAINUARTDriver::vprintf(const char *pstr, va_list ap) {}
void VRBRAINUARTDriver::vprintf_P(const prog_char *pstr, va_list ap) {}

/* VRBRAIN implementations of Stream virtual methods */
int16_t VRBRAINUARTDriver::available() { return 0; }
int16_t VRBRAINUARTDriver::txspace() { return 1; }
int16_t VRBRAINUARTDriver::read() { return -1; }
int16_t VRBRAINUARTDriver::peek() { return -1; }

/* VRBRAIN implementations of Print virtual methods */
size_t VRBRAINUARTDriver::write(uint8_t c) { return 0; }
