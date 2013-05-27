/*
 * Console.cpp --- AP_HAL_VRBRAIN console driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#include <stdarg.h>
#include "Console.h"
#include "UARTDriver.h"
#include <stdio.h>

using namespace VRBRAIN;

VRBRAINConsoleDriver::VRBRAINConsoleDriver(){}

void VRBRAINConsoleDriver::init(void *uart)
{
	_uart = (VRBRAINUARTDriver *)uart;
}

void VRBRAINConsoleDriver::backend_open()
{}

void VRBRAINConsoleDriver::backend_close()
{}

size_t VRBRAINConsoleDriver::backend_read(uint8_t *data, size_t len) {
    return 0;
}

size_t VRBRAINConsoleDriver::backend_write(const uint8_t *data, size_t len) {
    return 0;
}

void VRBRAINConsoleDriver::print_P(const prog_char_t *pstr) {
    _uart->print_P(pstr);
}

void VRBRAINConsoleDriver::println_P(const prog_char_t *pstr)
{
  _uart->println_P(pstr);
}

void VRBRAINConsoleDriver::printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    _uart->vprintf(fmt, ap);
    va_end(ap);
}

void VRBRAINConsoleDriver::_printf_P(const prog_char *fmt, ...) {
    va_list ap;
    va_start(ap,fmt);
    _uart->vprintf(fmt, ap);
    va_end(ap);
}

void VRBRAINConsoleDriver::vprintf(const char *fmt, va_list ap) {
    _uart->vprintf(fmt, ap);
}

void VRBRAINConsoleDriver::vprintf_P(const prog_char *fmt, va_list ap) {
    _uart->vprintf(fmt, ap);
}

int16_t VRBRAINConsoleDriver::available() {
    return _uart->available();
}

int16_t VRBRAINConsoleDriver::txspace() {
    return _uart->txspace();
}

int16_t VRBRAINConsoleDriver::read() {
    return _uart->read();
}

size_t VRBRAINConsoleDriver::write(uint8_t c) {
    return _uart->write(c);
}

