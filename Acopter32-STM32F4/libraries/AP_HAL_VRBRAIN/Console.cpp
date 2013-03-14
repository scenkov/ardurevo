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

using namespace VRBRAIN;

VRBRAINConsoleDriver::VRBRAINConsoleDriver(AP_HAL::BetterStream* delegate) :
    _d(delegate)
{}

void VRBRAINConsoleDriver::init(void *args)
{
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
    _d->print_P(pstr);
}

void VRBRAINConsoleDriver::println_P(const prog_char_t *pstr)
{
  _d->println_P(pstr);
}

void VRBRAINConsoleDriver::printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

void VRBRAINConsoleDriver::_printf_P(const prog_char *fmt, ...) {
    va_list ap;
    va_start(ap,fmt);
    vprintf_P(fmt, ap);
    va_end(ap);
}

void VRBRAINConsoleDriver::vprintf(const char *fmt, va_list ap) {
    _d->vprintf(fmt, ap);
}

void VRBRAINConsoleDriver::vprintf_P(const prog_char *fmt, va_list ap) {
    _d->vprintf_P(fmt, ap);
}

int16_t VRBRAINConsoleDriver::available() {
    return _d->available();
}

int16_t VRBRAINConsoleDriver::txspace() {
    return _d->txspace();
}

int16_t VRBRAINConsoleDriver::read() {
    return _d->read();
}

size_t VRBRAINConsoleDriver::write(uint8_t c) {
    return _d->write(c);
}

