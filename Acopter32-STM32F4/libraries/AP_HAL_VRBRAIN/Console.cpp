/*
 * Console.cpp --- AP_HAL_VRBRAIN console driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#include <AP_HAL.h>
#include <stdarg.h>
#include "Console.h"


using namespace VRBRAIN;

VRBRAINConsoleDriver::VRBRAINConsoleDriver(void* baseuartdriver)
{}

void VRBRAINConsoleDriver::init(void *uart)
{
	_uart = (AP_HAL::UARTDriver *)uart;
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
size_t VRBRAINConsoleDriver::write(const uint8_t *buffer, size_t size) {
    return _uart->write(buffer, size);
}

