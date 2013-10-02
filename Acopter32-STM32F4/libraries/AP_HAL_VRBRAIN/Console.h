/*
 * Console.h --- AP_HAL_VRBRAIN console driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#ifndef __AP_HAL_VRBRAIN_CONSOLE_H__
#define __AP_HAL_VRBRAIN_CONSOLE_H__

#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_VRBRAIN_Namespace.h>

class VRBRAIN::VRBRAINConsoleDriver : public AP_HAL::ConsoleDriver {
public:
    VRBRAINConsoleDriver(void* baseuartdriver);
    void init(void *uart);
    void backend_open();
    void backend_close();
    size_t backend_read(uint8_t *data, size_t len);
    size_t backend_write(const uint8_t *data, size_t len);

    int16_t available();
    int16_t txspace();
    int16_t read();

    size_t write(uint8_t c);
private:
    AP_HAL::UARTDriver* _uart;
};

#endif // __AP_HAL_VRBRAIN_CONSOLE_H__
