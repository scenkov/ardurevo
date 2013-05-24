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

class VRBRAIN::VRBRAINConsoleDriver : public AP_HAL::ConsoleDriver {
public:
    void init(void *baseuartdriver);
    void backend_open();
    void backend_close();
    size_t backend_read(uint8_t *data, size_t len);
    size_t backend_write(const uint8_t *data, size_t len);

    void print_P(const prog_char_t *pstr);
    void println_P(const prog_char_t *pstr);
    void printf(const char *pstr, ...)
            __attribute__ ((format(__printf__, 2, 3)));
    void _printf_P(const prog_char *pstr, ...)
            __attribute__ ((format(__printf__, 2, 3)));
			
    void vprintf(const char *pstr, va_list ap);
    void vprintf_P(const prog_char *pstr, va_list ap);

    int16_t available();
    int16_t txspace();
    int16_t read();

    size_t write(uint8_t c);
private:
    AP_HAL::UARTDriver *_d;
};

#endif // __AP_HAL_VRBRAIN_CONSOLE_H__
