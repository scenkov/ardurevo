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
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>

#include <usart.h>
#include <usb.h>
#include <gpio_hal.h>

using namespace VRBRAIN;

//definisco qui i parametri per le varie seriali preconfigurate

VRBRAINUARTDriver::VRBRAINUARTDriver(struct usart_dev *usart):
    usart_device(usart)
{
    this->tx_pin = usart_device->tx_pin;
    this->rx_pin = usart_device->rx_pin;
    this->_initialized = true;
    begin(57600);
}

extern const AP_HAL::HAL& hal;

void VRBRAINUARTDriver::begin(uint32_t baud) {


    const stm32_pin_info *txi = &PIN_MAP[this->tx_pin];
    const stm32_pin_info *rxi = &PIN_MAP[this->rx_pin];

    gpio_set_af_mode(txi->gpio_device, txi->gpio_bit, this->usart_device->gpio_af);
    gpio_set_mode(txi->gpio_device, txi->gpio_bit, GPIO_AF_OUTPUT_PP);
    gpio_set_af_mode(rxi->gpio_device, rxi->gpio_bit, this->usart_device->gpio_af);
    gpio_set_mode(rxi->gpio_device, rxi->gpio_bit, GPIO_AF_OUTPUT_PP);

    usart_init(this->usart_device);
    usart_setup(this->usart_device, (uint32)baud, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, USART_Mode_Rx | USART_Mode_Tx, USART_HardwareFlowControl_None, DEFAULT_TX_TIMEOUT);
    usart_enable(this->usart_device);
}

void VRBRAINUARTDriver::begin(uint32_t baud, uint16_t rxS, uint16_t txS) {
    begin(baud);
}

void VRBRAINUARTDriver::end() {
    usart_disable(this->usart_device);
}

void VRBRAINUARTDriver::flush() {
    usart_reset_rx(this->usart_device);
    usart_reset_tx(this->usart_device);
}

void VRBRAINUARTDriver::set_blocking_writes(bool blocking) {
    usart_reset_tx(this->usart_device);
    this->usart_device->usetxrb = !blocking;
}

bool VRBRAINUARTDriver::tx_pending() {
    return (usart_txfifo_nbytes(this->usart_device) > 0 ? true : false);
}

/* VRBRAIN implementations of BetterStream virtual methods */
void VRBRAINUARTDriver::print_P(const prog_char_t *pstr) {

    print(pstr);

}
void VRBRAINUARTDriver::println_P(const prog_char_t *pstr) {

    println(pstr);

}
void VRBRAINUARTDriver::printf(const char *fmt, ...) {

    va_list ap;
    va_start(ap, fmt);
    _vprintf(fmt, ap);
    va_end(ap);

}
void VRBRAINUARTDriver::_printf_P(const prog_char *fmt, ...) {

    va_list ap;
    va_start(ap, fmt);
    _vprintf(fmt, ap);
    va_end(ap);

}

void VRBRAINUARTDriver::vprintf(const char *pstr, va_list ap) {

    _vprintf(pstr, ap);

}
void VRBRAINUARTDriver::vprintf_P(const prog_char *fmt, va_list ap) {

    _vprintf(fmt, ap);

}

void VRBRAINUARTDriver::_internal_vprintf(const char *fmt, va_list ap)
{
    //if (hal.scheduler->in_timerprocess()) {
        // not allowed from timers
      //  return;
    //}

    unsigned char c;
    char buf[1024];  //destination buffer
    int i = 0;

    vsprintf(buf, fmt, ap);

    //per sicurezza....
    buf[1023] = 0;

    c = buf[i];
    while (c != 0)
    {

    	if (c == '\n')
    	    write('\r');
    	write(c);
    	i++;
    	c = buf[i];
    }

    /*
    char *buf = NULL;
    int n = avsprintf(&buf, fmt, ap);
    if (n > 0) {
        write((const uint8_t *)buf, n);
    }
    if (buf != NULL) {
        free(buf);    
    }
    */
}

// handle %S -> %s
void VRBRAINUARTDriver::_vprintf(const char *fmt, va_list ap)
{
    //if (hal.scheduler->in_timerprocess()) {
        // not allowed from timers
    //    return;
    //}
    // we don't use vdprintf() as it goes directly to the file descriptor
	if (strstr(fmt, "%S")) {
		char *fmt2 = strdup(fmt);
		if (fmt2 != NULL) {
			for (uint16_t i=0; fmt2[i]; i++) {
				if (fmt2[i] == '%' && fmt2[i+1] == 'S') {
					fmt2[i+1] = 's';
				}
			}
            _internal_vprintf(fmt2, ap);
			free(fmt2);
		}
	} else {
        _internal_vprintf(fmt, ap);
	}
}


/* VRBRAIN implementations of Stream virtual methods */
int16_t VRBRAINUARTDriver::available() {
    return usart_data_available(this->usart_device);
}

int16_t VRBRAINUARTDriver::txspace() {
    return usart_txfifo_freebytes(this->usart_device);
}

int16_t VRBRAINUARTDriver::read() {
	if (available() <= 0)
	    return (-1);
	return usart_getc(this->usart_device);
}

/* VRBRAIN implementations of Print virtual methods */
size_t VRBRAINUARTDriver::write(uint8_t c) {
    usart_putc(this->usart_device, c);
    return 1;
}
