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
#include <usart.h>
#include <usb.h>
#include <gpio.h>

using namespace VRBRAIN;

//definisco qui i parametri per le varie seriali preconfigurate

VRBRAINUARTDriver::VRBRAINUARTDriver(struct usart_dev *usart):
    usart_device(usart)
{
    this->tx_pin = usart_device->tx_pin;
    this->rx_pin = usart_device->rx_pin;
    this->_initialized = 1;
    begin(57600);
}

void VRBRAINUARTDriver::begin(uint32_t baud) {


    const stm32_pin_info *txi = &PIN_MAP[this->tx_pin];
    const stm32_pin_info *rxi = &PIN_MAP[this->rx_pin];

    gpio_set_af_mode(txi->gpio_device, txi->gpio_bit, usart_device->gpio_af);
    gpio_set_mode(txi->gpio_device, txi->gpio_bit, GPIO_AF_OUTPUT_PP);
    gpio_set_af_mode(rxi->gpio_device, rxi->gpio_bit, usart_device->gpio_af);
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
