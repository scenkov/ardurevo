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
#include <stdio.h>              // for vsnprintf
#include "usb.h"
#include <usart.h>
#include <gpio.h>

//#define TX1 BOARD_USART1_TX_PIN
//#define RX1 BOARD_USART1_RX_PIN
//#define TX2 BOARD_USART2_TX_PIN
//#define RX2 BOARD_USART2_RX_PIN
//#define TX3 BOARD_USART3_TX_PIN
//#define RX3 BOARD_USART3_RX_PIN
//#define TX4 BOARD_UART4_TX_PIN
//#define RX4 BOARD_UART4_RX_PIN
//#define TX5 BOARD_UART5_TX_PIN
//#define RX5 BOARD_UART5_RX_PIN
static usb_attr_t usb_attr;
static uint8_t usb_connected;

using namespace VRBRAIN;

// XXX the AVR driver enables itself in the constructor.  This seems
// like a very bad idea, since it will run somewhere in the startup
// code before our clocks are all set up and such.
VRBRAINUARTDriver::VRBRAINUARTDriver(struct usart *dev)
  : m_dev(dev), m_initialized(false), m_blocking(true)
{
}

void VRBRAINUARTDriver::begin(uint32_t baud)
{
	if (usb == 0)
	begin(baud, DEFAULT_TX_TIMEOUT);
	else
{
	//begin(baud, DEFAULT_TX_TIMEOUT);

usb_open();

usb_default_attr(&usb_attr);
	usb_attr.preempt_prio = 1;
	usb_attr.sub_prio = 3;
	usb_attr.use_present_pin = 1;
	usb_attr.present_port = _GPIOD;
	usb_attr.present_pin = 4;

usb_ioctl(I_USB_SETATTR, &usb_attr);
usb_ioctl(I_USB_CONNECTED, &usb_connected);
usb_present = usb_connected;

}
}

// XXX buffer sizes ignored
void VRBRAINUARTDriver::begin(uint32_t baud, uint16_t rxS, uint16_t txS)
{
    assert_param((uint32)baud <= this->usart_device->max_baud);

        if ((uint32)baud > this->usart_device->max_baud) {
            return;
        }

        const stm32_pin_info *txi = &PIN_MAP[this->tx_pin];
        const stm32_pin_info *rxi = &PIN_MAP[this->rx_pin];


        uint8 mode;
        if (this->usart_device->USARTx == USART1) mode = GPIO_AF_USART1;
        else if (this->usart_device->USARTx == USART2) mode = GPIO_AF_USART2;
        else if (this->usart_device->USARTx == USART3) mode = GPIO_AF_USART3;
        else if (this->usart_device->USARTx == UART4) mode = GPIO_AF_UART4;
        else if (this->usart_device->USARTx == UART5) mode = GPIO_AF_UART5;
        else if (this->usart_device->USARTx == USART6) mode = GPIO_AF_USART6;
        else
        {
    		assert_param(0);
    		return;
        }

        gpio_set_af_mode(txi->gpio_device, txi->gpio_bit, mode);
        gpio_set_mode(txi->gpio_device, txi->gpio_bit, GPIO_AF_OUTPUT_PP);
        gpio_set_af_mode(rxi->gpio_device, rxi->gpio_bit, mode);
        gpio_set_mode(rxi->gpio_device, rxi->gpio_bit, GPIO_AF_OUTPUT_PP);

        usart_init(this->usart_device);
        usart_setup(this->usart_device, (uint32)baud, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, USART_Mode_Rx | USART_Mode_Tx, USART_HardwareFlowControl_None, tx_timeout);
        usart_enable(this->usart_device);

}

// XXX hwf4 doesn't support de-initializing a USART
void VRBRAINUARTDriver::end()
{
    if (usb == 0 )
        usart_disable(this->usart_device);
}

// XXX hwf4 doesn't support flushing, could be tricky to get the
// synchronization right.  Would we just force the TX/RX queues to
// empty?
void VRBRAINUARTDriver::flush()
{
    if (usb == 0)
    	{
        usart_reset_rx(this->usart_device);
        usart_reset_tx(this->usart_device);
    	}
    else
    {
        //rb_reset(rxfifo); // reset the rxfifo on usb.
    	usb_reset_rx();
        usb_reset_tx();
    }

}

 err --------------------- wip vrbrain ----------------------- eRR

bool VRBRAINUARTDriver::is_initialized()
{
  return m_initialized;
}

void VRBRAINUARTDriver::set_blocking_writes(bool blocking)
{
  m_blocking = blocking;
}

bool VRBRAINUARTDriver::tx_pending()
{
  return (0);usart_is_tx_pending(m_dev);
}

/* VRBRAIN implementations of BetterStream virtual methods */
void VRBRAINUARTDriver::print_P(const prog_char_t *pstr)
{
  while (*pstr)
    write(*pstr++);
}

void VRBRAINUARTDriver::println_P(const prog_char_t *pstr)
{
  print_P(pstr);
  println();
}

// XXX this will be changing, putting this on the stack hurts but
// allows us to be easily re-entrant
void VRBRAINUARTDriver::printf(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
}

void VRBRAINUARTDriver::_printf_P(const prog_char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
}

void VRBRAINUARTDriver::vprintf(const char *pstr, va_list ap)
{
  char buf[128];
  vsnprintf(buf, sizeof(buf), pstr, ap);
  print(buf);
}

void VRBRAINUARTDriver::vprintf_P(const prog_char *pstr, va_list ap)
{
  vprintf(pstr, ap);
}

/* VRBRAIN implementations of Stream virtual methods */
int16_t VRBRAINUARTDriver::available()
{
  return (int16_t)usart_available(m_dev);
}

int16_t VRBRAINUARTDriver::txspace()
{
  return (int16_t)usart_txspace(m_dev);
}

// It looks like this should always be a non-blocking read, so return
// -1 if there is nothing to receive immediately.
int16_t VRBRAINUARTDriver::read()
{
  uint8_t c;

  if (usart_read_timeout(m_dev, 0, &c, 1) == 0)
    return -1;

  return (int16_t)c;
}

int16_t VRBRAINUARTDriver::peek()
{
  uint8_t c;

  if (!usart_peek(m_dev, &c))
    return -1;

  return (int16_t)c;
}

/* VRBRAIN implementations of Print virtual methods */
size_t VRBRAINUARTDriver::write(uint8_t c)
{
  portTickType delay = m_blocking ? portMAX_DELAY : 0;
  return usart_write_timeout(m_dev, delay, &c, 1);
}
