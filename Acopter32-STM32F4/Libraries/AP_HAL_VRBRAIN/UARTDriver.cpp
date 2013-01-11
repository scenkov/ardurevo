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
  usart_init(m_dev, baud);
  usart_enable(m_dev);
  m_initialized = true;
}

// XXX buffer sizes ignored
void VRBRAINUARTDriver::begin(uint32_t baud, uint16_t rxS, uint16_t txS)
{
  begin(baud);
}

// XXX hwf4 doesn't support de-initializing a USART
void VRBRAINUARTDriver::end()
{
}

// XXX hwf4 doesn't support flushing, could be tricky to get the
// synchronization right.  Would we just force the TX/RX queues to
// empty?
void VRBRAINUARTDriver::flush()
{
}

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
  return usart_is_tx_pending(m_dev);
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
