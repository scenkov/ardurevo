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
#include <stdlib.h>

#include "Console.h"

using namespace VRBRAIN;

VRBRAINConsoleDriver::VRBRAINConsoleDriver(AP_HAL::BetterStream* delegate) :
  _d(delegate), _user_backend(false)
{
}

void VRBRAINConsoleDriver::init(void *args)
{
}

void VRBRAINConsoleDriver::backend_open()
{
  _txbuf.allocate(128);
  _rxbuf.allocate(16);
  _user_backend = true;
}

void VRBRAINConsoleDriver::backend_close()
{
  _user_backend = false;
}

size_t VRBRAINConsoleDriver::backend_read(uint8_t *data, size_t len)
{
  for (size_t i = 0; i < len; i++) {
    int16_t b = _txbuf.pop();
    if (b != -1) {
      data[i] = (uint8_t) b;
    } else {
      return i;
    }
  }

  return len;
}

size_t VRBRAINConsoleDriver::backend_write(const uint8_t *data, size_t len)
{
   for (size_t i = 0; i < len; i++) {
     bool valid = _rxbuf.push(data[i]);
     if (!valid) {
       return i;
     }
   }

   return len;
 }

void VRBRAINConsoleDriver::print_P(const prog_char_t *pstr)
{
  _d->print_P(pstr);
}

void VRBRAINConsoleDriver::println_P(const prog_char_t *pstr)
{
  _d->println_P(pstr);
}

void VRBRAINConsoleDriver::printf(const char *pstr, ...)
{
  va_list ap;
  va_start(ap, pstr);
  _d->vprintf(pstr, ap);
  va_end(ap);
}

void VRBRAINConsoleDriver::_printf_P(const prog_char *pstr, ...)
{
  va_list ap;
  va_start(ap, pstr);
  _d->vprintf_P(pstr, ap);
  va_end(ap);
}

void VRBRAINConsoleDriver::vprintf(const char *pstr, va_list ap)
{
  _d->vprintf(pstr, ap);
}

void VRBRAINConsoleDriver::vprintf_P(const prog_char *pstr, va_list ap)
{
  _d->vprintf(pstr, ap);
}

int16_t VRBRAINConsoleDriver::available()
{
  if (_user_backend) {
    return _rxbuf.bytes_used();
  } else {
    return _d->available();
  }
}

int16_t VRBRAINConsoleDriver::txspace()
{
  if (_user_backend) {
    return _txbuf.bytes_free();
  } else {
    return _d->txspace();
  }
}

int16_t VRBRAINConsoleDriver::read()
{
  if (_user_backend) {
    return _rxbuf.pop();
  } else {
    return _d->read();
  }
}

int16_t VRBRAINConsoleDriver::peek()
{
  if (_user_backend) {
    return _rxbuf.peek();
  } else {
    return _d->peek();
  }
}

size_t VRBRAINConsoleDriver::write(uint8_t c)
{
  if (_user_backend) {
    return (_txbuf.push(c) ? 1 : 0);
  } else {
    return _d->write(c);
  }
}

/**
 * VRBRAINConsoleDriver::Buffer implementation.
 * A synchronous nonblocking ring buffer, based on the AVRUARTDriver::Buffer
 */

bool VRBRAINConsoleDriver::Buffer::allocate(uint16_t size)
{
  _head = 0;
  _tail = 0;
  uint8_t shift;
  /* Hardcoded max size of 1024. sue me. */
  for ( shift = 1;
        ( 1 << shift ) < 1024 && ( 1 << shift) < size;
        shift++
    ) ;
  uint16_t tmpmask  = (1 << shift) - 1;

  if ( _bytes != NULL ) {
    if ( _mask == tmpmask ) {
      return true;
    }
    free(_bytes);
  }
  _mask = tmpmask;
  _bytes = (uint8_t*) malloc(_mask+1);
  return (_bytes != NULL);
}

bool VRBRAINConsoleDriver::Buffer::push(uint8_t b)
{
  uint16_t next = (_head + 1) & _mask;
  if ( next == _tail ) {
    return false;
  }
  _bytes[_head] = b;
  _head = next;
  return true;
}

int16_t VRBRAINConsoleDriver::Buffer::pop()
{
  if ( _tail == _head ) {
    return -1;
  }
  uint8_t b = _bytes[_tail];
  _tail = ( _tail + 1 ) & _mask;
  return (int16_t) b;
}

int16_t VRBRAINConsoleDriver::Buffer::peek()
{
  if ( _tail == _head ) {
    return -1;
  }
  uint8_t b = _bytes[_tail];
  return (int16_t) b;
}

uint16_t VRBRAINConsoleDriver::Buffer::bytes_used()
{
  return ((_head - _tail) & _mask);
}

uint16_t VRBRAINConsoleDriver::Buffer::bytes_free()
{
  return ((_mask+1) - ((_head - _tail) & _mask));
}
