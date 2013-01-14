
#ifndef __AP_HAL_VRBRAIN_UARTDRIVER_H__
#define __AP_HAL_VRBRAIN_UARTDRIVER_H__

#include <AP_HAL_VRBRAIN.h>
#include <usart.h>
#include <usb.h>
#include <gpio.h>
#include <wirish.h>

#define DEFAULT_TX_TIMEOUT 10000 // 10 ms
#define _USB99 99

class VRBRAIN::VRBRAINUARTDriver : public AP_HAL::UARTDriver
{
private:
    usart_dev *usart_device;
    uint8 tx_pin;
    uint8 rx_pin;
public:
    uint8_t usb;
    uint8_t usb_present;
public:
  VRBRAINUARTDriver(struct usart *dev);

  /* VRBRAIN implementations of UARTDriver virtual methods */
  void begin(uint32_t b);
  void begin(uint32_t b, uint16_t rxS, uint16_t txS);
  void end();
  void flush();
  bool is_initialized();
  void set_blocking_writes(bool blocking);
  bool tx_pending();

  /* VRBRAIN implementations of BetterStream virtual methods */
  void print_P(const prog_char_t *pstr);
  void println_P(const prog_char_t *pstr);
  void printf(const char *pstr, ...);
  void _printf_P(const prog_char *pstr, ...);

  void vprintf(const char* fmt, va_list ap);
  void vprintf_P(const prog_char* fmt, va_list ap);

  /* VRBRAIN implementations of Stream virtual methods */
  int16_t available();
  int16_t txspace();
  int16_t read();
  int16_t peek();

  /* VRBRAIN implementations of Print virtual methods */
  size_t write(uint8_t c);

private:
  struct usart *m_dev;
  bool m_initialized;
  bool m_blocking;
};


//TEO 20110505
//definisco qui i parametri per le varie seriali preconfigurate
#define FSUSART0 	_USART1
#define FSTXPIN0 	BOARD_USART1_TX_PIN //9
#define FSRXPIN0 	BOARD_USART1_RX_PIN //10

#define FSUSART1 	_USART2
#define FSTXPIN1 	BOARD_USART2_TX_PIN //5
#define FSRXPIN1 	BOARD_USART2_RX_PIN //6

#define FSUSART2 	_USART3
#define FSTXPIN2 	BOARD_USART3_TX_PIN //8
#define FSRXPIN2 	BOARD_USART3_RX_PIN //9

#define FSUSART3 	_UART4
#define FSTXPIN3 	BOARD_UART4_TX_PIN //10
#define FSRXPIN3 	BOARD_UART4_RX_PIN //11


#endif // __AP_HAL_VRBRAIN_UARTDRIVER_H__
