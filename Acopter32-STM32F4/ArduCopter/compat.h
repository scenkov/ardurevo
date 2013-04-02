
#ifndef __COMPAT_H__
#define __COMPAT_H__

#if CONFIG_HAL_BOARD  != HAL_BOARD_VRBRAIN
#define OUTPUT GPIO_OUTPUT
#define INPUT GPIO_INPUT
#endif

#define HIGH 1
#define LOW 0

/* Forward declarations to avoid broken auto-prototyper (coughs on '::'?) */
static void run_cli(AP_HAL::UARTDriver *port);

#endif // __COMPAT_H__

