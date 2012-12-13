#ifndef _HAL_H_
#define _HAL_H_

#include <errno.h>

#include <stm32f4xx.h>

#include "hal_types.h"
#include "stm32.h"
#include "stopwatch.h"
#include "util.h"
#include "gpio.h"
#include "delay.h"
#include "adc.h"

#define OK		1
#define ERROR	0

/*
 * Where to put usercode, based on space reserved for bootloader.
 *
 * FIXME this has no business being here
 */
#define USER_ADDR_ROM 0x08005000
#define USER_ADDR_RAM 0x20000C00
#define STACK_TOP     0x20000800

#endif

