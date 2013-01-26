// Libraries
//#include <FastSerial.h>
#include "Wirish.h"
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>

#include <AP_HAL.h>
#include <AP_HAL_VRBRAIN.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
int abs1;
float abs2;
const prog_char_t *msg;

void setup() 
{
    hal.console->println_P("hello world");
    abs1 = -15;
    abs2 = -123.897;
    msg = PSTR("It works!");
}

void loop()
{  	


	hal.console->println("*");
	hal.console->printf_P(PSTR("abs1: %d abs2:%f and %S"),abs(abs1),fabs(abs2),msg);
	delay(1000);
	
}

AP_HAL_MAIN();
