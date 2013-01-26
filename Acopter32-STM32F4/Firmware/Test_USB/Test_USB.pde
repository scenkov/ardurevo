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

void setup() 
{
    hal.console->println_P("hello world");
    abs1 = -15;
    abs2 = -123.897;
}

void loop()
{  	


	hal.console->println("*");
	hal.console->printf_P(PSTR("abs1: %d abs2:%f"),abs(abs1),fabs(abs2));
	delay(1000);
	
}

AP_HAL_MAIN();
