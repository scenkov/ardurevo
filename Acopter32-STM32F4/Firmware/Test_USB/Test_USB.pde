// Libraries
#include <FastSerial.h>
#include "Wirish.h"
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
//#include <AP_Math.h>

#include <AP_HAL.h>
#include <AP_HAL_VRBRAIN.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


void setup() 
{
    hal.console->println_P("hello world");
}

void loop()
{  	

	hal.console->println("*");
	delay(1000);
	
}

AP_HAL_MAIN();
