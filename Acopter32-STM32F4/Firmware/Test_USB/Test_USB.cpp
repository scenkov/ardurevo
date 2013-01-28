#line 1 "./Firmware/Test_USB/Test_USB.pde"
// Libraries
//#include <FastSerial.h>
#include "Wirish.h"
#include <AP_Common.h>
#include <AP_Progmem.h>

#include <AP_Param.h>
#include <AP_Math.h>


#include <AP_HAL.h>
#include <AP_HAL_VRBRAIN.h>

//#include <EEPROM.h>
#include <AP_Compass.h>

#define EEPROM_MAX_ADDR		4096

  void setup()  ;
  void loop() ;
#line 19 "./Firmware/Test_USB/Test_USB.pde"
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
int abs1;
float abs2;
const prog_char_t *msg;
AP_Compass_HMC5843 compass;

void setup() 
{
    hal.console->println_P("hello world");
    abs1 = -15;
    abs2 = -123.897;
    msg = PSTR("It works!");
    //EEPROM.init();
    //compass.set_orientation(MAG_ORIENTATION);                                                   // set compass's orientation on aircraft
    //if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
	//hal.console->println_P(PSTR("COMPASS INIT ERROR"));
    //}
}

void loop()
{  	


	hal.console->println("*");
	hal.console->printf_P(PSTR("abs1: %d abs2:%f and %S"),abs(abs1),fabs(abs2),msg);
	hal.scheduler->delay(1000);
	/*
	 *
	 if (compass.read()) {
		//float heading = compass.calculate_heading(ahrs.get_dcm_matrix());
	        hal.console->printf_P(PSTR("XYZ: %d, %d, %d\n"),
	                                compass.mag_x,
	                                compass.mag_y,
	                                compass.mag_z);
	        } else {
	            hal.console->println_P(PSTR("not healthy"));
	}*/
	    for (int i = 0; i < EEPROM_MAX_ADDR; i += 16) {
	        hal.console->printf_P(PSTR("%04x:"), i);
	        for (int j = 0; j < 16; j++)
	            hal.console->printf_P(PSTR(" %02x"), hal.storage->read_byte((const uint8_t *)(i + j)));
	        hal.console->println();
	    }
	
}

AP_HAL_MAIN();
