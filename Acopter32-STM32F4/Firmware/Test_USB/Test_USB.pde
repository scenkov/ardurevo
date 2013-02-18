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
//#include <AP_Compass.h>

#define EEPROM_MAX_ADDR		4096

const AP_HAL::HAL& hal = AP_HAL_VRBRAIN;
int abs1;
float abs2;
const prog_char_t *msg;

#define EEPROM_ADDRESS	0xA0
#define EEPROM_START_ADDRESS	0x00

uint8_t fibs[12] = { 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144 };

void test_erase() {
    hal.console->printf_P(PSTR("erasing... "));
    hal.console->println();
    for(int i = 0; i < 100; i++) {
        hal.storage->write_byte(i, 0);
    }
    hal.console->printf_P(PSTR(" done.\r\n"));
    hal.console->println();
}

void test_write() {
    hal.console->printf_P(PSTR("writing... "));
    hal.console->println();
    hal.storage->write_block(0, fibs, 12);
    hal.console->printf_P(PSTR(" done.\r\n"));
    hal.console->println();
}

void test_readback() {
    hal.console->printf_P(PSTR("reading back...\r\n"));
    hal.console->println();
    uint8_t readback[12];
    bool success = true;
    hal.storage->read_block(readback, 0, 12);
    for (int i = 0; i < 12; i++) {
        if (readback[i] != fibs[i]) {
            success = false;
            hal.console->printf_P(PSTR("At index %d expected %d got %d\r\n"),
                    i, (int) fibs[i], (int) readback[i]);
            hal.console->println();
        }
    }
    if (success) {
        hal.console->printf_P(PSTR("all bytes read successfully\r\n"));
        hal.console->println();
    }
    uint8_t aa = hal.storage->read_byte(10);
    hal.console->printf_P(PSTR("done reading back %d.\r\n"),aa);
}
void setup() 
{
    hal.console->printf_P(PSTR("Starting AP_HAL_AVR::Storage test\r\n"));
    test_erase();
    test_write();
    test_readback();
}

void loop()
{  	
	
}

AP_HAL_MAIN();
