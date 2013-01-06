/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
	APM_MS5611.cpp - Arduino Library for MS5611-01BA01 absolute pressure sensor
	Code by Jose Julio, Pat Hickey and Jordi Muñoz. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Sensor is conected to standard SPI port
	Chip Select pin: Analog2 (provisional until Jordi defines the pin)!!

	Variables:
		Temp : Calculated temperature (in Celsius degrees * 100)
		Press : Calculated pressure   (in mbar units * 100)


	Methods:
		init() : Initialization and sensor reset
		read() : Read sensor data and _calculate Temperature, Pressure
		         This function is optimized so the main host don´t need to wait
				 You can call this function in your main loop
				 Maximum data output frequency 100Hz - this allows maximum oversampling in the chip ADC
				 It returns a 1 if there are new data.
		get_pressure() : return pressure in mbar*100 units
		get_temperature() : return temperature in celsius degrees*100 units

	Internal functions:
		_calculate() : Calculate Temperature and Pressure (temperature compensated) in real units


*/

#include <SPI.h>
#include "AP_Baro_MS5611.h"


/* on APM v.24 MS5661_CS is PG1 (Arduino pin 40 MP32 pin 64) */
//#define MS5611_CS 94

static FastSerial *serPort;

#ifdef AP_BARO_MS5611_DEBUG_ENABLE
#pragma message "*** AP_BARO_MS5611 Debug Enabled ***"
#define debug(fmt, args...) do { if (serPort != NULL) { serPort->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__ , ##args); delay(100); } } while(0)
#else
#define debug(fmt, args...)
#endif
#define notify(fmt, args...) do { if (serPort != NULL) { serPort->printf(fmt, ##args); delay(100); } } while(0)

#define CMD_MS5611_RESET 0x1E
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_C1 0xA2
#define CMD_MS5611_PROM_C2 0xA4
#define CMD_MS5611_PROM_C3 0xA6
#define CMD_MS5611_PROM_C4 0xA8
#define CMD_MS5611_PROM_C5 0xAA
#define CMD_MS5611_PROM_C6 0xAC
#define CMD_MS5611_PROM_CRC 0xAE
#define CMD_CONVERT_D1_OSR4096 0x48   // Maximum resolution (oversampling)
#define CMD_CONVERT_D2_OSR4096 0x58   // Maximum resolution (oversampling)

uint32_t volatile AP_Baro_MS5611::_s_D1;
uint32_t volatile AP_Baro_MS5611::_s_D2;
uint8_t  volatile AP_Baro_MS5611::_d1_count;
uint8_t  volatile AP_Baro_MS5611::_d2_count;
uint8_t  AP_Baro_MS5611::_state;
uint32_t AP_Baro_MS5611::_timer;
bool     volatile AP_Baro_MS5611::_updated;
uint8_t  AP_Baro_MS5611::_cs_pin;
HardwareSPI *AP_Baro_MS5611::_SPIx;

AP_Baro_MS5611::AP_Baro_MS5611(uint8_t cs_pin, HardwareSPI *spi_dev, FastSerial *ser_port)
{
	serPort = ser_port;
    _scheduler = NULL;
	_cs_pin = cs_pin;
	_SPIx = spi_dev;
}

uint8_t AP_Baro_MS5611::_spi_read(uint8_t reg)
{
  uint8_t return_value;
  uint8_t addr = reg; // | 0x80; // Set most significant bit
  digitalWrite(_cs_pin, LOW);
  _SPIx->transfer(addr);
  return_value = _SPIx->transfer(0);
  digitalWrite(_cs_pin, HIGH);
  return return_value;
}

uint16_t AP_Baro_MS5611::_spi_read_16bits(uint8_t reg)
{
  uint8_t byteH, byteL;
  uint16_t return_value;
  uint8_t addr = reg; // | 0x80; // Set most significant bit
  digitalWrite(_cs_pin, LOW);
  _SPIx->transfer(addr);
  byteH = _SPIx->transfer(0);
  byteL = _SPIx->transfer(0);
  digitalWrite(_cs_pin, HIGH);
  return_value = ((uint16_t)byteH<<8) | (byteL);
  return return_value;
}

uint32_t AP_Baro_MS5611::_spi_read_adc()
{
  uint8_t byteH,byteM,byteL;
  uint32_t return_value;
  uint8_t addr = 0x00;
  digitalWrite(_cs_pin, LOW);
  _SPIx->transfer(addr);
  byteH = _SPIx->transfer(0);
  byteM = _SPIx->transfer(0);
  byteL = _SPIx->transfer(0);
  digitalWrite(_cs_pin, HIGH);
  return_value = (((uint32_t)byteH)<<16) | (((uint32_t)byteM)<<8) | (byteL);
  return return_value;
}


void AP_Baro_MS5611::_spi_write(uint8_t reg)
{
  digitalWrite(_cs_pin, LOW);
  _SPIx->transfer(reg);
  digitalWrite(_cs_pin, HIGH);
}

// Public Methods //////////////////////////////////////////////////////////////
// SPI should be initialized externally
bool AP_Baro_MS5611::init( AP_PeriodicProcess *scheduler )
{
    //scheduler->suspend_timer();

	pinMode(_cs_pin, OUTPUT);	 // Chip select Pin
	digitalWrite(_cs_pin, HIGH);
	delay(1);

	_spi_write(CMD_MS5611_RESET);
	delay(100);

	// We read the factory calibration
  // The on-chip CRC is not used
	C1 = _spi_read_16bits(CMD_MS5611_PROM_C1);
	C2 = _spi_read_16bits(CMD_MS5611_PROM_C2);
	C3 = _spi_read_16bits(CMD_MS5611_PROM_C3);
	C4 = _spi_read_16bits(CMD_MS5611_PROM_C4);
	C5 = _spi_read_16bits(CMD_MS5611_PROM_C5);
	C6 = _spi_read_16bits(CMD_MS5611_PROM_C6);


	//Send a command to read Temp first
	_spi_write(CMD_CONVERT_D2_OSR4096);
	_timer = micros();
	_state = 0;
	Temp=0;
	Press=0;

    _s_D1 = 0;
    _s_D2 = 0;
    _d1_count = 0;
    _d2_count = 0;

	//scheduler->resume_timer();
	//_scheduler = scheduler;
	//if (_scheduler)
	//	_scheduler->register_process( AP_Baro_MS5611::_update );


	// wait for at least one value to be read
	while (!_updated)
	{
		update();
		delay(10);
	}
	read();
	healthy = true;
    return true;
}

void AP_Baro_MS5611::update()
{
	AP_Baro_MS5611::_update((uint32_t)micros());
}
/*
bool AP_Baro_MS5611::automaticRead()
{
	if (_scheduler)
		return true;
	return false;
}
*/
// Read the sensor. This is a state machine
// We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
// temperature does not change so quickly...
void AP_Baro_MS5611::_update(uint32_t tnow)
{
    // Throttle read rate to 100hz maximum.
    // note we use 9500us here not 10000us
    // the read rate will end up at exactly 100hz because the Periodic Timer fires at 1khz
    if (tnow - _timer < 9500) {
        return;
    }

    _timer = tnow;

    if (_state == 0) {
	    _s_D2 += _spi_read_adc();  				 // On state 0 we read temp
        _d2_count++;
        if (_d2_count == 32) {
            // we have summed 32 values. This only happens
            // when we stop reading the barometer for a long time
            // (more than 1.2 seconds)
            _s_D2 >>= 1;
            _d2_count = 16;
        }
	    _state++;
	    _spi_write(CMD_CONVERT_D1_OSR4096);  // Command to read pressure
    } else {
	    _s_D1 += _spi_read_adc();
        _d1_count++;
        if (_d1_count == 128) {
            // we have summed 128 values. This only happens
            // when we stop reading the barometer for a long time
            // (more than 1.2 seconds)
            _s_D1 >>= 1;
            _d1_count = 64;
        }
	    _state++;
	    _updated = true;					                // New pressure reading
        if (_state == 5) {
            _spi_write(CMD_CONVERT_D2_OSR4096);	// Command to read temperature
            _state = 0;
        } else {
           _spi_write(CMD_CONVERT_D1_OSR4096);  // Command to read pressure
        }
    }
}

uint8_t AP_Baro_MS5611::read()
{
    bool updated = _updated;
    if (updated) {
        uint32_t sD1, sD2;
        uint8_t d1count, d2count;
        // we need to disable interrupts to access
        // _s_D1 and _s_D2 as they are not atomic
        noInterrupts();
        sD1 = _s_D1;
        _s_D1 = 0;
        sD2 = _s_D2;
        _s_D2 = 0;
        d1count = _d1_count;
        _d1_count = 0;
        d2count = _d2_count;
        _d2_count = 0;
        _updated = false;
        interrupts();
        if (d1count != 0) {
            D1 = (sD1) / d1count;
        }
        if (d2count != 0) {
            D2 = (sD2) / d2count;
        }
        _pressure_samples = d1count;
        _raw_press = D1;
        _raw_temp = D2;
    }
    _calculate();
    if (updated) {
        _last_update = millis();
    }
    return updated ? 1 : 0;
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_MS5611::_calculate()
{
	int32_t dT;
	int64_t TEMP;  // 64 bits
	int64_t OFF;
	int64_t SENS;
	int64_t P;

	// Formulas from manufacturer datasheet
	// as per data sheet some intermediate results require over 32 bits, therefore
  // we define parameters as 64 bits to prevent overflow on operations
  // sub -20c temperature compensation is not included
	dT = D2-((long)C5*256);
	TEMP = 2000 + ((int64_t)dT * C6)/8388608;
	OFF = (int64_t)C2 * 65536 + ((int64_t)C4 * dT ) / 128;
	SENS = (int64_t)C1 * 32768 + ((int64_t)C3 * dT) / 256;

	if (TEMP < 2000){   // second order temperature compensation
		int64_t T2 = (((int64_t)dT)*dT) >> 31;
		int64_t Aux_64 = (TEMP-2000)*(TEMP-2000);
		int64_t OFF2 = (5*Aux_64)>>1;
		int64_t SENS2 = (5*Aux_64)>>2;
		TEMP = TEMP - T2;
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
	}

	P = (D1*SENS/2097152 - OFF)/32768;
	Temp = TEMP;
	Press = P;
}

float AP_Baro_MS5611::get_pressure()
{
	return ((float)Press);
}

float AP_Baro_MS5611::get_temperature()
{
	// callers want the temperature in 0.1C units
	return ((float)Temp/10.0);
}

int32_t AP_Baro_MS5611::get_raw_pressure() {
	return _raw_press;
}

int32_t AP_Baro_MS5611::get_raw_temp() {
	return _raw_temp;
}


