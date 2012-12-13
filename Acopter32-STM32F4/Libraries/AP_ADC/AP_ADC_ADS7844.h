/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_ADC_ADS7844_H
#define AP_ADC_ADS7844_H

//#define AP_ADC_ADS7844_DEBUG_ENABLE		//Enable debug

#define bit_set(p,m)   ((p) |= ( 1<<m))
#define bit_clear(p,m) ((p) &= ~(1<<m))

// DO NOT CHANGE FROM 8!!
#define ADC_ACCEL_FILTER_SIZE 8

#include <stdlib.h>

#include <wirish.h>
#include "AP_ADC.h"
#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"

class AP_ADC_ADS7844 : public AP_ADC
{
public:
	AP_ADC_ADS7844(uint8_t cs_pin, HardwareSPI *spi_dev);

	void Init( AP_PeriodicProcess * scheduler );

	// Read 1 sensor value
	float Ch(unsigned char ch_num);

	// Read 6 sensors at once
	uint32_t Ch6(const uint8_t *channel_numbers, float *result);

    // check if Ch6 would block
	bool new_data_available(const uint8_t *channel_numbers);

	void read();
	bool automaticRead();

private:
	static uint8_t		_cs_pin;
	static HardwareSPI	*_SPIx;
    AP_PeriodicProcess * _scheduler;

    static void read(uint32_t);

};

#endif
