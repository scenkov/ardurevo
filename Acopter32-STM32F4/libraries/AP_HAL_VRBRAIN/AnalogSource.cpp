/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Copied from: Flymaple port by Mike McCauley
 */
#define ANLOGSOURCRE_DEBUGGING 0

#if ANLOGIN_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
 # define Debug(fmt, args ...)
#endif


#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include <AP_HAL.h>
#include "AnalogIn.h"
#include <adc.h>
#include <boards.h>
#include <gpio_hal.h>
#include <GPIO.h>

extern const AP_HAL::HAL& hal;

using namespace VRBRAIN;

VRBRAINAnalogSource::VRBRAINAnalogSource(uint8_t pin) :
    _sum_count(0),
    _sum(0),
    _last_average(0),
    _pin(ANALOG_INPUT_NONE),
    _stop_pin(ANALOG_INPUT_NONE),
    _settle_time_ms(0)
{
    Debug("Initialize VRBRAINAnalogSource pin %u", pin)
    set_pin(pin);
}

float VRBRAINAnalogSource::read_average() {
    return _read_average();
}

float VRBRAINAnalogSource::read_latest() {
    noInterrupts();
    uint16_t latest = _latest;
    interrupts();
    return latest;
}

/*
  return voltage from 0.0 to 3.3V, scaled to Vcc
 */
float VRBRAINAnalogSource::voltage_average(void)
{
    return voltage_average_ratiometric();
}

float VRBRAINAnalogSource::voltage_latest(void)
{
    float v = read_latest();
    return v * (3.3f / 4096.0f);
}

/*
  return voltage from 0.0 to 3.3V, assuming a ratiometric sensor. This
  means the result is really a pseudo-voltage, that assumes the supply
  voltage is exactly 3.3V.
 */
float VRBRAINAnalogSource::voltage_average_ratiometric(void)
{
    float v = read_average();
    return v * (3.3f / 4096.0f);
}

void VRBRAINAnalogSource::set_pin(uint8_t pin) {
    if (pin == _pin)
	return;

    // ensure the pin is marked as an INPUT pin
    if (pin != ANALOG_INPUT_NONE && pin != ANALOG_INPUT_BOARD_VCC && pin < BOARD_NR_GPIO_PINS && pin > 0) {
        int8_t dpin = hal.gpio->analogPinToDigitalPin(pin);
        if (dpin != -1) {
            hal.gpio->pinMode(dpin, INPUT_ANALOG);
        }
    }

    noInterrupts();
    _sum = 0;
    _sum_count = 0;
    _last_average = 0;
    _latest = 0;
    _pin = pin;
    interrupts();
}

void VRBRAINAnalogSource::set_stop_pin(uint8_t pin) {
    _stop_pin = pin;
}

void VRBRAINAnalogSource::set_settle_time(uint16_t settle_time_ms)
{
    _settle_time_ms = settle_time_ms;
}

/* read_average is called from the normal thread (not an interrupt). */
float VRBRAINAnalogSource::_read_average()
{
    uint16_t sum;
    uint8_t sum_count;

    if (_sum_count == 0) {
        // avoid blocking waiting for new samples
        return _last_average;
    }

    /* Read and clear in a critical section */
    noInterrupts();
    sum = _sum;
    sum_count = _sum_count;
    _sum = 0;
    _sum_count = 0;

    interrupts();
    float avg = sum / (float) sum_count;

    _last_average = avg;
    return avg;
}

void VRBRAINAnalogSource::setup_read() {
    if (_stop_pin != ANALOG_INPUT_NONE) {
        uint8_t digital_pin = hal.gpio->analogPinToDigitalPin(_stop_pin);
        hal.gpio->pinMode(digital_pin, GPIO_OUTPUT);
        hal.gpio->write(digital_pin, 1);
    }
    if (_settle_time_ms != 0) {
        _read_start_time_ms = hal.scheduler->millis();
    }
    const adc_dev *dev = PIN_MAP[_pin].adc_device;

    if(dev != NULL)
	{
	adc_set_reg_seqlen(dev, 1);
	uint8_t channel = 0;

	if (_pin == ANALOG_INPUT_BOARD_VCC){
	    ADC_TempSensorVrefintCmd(ENABLE);
	    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 2, ADC_SampleTime_56Cycles);
	}
	else if (_pin == ANALOG_INPUT_NONE)
	    ; // NOOP
	else {
	    channel = PIN_MAP[_pin].adc_channel;
	}
	if(channel != 0){
	    adc_disable(dev);
	    ADC_RegularChannelConfig(dev->adcx, channel, 1, ADC_SampleTime_56Cycles);
	    adc_enable(dev);
	}

	}
}

void VRBRAINAnalogSource::stop_read() {
    if(_pin == ANALOG_INPUT_BOARD_VCC) {
	ADC_TempSensorVrefintCmd(DISABLE);
    }
    if (_stop_pin != ANALOG_INPUT_NONE) {
        uint8_t digital_pin = hal.gpio->analogPinToDigitalPin(_stop_pin);
        hal.gpio->pinMode(digital_pin, GPIO_OUTPUT);
        hal.gpio->write(digital_pin, 0);
    }
}

bool VRBRAINAnalogSource::reading_settled()
{
    if (_settle_time_ms != 0 && (hal.scheduler->millis() - _read_start_time_ms) < _settle_time_ms) {
        return false;
    }
    return true;
}

/* new_sample is called from an interrupt. It always has access to
 *  _sum and _sum_count. Lock out the interrupts briefly with
 * noInterrupts()/interrupts() to read these variables from outside an interrupt. */
void VRBRAINAnalogSource::new_sample(uint16_t sample) {
    _sum += sample;
    _latest = sample;
    // Copied from AVR code in ArduPlane-2.74b, but AVR code is wrong!
    if (_sum_count >= 15) { // VRBRAIN has a 12 bit ADC, so can only sum 16 in a uint16_t
        _sum >>= 1;
        _sum_count = 8;
    } else {
        _sum_count++;
    }
}

#endif 
