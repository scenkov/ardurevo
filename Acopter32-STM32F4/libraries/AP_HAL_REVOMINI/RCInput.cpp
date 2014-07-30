/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


#include <AP_HAL.h>
#include <exti.h>
#include <timer.h>
#include "RCInput.h"
#include <pwm_in.h>
#include <utility/SBUS.h>



// Constructors ////////////////////////////////////////////////////////////////
using namespace AP_HAL;
using namespace REVOMINI;


extern const AP_HAL::HAL& hal;

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
// PATCH FOR FAILSAFE AND FRSKY
#define MINOFFWIDTH 1000
#define MAXOFFWIDTH 22000

#define MINCHECK 900
#define MAXCHECK 2100

/* private variables to communicate with input capture isr */
volatile uint16_t REVOMINIRCInput::_channel[REVOMINI_RC_INPUT_NUM_CHANNELS] = {0};
volatile uint32_t REVOMINIRCInput::_last_pulse[REVOMINI_RC_INPUT_NUM_CHANNELS] = {0};
volatile uint8_t  REVOMINIRCInput::_valid_channels = 0;
volatile uint32_t REVOMINIRCInput::_last_input_interrupt_time = 0; // Last time the input interrupt ran

volatile unsigned char radio_status_rc = 0;
volatile unsigned char sync = 0;
volatile unsigned int currentChannel = 0;

unsigned int uiRcErrCnt1 = 0;
unsigned int uiRcErrCnt2 = 0;
unsigned int uiRcErrCnt3 = 0;

typedef struct
    {
    byte edge;
    unsigned long riseTime;
    unsigned long fallTime;
    unsigned int lastGoodWidth;
    } tPinTimingData;
volatile static tPinTimingData pinData[8];



REVOMINIRCInput::REVOMINIRCInput()
    {
    }

void REVOMINIRCInput::init(void* machtnichts)
    {
/*
    input_channel_ch1 = 75;
    input_channel_ch2 = 80;
    input_channel_ch3 = 86;
    input_channel_ch4 = 89;
    input_channel_ch5 = 12;
    input_channel_ch6 = 13;
    input_channel_ch7 = 14;
    input_channel_ch8 = 15;
*/
    input_channel_ch1 = 4;  // PB14 T12/1
    input_channel_ch2 = 5;  // PB15 T12/2
    input_channel_ch3 = 12; // PC6 T8/1
    input_channel_ch4 = 13; // PC7 T8/2
    input_channel_ch5 = 14; // PC8 T8/3
    input_channel_ch6 = 15; // PC9 T8/4

    _detect_rc();

    switch (_rc_type) {
    case SBUS:
	g_is_ppmsum = 3;
        _sbus = new SBUSClass(hal.uartD);
        _sbus->begin();
        break;
    case PPMSUM:
	g_is_ppmsum = 1;
	attachPWMCaptureCallback(rxIntPPMSUM);
	pwmInit();
	break;
    case PWM:
    default:
	g_is_ppmsum = 0;
        for (byte channel = 0; channel < 8; channel++) {
            pinData[channel].edge = FALLING_EDGE;
        }
        attachPWMCaptureCallback(rxIntPWM);
        pwmInit();
        break;

    }
    clear_overrides();
}
bool REVOMINIRCInput::new_input() {
    if (g_is_ppmsum < 3 && (hal.scheduler->millis() - _last_input_interrupt_time) > 500)
	_valid_channels = 0; // Lost RC Input?
    return _valid_channels != 0;
}

uint8_t REVOMINIRCInput::num_channels()
    {
	return _valid_channels;
    }

/* constrain captured pulse to be between min and max pulsewidth. */
static inline uint16_t constrain_pulse(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}
/* constrain captured pulse to be between min and max pulsewidth. */
static inline bool check_pulse(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return false;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return false;
    return true;
}

uint16_t REVOMINIRCInput::read(uint8_t ch)
    {
    uint16_t data;

    noInterrupts();
    if (g_is_ppmsum == 3) {
	data = _sbus->getChannel(ch);
	_valid_channels = 14;

    } else {
	data = _channel[ch];
    }
    interrupts();

    /* Check for override */
    uint16_t over = _override[ch];

    return (over == 0) ? data : over;
    }

uint8_t REVOMINIRCInput::read(uint16_t* periods, uint8_t len)
    {
    noInterrupts();
    for (uint8_t i = 0; i < len; i++) {
	if (g_is_ppmsum == 3) { //SBUS
	    periods[i] = _sbus->getChannel(i);
	    _valid_channels = 14;
	} else {
	    periods[i] = _channel[i];
	}
    }
    interrupts();

    for (uint8_t i = 0; i < len; i++) {
	if (_override[i] != 0) {
	    periods[i] = _override[i];
	}
    }

    return len;
    }

bool REVOMINIRCInput::set_overrides(int16_t *overrides, uint8_t len)
    {
    bool res = false;
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
    }

bool REVOMINIRCInput::set_override(uint8_t channel, int16_t override)
    {
    if (override < 0) return false; /* -1: no change. */
    if (channel < REVOMINI_RC_INPUT_NUM_CHANNELS) {
        _override[channel] = override;
        if (override != 0) {
            return true;
        }
    }
    return false;
    }

void REVOMINIRCInput::clear_overrides()
    {
    for (uint8_t i = 0; i < REVOMINI_RC_INPUT_NUM_CHANNELS; i++) {
	_override[i] = 0;
    }
    }

void REVOMINIRCInput::rxIntPPMSUM(uint8_t state, uint16_t value)
    {
    static uint8_t  channel_ctr;
    _last_input_interrupt_time = hal.scheduler->millis();

    if (value > 4000) // Frame synchronization
	{
	    if( channel_ctr >= REVOMINI_RC_INPUT_MIN_CHANNELS ) {
		_valid_channels = channel_ctr;
	    }
	    channel_ctr = 0;
	}
    else
	{
        if (channel_ctr < REVOMINI_RC_INPUT_NUM_CHANNELS) {
            _channel[channel_ctr] = check_pulse(value)?value:_channel[channel_ctr]; //sanity check
            _last_pulse[channel_ctr] = hal.scheduler->millis();;
            channel_ctr++;
            if (channel_ctr == REVOMINI_RC_INPUT_NUM_CHANNELS) {
                _valid_channels = REVOMINI_RC_INPUT_NUM_CHANNELS;
            }
        }

	}
    }

void REVOMINIRCInput::rxIntPWM(uint8_t channel, uint16_t value)
    {
    _last_input_interrupt_time = hal.scheduler->millis();
    _channel[channel] = value;
    _last_pulse[channel] = hal.scheduler->millis();
    _valid_channels = REVOMINI_RC_INPUT_NUM_CHANNELS;
    }

void REVOMINIRCInput::_detect_rc(){

    /*try to detect correct RC type*/
    _detected = false;
    _rc_type = PWM;
/*
    //First try with SBUS
    _sbus_dct();
    if (_detected) {
	hal.console->println("Init SBUS");
	return;
    }
*/
    //Then try with PPMSUM
    _ppmsum_dct();
    if (_detected) {
	hal.console->println("Init PPMSUM");
	return;
    }

    hal.console->println("Init PWM");
    // Else we have standard PWM
    _detected = true;
    _rc_type = PWM;

}

bool REVOMINIRCInput::_sbus_dct(){
    return false;
}

bool REVOMINIRCInput::_ppmsum_dct(){
    /*initial check for pin2-pin3 bridge. If detected switch to PPMSUM  */
    uint8_t channel3_status = 0;
        uint8_t pin2, pin3;
        //input pin 2
        pin2 = input_channel_ch2;
        //input pin 3
        pin3 = input_channel_ch3;

        //set pin2 as output and pin 3 as input
        hal.gpio->pinMode(pin2, OUTPUT);
        hal.gpio->pinMode(pin3, INPUT_PULLUP);

        //default pin3 to 0
        hal.gpio->write(pin3, 0);
        hal.scheduler->delay(1);

        //write 1 to pin 2 and read pin3
        hal.gpio->write(pin2, 1);
        hal.scheduler->delay(1);
        //if pin3 is 1 increment counter
        if (hal.gpio->read(pin3) == 1)
    	channel3_status++;

        //write 0 to pin 2 and read pin3
        hal.gpio->write(pin2, 0);
        hal.scheduler->delay(1);
        //if pin3 is 0 increment counter
        if (hal.gpio->read(pin3) == 0)
    	channel3_status++;

        //write 1 to pin 2 and read pin3
        hal.gpio->write(pin2, 1);
        hal.scheduler->delay(1);
        //if pin3 is 1 increment counter
        if (hal.gpio->read(pin3) == 1)
    	channel3_status++;

        if (channel3_status == 3) {
            _detected = true;
            _rc_type = PPMSUM;
            return true;
        }
    return false;
}


