#include <exti.h>
#include <timer.h>
#include "RCInput.h"
#include <pwm_in.h>


// Constructors ////////////////////////////////////////////////////////////////
using namespace AP_HAL;
using namespace VRBRAIN;


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
volatile uint16_t VRBRAINRCInput::_pulse_capt[VRBRAIN_RC_INPUT_NUM_CHANNELS] = {0};
volatile uint8_t  VRBRAINRCInput::_valid_channels = 0;

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

/* constrain captured pulse to be between min and max pulsewidth. */
static inline uint16_t constrain_pulse(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

void VRBRAINRCInput::rxIntPPMSUM(uint8_t state, uint16_t value)
    {
    static uint8_t  channel_ctr;

    if (value >= 4000) // Frame synchronization
	{
	    if( channel_ctr >= VRBRAIN_RC_INPUT_MIN_CHANNELS ) {
		_valid_channels = channel_ctr;
	    }
	    channel_ctr = 0;
	}
    else
	{
        if (channel_ctr < VRBRAIN_RC_INPUT_NUM_CHANNELS) {
            _pulse_capt[channel_ctr] = value;
            channel_ctr++;
            if (channel_ctr == VRBRAIN_RC_INPUT_NUM_CHANNELS) {
                _valid_channels = VRBRAIN_RC_INPUT_NUM_CHANNELS;
            }
        }

	}
    }


void VRBRAINRCInput::InitDefaultPPM(char board)
    {
    switch (board)
	{
    case 0:

	// MP32V1F1
	input_channel_ch1 = 22;
	input_channel_ch2 = 23;
	input_channel_ch3 = 24;
	input_channel_ch4 = 89;
	input_channel_ch5 = 59;
	input_channel_ch6 = 62;
	input_channel_ch7 = 60;
	input_channel_ch8 = 0;
	break;
    case 1:

	// MP32V3F1
	input_channel_ch1 = 22;
	input_channel_ch2 = 63;
	input_channel_ch3 = 66;
	input_channel_ch4 = 89;
	input_channel_ch5 = 59;
	input_channel_ch6 = 62;
	input_channel_ch7 = 60;
	input_channel_ch8 = 12;
	/*
	 input_channel_ch7=12;
	 input_channel_ch8=60;
	 */

	break;
    case 2:

	// VRBRAIN
	input_channel_ch1 = 75;
	input_channel_ch2 = 80;
	input_channel_ch3 = 86;
	input_channel_ch4 = 89;
	input_channel_ch5 = 12;
	input_channel_ch6 = 13;
	input_channel_ch7 = 14;
	input_channel_ch8 = 0;

	//input_channel_ch8=15;
	break;

	}
    }


VRBRAINRCInput::VRBRAINRCInput()
    {
    }

void VRBRAINRCInput::init(void* machtnichts)
    {

    /*initial check for pin2-pin3 bridge. If detected switch to PPMSUM  */
    //default to standard PPM
    _iboard = 2;

    uint8_t channel3_status = 0;
    uint8_t pin2, pin3;
    //input pin 2
    pin2 = 80;
    //input pin 3
    pin3 = 86;

    //set pin2 as output and pin 3 as input
    hal.gpio->pinMode(pin2, OUTPUT);
    hal.gpio->pinMode(pin3, INPUT);

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

    //if counter is 3 then we are in PPMSUM
    if (channel3_status == 3)
	_iboard = 11;

    if (_iboard < 10) //PWM
	{
	for (byte channel = 0; channel < 8; channel++)
	    pinData[channel].edge = FALLING_EDGE;
	// Init Radio In
	hal.console->println("Init Default PPM");
	pwmInit(false);
	}
    else //PPMSUM
	{
	// Init Radio In
	hal.console->println("Init Default PPMSUM");
	attachPWMCaptureCallback(rxIntPPMSUM);
	pwmInit(true);
	}

    clear_overrides();
    }

uint8_t VRBRAINRCInput::valid_channels()
    {
    if(_iboard < 10)
	return 1;
    else
	return _valid_channels;

    }

uint16_t VRBRAINRCInput::read(uint8_t ch)
    {
    uint16_t data;
    noInterrupts();
    if (_iboard < 10)
	{
	//data = rcPinValue[ch];
	data = pwmRead(ch);
	}
    else
	{
	data = _pulse_capt[ch];
	}
    interrupts();

    /* Check for override */
    uint16_t over = _override[ch];

    return (over == 0) ? data : over;
    }

uint8_t VRBRAINRCInput::read(uint16_t* periods, uint8_t len)
    {
    noInterrupts();
    for (uint8_t i = 0; i < len; i++)
	{
	    if (_iboard < 10)
		periods[i] = pwmRead(i);
	    else
		periods[i] = _pulse_capt[i];

	    if (_override[i] != 0)
		periods[i] = _override[i];
	}
    interrupts();

    return len;
    }

bool VRBRAINRCInput::set_overrides(int16_t *overrides, uint8_t len)
    {
    bool res = false;
    for (int i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
    }

bool VRBRAINRCInput::set_override(uint8_t channel, int16_t override)
    {
    if (override < 0) return false; /* -1: no change. */
    if (channel < 8) {
        _override[channel] = override;
        if (override != 0) {
            return true;
        }
    }
    return false;
    }

void VRBRAINRCInput::clear_overrides()
    {
    for (int i = 0; i < 8; i++) {
	set_override(i, 0);
    }
    }

