#include <exti.h>
#include <timer.h>
#include "RCInput.h"
#include <pwm_in.h>

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

// STANDARD PPM VARIABLE

volatile uint16_t rcPinValue[8] =
    {
    1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500
    };
// interval [1000;2000]

// ***PPM SUM SIGNAL***
static uint8_t rcChannel[12];
volatile uint16_t rcValue[20] =
    {
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500
    }; // interval [1000;2000]
volatile uint16_t rcTmpValue[20] =
    {
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500,
	    1500
    }; // interval [1000;2000]

// Variable definition for Input Capture interrupt
//volatile uint16_t APM_RC_MP32::_PWM_RAW[NUM_CHANNELS] = {2400,2400,2400,2400,2400,2400,2400,2400};
//volatile uint8_t APM_RC_MP32::_radio_status=0;

volatile unsigned char radio_status_rc = 0;
volatile unsigned char sync = 0;
volatile unsigned int currentChannel = 0;
static unsigned int last = 0;

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

void rxIntPPMSUM(void)
    {
    volatile unsigned int now;
    volatile unsigned int diff;
    int i;

    //hal.scheduler->suspend_timer_procs();

    now = hal.scheduler->micros();
    diff = now - last;
    last = now;
    if (diff > 4000 && diff < 21000) // Sincro del frame
	{
	currentChannel = 0;
	radio_status_rc = 0;
	if (uiRcErrCnt1 == 0)
	    {		// if the frame is error free, copy it to rcValue Array
	    for (i = 0; i < 9; i++)
		{
		rcValue[i] = rcTmpValue[i]; // THE PPMSUM VALUE START FROM 10 ' STANDARD PPM channel < 10
		}
	    }
	sync = 1;
	uiRcErrCnt1 = 0;	// Reset Error counter
	}
    else if ((diff > 2400) || (diff < 650))
	{// the signal from my jeti receiver goes around 740 to 1550 ms, with <650 or >2000 bad data will be recorded
	uiRcErrCnt1++;
	}
    if (sync == 1)
	{
	//rcValue[currentChannel] = diff;
	rcTmpValue[currentChannel] = diff;
	currentChannel++;
	if (diff <= MAXCHECK && diff >= MINCHECK)
	    radio_status_rc++;
	}
    if (currentChannel > 9)
	{
	//currentChannel=0;
	sync = 0;
	radio_status_rc = 0;
	}

    //hal.scheduler->resume_timer_procs();
    }

/*
 0 PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
 1 PE11		80	PWM_IN1		 IRQ 10-15			  PPM2
 2 PE13		86	PWM_IN2		 IRQ 10-15			  PPM3
 3 PE14		89	PWM_IN3		 IRQ 10-15			  PPM4
 4 PC6		12	PWM_IN4		 IRQ 5-9			  PPM5
 5 PC7		13	PWM_IN5		 IRQ 5-9			  PPM6
 6 PC8		14	PWM_IN6		 IRQ 5-9			  PPM7
 7 PC9		15	PWM_IN7	     IRQ 5-9   * Conflict (PPMSUM)
 */

// Constructors ////////////////////////////////////////////////////////////////
using namespace VRBRAIN;

/* ADD ON PIN NORMALLY AVAILABLE ON RX BUT IF PPM SUM ACTIVE AVAILABLE AS SERVO OUTPUT */

void VRBRAINRCInput::InitDefaultPPMSUM(char board)
    {
    switch (board)
	{
    case 10:
	ppm_sum_channel = 75;
	rcChannel[0] = 0;
	rcChannel[1] = 1;
	rcChannel[2] = 2;
	rcChannel[3] = 3;
	rcChannel[4] = 4;
	rcChannel[5] = 5;
	rcChannel[6] = 6;
	rcChannel[7] = 7;
	rcChannel[8] = 8;
	break;

    case 11:
	ppm_sum_channel = 75;
	rcChannel[0] = 0;
	rcChannel[1] = 1;
	rcChannel[2] = 2;
	rcChannel[3] = 3;
	rcChannel[4] = 4;
	rcChannel[5] = 5;
	rcChannel[6] = 6;
	rcChannel[7] = 7;
	rcChannel[8] = 8;

	break;
	}
    }

void VRBRAINRCInput::InitPPM(void)
    {
    for (byte channel = 0; channel < 8; channel++)
	pinData[channel].edge = FALLING_EDGE;

    //attachPWMCaptureCallback(PWMCaptureCallback);
    pwmInit();
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
	//PIN 13 freeze board (was USB DISC)
	/*
	 PE9		75	PWM_IN0		IRQ 5-9  * Conflict  PPM1
	 PE11		80	PWM_IN1		IRQ 10-15			  PPM2
	 PE13		86	PWM_IN2		IRQ 10-15			  PPM3
	 PE14		89	PWM_IN3		IRQ 10-15			  PPM4
	 PC6		12	PWM_IN4		IRQ 5-9			  PPM5
	 PC7		13	PWM_IN5		IRQ 5-9			  PPM6
	 PC8		14	PWM_IN6		IRQ 5-9			  PPM7
	 PC9		15	PWM_IN7	     	IRQ 5-9   * Conflict (PPMSUM)
	 */

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

// Public Methods //////////////////////////////////////////////////////////////
void VRBRAINRCInput::InitPPMSUM(void)
    {
    hal.gpio->pinMode(ppm_sum_channel, INPUT);
    hal.gpio->attach_interrupt(ppm_sum_channel, rxIntPPMSUM, RISING);
    }

uint16_t VRBRAINRCInput::InputCh(unsigned char ch)
    {
    uint16_t data;
    noInterrupts();
    if (_iboard < 10)
	data = pwmRead(ch);
    else
	{
	data = rcValue[rcChannel[ch + 1]];
	}
    interrupts();
    return data; // We return the value correctly copied when the IRQ's where disabled
    }

unsigned char VRBRAINRCInput::GetState(void)
    {
    return (radio_status_rc);
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

    if (_iboard < 10)
	{
	// Init Radio In
	hal.console->println("Init Default PPM");
	InitPPM();
	}
    else
	{
	// Init Radio In
	hal.console->println("Init Default PPMSUM");
	InitDefaultPPMSUM(_iboard);
	InitPPMSUM();
	}
    }

uint8_t VRBRAINRCInput::valid_channels()
    {
    return 1;
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
	data = rcValue[rcChannel[ch + 1]];
	}
    interrupts();
    return data; // We return the value correctly copied when the IRQ's where disabled
    }

uint8_t VRBRAINRCInput::read(uint16_t* periods, uint8_t len)
    {
    noInterrupts();
    for (uint8_t i = 0; i < len; i++)
	{
	if (_iboard < 10)
	    //periods[i] = rcPinValue[i];
	    periods[i] = pwmRead(i);
	else
	    {
	    periods[i] = rcValue[rcChannel[i + 1]];
	    }
	}
    interrupts();
    return len;
    }

bool VRBRAINRCInput::set_overrides(int16_t *overrides, uint8_t len)
    {
    return true;
    }

bool VRBRAINRCInput::set_override(uint8_t channel, int16_t override)
    {
    return true;
    }

void VRBRAINRCInput::clear_overrides()
    {
    }

