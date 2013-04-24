#include <exti.h>
#include <timer.h>
#include "RCInput.h"

extern const AP_HAL::HAL& hal;

// MP32F4
#define PPM_IN_CH1 22
#define PPM_IN_CH2 63  // PA8
#define PPM_IN_CH3 66  //
#define PPM_IN_CH4 89
#define PPM_IN_CH5 59
#define PPM_IN_CH6 62
#define PPM_IN_CH7 0 //57
#define PPM_IN_CH8 60

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075

#define MINCHECK 900
#define MAXCHECK 2100

// PATCH FOR FAILSAFE AND FRSKY
#define MINOFFWIDTH 1000
#define MAXOFFWIDTH 30000

// STANDARD PPM VARIABLE

volatile uint16_t rcPinValue[12] =
    {
    0, 0, 1000, 0, 1500, 1500, 1500, 1000, 0, 0, 0, 0
    };
// interval [1000;2000]
static byte receiverPin[8] =
    {
	    PPM_IN_CH1,
	    PPM_IN_CH2,
	    PPM_IN_CH3,
	    PPM_IN_CH4,
	    PPM_IN_CH5,
	    PPM_IN_CH6,
	    PPM_IN_CH7,
	    PPM_IN_CH8
    };

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
static int analogOutPin[20];
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

// PE5 is PIN69
#define PIN69	69
#define PIN69IN	(*((unsigned long int *) 0x42420214))
#define PIN69OUT	(*((unsigned long int *) 0x42420294))

// PE6 is PIN70
#define PIN70	70
#define PIN70IN	(*((unsigned long int *) 0x42420218))
#define PIN70OUT	(*((unsigned long int *) 0x42420298))


static void PWMCaptureCallback(uint8_t channel, uint8_t state, int16_t value)
{
	uint32_t currentTime;
	uint32_t time;

	//state 0: rise; state 1: fall
	if (state == 0)
	{
		currentTime = hal.scheduler->micros();
		time = currentTime - pinData[channel].fallTime;
		pinData[channel].riseTime = currentTime;
		if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
			pinData[channel].edge = RISING_EDGE;
		else
		{
			pinData[channel].edge = FALLING_EDGE; // invalid rising edge detected
			radio_status_rc = 0;
			sync = 0;
		}
	}
	else
	{
		currentTime = hal.scheduler->micros();
		time = currentTime - pinData[channel].riseTime;
		pinData[channel].fallTime = currentTime;
		if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[channel].edge == RISING_EDGE))
		{
			pinData[channel].lastGoodWidth = time;
			radio_status_rc = channel;
			rcPinValue[channel] = time;
			sync = 1;
			pinData[channel].edge = FALLING_EDGE;
		}
	}
}

void rxIntPPMSUM(void)
    {
    volatile unsigned int now;
    volatile unsigned int diff;
    int i;

    hal.scheduler->suspend_timer_procs();

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

    hal.scheduler->resume_timer_procs();
    }

// PE7 is PIN71
#define PIN71	71
#define PIN71IN	(*((unsigned long int *) 0x4242021C))
#define PIN71OUT	(*((unsigned long int *) 0x4242029C))

void rxIntPPM(void)
    {
    uint32_t currentTime;
    uint32_t time;
    uint8_t pin;
    uint32_t mask, pending;

    hal.scheduler->suspend_timer_procs();

    //byte channel=0;
    pending = EXTI ->PR;
    currentTime = hal.scheduler->micros();

    for (byte channel = 0; channel < 8; channel++)
	{
	pin = receiverPin[channel];

	mask = BIT(PIN_MAP[pin].gpio_bit);

	if (mask & pending)
	    {
	    EXTI ->PR |= mask; // clear pending

	    if (hal.gpio->read(pin))
		{
		time = currentTime - pinData[channel].fallTime;
		pinData[channel].riseTime = currentTime;
		//Serial4.print("1");
		if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
		    pinData[channel].edge = RISING_EDGE;
		else
		    {
		    pinData[channel].edge = FALLING_EDGE; // invalid rising edge detected
		    radio_status_rc = 0;
		    sync = 0;
		    //  Serial4.print("-");
		    }
		}
	    else
		{
		time = currentTime - pinData[channel].riseTime;
		pinData[channel].fallTime = currentTime;
		if ((time >= MINONWIDTH) && (time <= MAXONWIDTH)
			&& (pinData[channel].edge == RISING_EDGE))
		    {
		    pinData[channel].lastGoodWidth = time;
		    //Serial4.print("0");
		    radio_status_rc = channel;
		    rcPinValue[channel] = time;
		    sync = 1;
		    pinData[channel].edge = FALLING_EDGE;
		    }
		}
	    }
	}
    hal.scheduler->resume_timer_procs();
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
//#define NEWEXTI

#ifdef NEWEXTI
static void rxIntPPM5_9(void)
    {

    uint32_t currentTime;
    uint32_t time;
    uint32_t channel;
    uint8_t pin;

    hal.scheduler->suspend_timer_procs();

    if (EXTI_GetITStatus(EXTI_Line5) != RESET)
	{

	//tGetCnt[1] = TIM2->CNT;
	EXTI_ClearITPendingBit (EXTI_Line5);
	}
    if (EXTI_GetITStatus(EXTI_Line6) != RESET)
	{

	//============================================================
	//PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
	pin = receiverPin[4];
	channel = 4;
	if (hal.gpio->read(pin))
		    {
	    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].fallTime;
		    pinData[channel].riseTime = currentTime;
		    //Serial4.print("1");
		    if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
			pinData[channel].edge = RISING_EDGE;
		    else
			{
			pinData[channel].edge = FALLING_EDGE; // invalid rising edge detected
			radio_status_rc = 0;
			sync = 0;
			//  Serial4.print("-");
			}
		    }
		else
		    {
		    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].riseTime;
		    pinData[channel].fallTime = currentTime;
		    if ((time >= MINONWIDTH) && (time <= MAXONWIDTH)
			    && (pinData[channel].edge == RISING_EDGE))
			{
			pinData[channel].lastGoodWidth = time;
			//Serial4.print("0");
			radio_status_rc = channel;
			rcPinValue[channel] = time;
			sync = 1;
			pinData[channel].edge = FALLING_EDGE;
			}
		    }
		//===============================================
		EXTI_ClearITPendingBit (EXTI_Line6);
		}

	    if(EXTI_GetITStatus(EXTI_Line7) != RESET)
		{

		//============================================================
		//PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
		pin=receiverPin[5];
		channel=5;
		if (hal.gpio->read(pin))
		    {
		    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].fallTime;
		    pinData[channel].riseTime = currentTime;
		    //Serial4.print("1");
		    if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
		    pinData[channel].edge = RISING_EDGE;
		    else
			{
			pinData[channel].edge = FALLING_EDGE; // invalid rising edge detected
			radio_status_rc=0;
			sync=0;
			//  Serial4.print("-");
			}
		    }
		else
		    {
		    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].riseTime;
		    pinData[channel].fallTime = currentTime;
		    if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[channel].edge == RISING_EDGE))
			{
			pinData[channel].lastGoodWidth = time;
			//Serial4.print("0");
			radio_status_rc=channel;
			rcPinValue[channel] = time;
			sync=1;
			pinData[channel].edge = FALLING_EDGE;
			}
		    }
		//===============================================
		EXTI_ClearITPendingBit(EXTI_Line7);
		}

	    if(EXTI_GetITStatus(EXTI_Line8) != RESET)
		{

		//============================================================
		//PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
		pin=receiverPin[6];
		channel=6;
		if (hal.gpio->read(pin))
		    {
		    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].fallTime;
		    pinData[channel].riseTime = currentTime;
		    //Serial4.print("1");
		    if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
		    pinData[channel].edge = RISING_EDGE;
		    else
			{
			pinData[channel].edge = FALLING_EDGE; // invalid rising edge detected
			radio_status_rc=0;
			sync=0;
			//  Serial4.print("-");
			}
		    }
		else
		    {
		    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].riseTime;
		    pinData[channel].fallTime = currentTime;
		    if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[channel].edge == RISING_EDGE))
			{
			pinData[channel].lastGoodWidth = time;
			//Serial4.print("0");
			radio_status_rc=channel;
			rcPinValue[channel] = time;
			sync=1;
			pinData[channel].edge = FALLING_EDGE;
			}

		    }
		//===============================================
		EXTI_ClearITPendingBit(EXTI_Line8);
		}

	    if(EXTI_GetITStatus(EXTI_Line9) != RESET)
		{

		//============================================================
		//PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
		pin=receiverPin[0];
		channel=0;
		if (hal.gpio->read(pin))
		    {
		    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].fallTime;
		    pinData[channel].riseTime = currentTime;
		    //Serial4.print("1");
		    if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
		    pinData[channel].edge = RISING_EDGE;
		    else
			{
			pinData[channel].edge = FALLING_EDGE; // invalid rising edge detected
			radio_status_rc=0;
			sync=0;
			//  Serial4.print("-");
			}
		    }
		else
		    {
		    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].riseTime;
		    pinData[channel].fallTime = currentTime;
		    if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[channel].edge == RISING_EDGE))
			{
			pinData[channel].lastGoodWidth = time;
			//Serial4.print("0");
			radio_status_rc=channel;
			rcPinValue[channel] = time;
			sync=1;
			pinData[channel].edge = FALLING_EDGE;
			}
		    }
		//===============================================
		EXTI_ClearITPendingBit(EXTI_Line9);
		}
	    hal.scheduler->resume_timer_procs();
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

static void rxIntPPM10_15(void)
	    {
	    uint32_t currentTime;
	    uint32_t time;
	    uint32_t channel;
	    uint8_t pin;

	    hal.scheduler->suspend_timer_procs();

	    if (EXTI_GetITStatus(EXTI_Line10) != RESET)
		{

		//tGetCnt[1] = TIM2->CNT;
		//EXTI_ClearITPendingBit(EXTI_Line8);
		}

	    if (EXTI_GetITStatus(EXTI_Line11) != RESET)
		{

		//============================================================
		//PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
		pin = receiverPin[1];
		channel = 1;
		if (hal.gpio->read(pin))
		    {
		    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].fallTime;
		    pinData[channel].riseTime = currentTime;
		    //Serial4.print("1");
		    if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
			pinData[channel].edge = RISING_EDGE;
		    else
			{
			pinData[channel].edge = FALLING_EDGE; // invalid rising edge detected
			radio_status_rc = 0;
			sync = 0;
			//  Serial4.print("-");
			}
		    }
		else
		    {
		    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].riseTime;
		    pinData[channel].fallTime = currentTime;
		    if ((time >= MINONWIDTH) && (time <= MAXONWIDTH)
			    && (pinData[channel].edge == RISING_EDGE))
			{
			pinData[channel].lastGoodWidth = time;
			//Serial4.print("0");
			radio_status_rc = channel;
			rcPinValue[channel] = time;
			sync = 1;
			pinData[channel].edge = FALLING_EDGE;
			}
		    }
		//============exti===================================
		EXTI_ClearITPendingBit (EXTI_Line10);
		}

	    if (EXTI_GetITStatus(EXTI_Line12) != RESET)
		{
		EXTI_ClearITPendingBit (EXTI_Line12);
		}

	    if (EXTI_GetITStatus(EXTI_Line13) != RESET)
		{

		//============================================================
		//PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
		pin = receiverPin[2];
		channel = 2;
		if (hal.gpio->read(pin))
		    {
		    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].fallTime;
		    pinData[channel].riseTime = currentTime;
		    //Serial4.print("1");
		    if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
			pinData[channel].edge = RISING_EDGE;
		    else
			{
			pinData[channel].edge = FALLING_EDGE; // invalid rising edge detected
			radio_status_rc = 0;
			sync = 0;
			//  Serial4.print("-");
			}
		    }
		else
		    {
		    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].riseTime;
		    pinData[channel].fallTime = currentTime;
		    if ((time >= MINONWIDTH) && (time <= MAXONWIDTH)
			    && (pinData[channel].edge == RISING_EDGE))
			{
			pinData[channel].lastGoodWidth = time;
			//Serial4.print("0");
			radio_status_rc = channel;
			rcPinValue[channel] = time;
			sync = 1;
			pinData[channel].edge = FALLING_EDGE;
			}
		    }
		//===============================================
		EXTI_ClearITPendingBit (EXTI_Line13);
		}

	    if (EXTI_GetITStatus(EXTI_Line14) != RESET)
		{

		//============================================================
		//PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
		pin = receiverPin[3];
		channel = 3;
		if (hal.gpio->read(pin))
		    {
		    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].fallTime;
		    pinData[channel].riseTime = currentTime;
		    //Serial4.print("1");
		    if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
			pinData[channel].edge = RISING_EDGE;
		    else
			{
			pinData[channel].edge = FALLING_EDGE; // invalid rising edge detected
			radio_status_rc = 0;
			sync = 0;
			//  Serial4.print("-");
			}
		    }
		else
		    {
		    currentTime = hal.scheduler->micros();
		    time = currentTime - pinData[channel].riseTime;
		    pinData[channel].fallTime = currentTime;
		    if ((time >= MINONWIDTH) && (time <= MAXONWIDTH)
			    && (pinData[channel].edge == RISING_EDGE))
			{
			pinData[channel].lastGoodWidth = time;
			//Serial4.print("0");
			radio_status_rc = channel;
			rcPinValue[channel] = time;
			sync = 1;
			pinData[channel].edge = FALLING_EDGE;
			}
		    }
		//===============================================
		EXTI_ClearITPendingBit (EXTI_Line15);
		}
	    hal.scheduler->resume_timer_procs();
}

#endif

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

	    receiverPin[0] = input_channel_ch1;
	    receiverPin[1] = input_channel_ch2;
	    receiverPin[2] = input_channel_ch3;
	    receiverPin[3] = input_channel_ch4;
	    receiverPin[4] = input_channel_ch5;
	    receiverPin[5] = input_channel_ch6;
	    receiverPin[6] = input_channel_ch7;
	    receiverPin[7] = input_channel_ch8;
	    /*
	    receiverPin[8] = input_channel_ch9;
	    receiverPin[9] = input_channel_ch10;
	    receiverPin[10] = input_channel_ch11;
	    receiverPin[11] = input_channel_ch12;
	    */
		/*
	     PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
	     PE11	80	PWM_IN1		 IRQ 10-15			  PPM2
	     PE13	86	PWM_IN2		 IRQ 10-15			  PPM3
	     PE14	89	PWM_IN3		 IRQ 10-15			  PPM4
	     PC6		12	PWM_IN4		 IRQ 5-9			  PPM5
	     PC7		13	PWM_IN5		 IRQ 5-9			  PPM6
	     PC8		14	PWM_IN6		 IRQ 5-9			  PPM7
	     PC9		15	PWM_IN7	     IRQ 5-9   * Conflict (PPMSUM)
	     */

#ifdef NEWEXTI

	    if (input_channel_ch1 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch1, rxIntPPM5_9, CHANGE);
		hal.gpio->pinMode(input_channel_ch1, INPUT);
		hal.scheduler->delay(100);
		}

	    if (input_channel_ch2 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch2, rxIntPPM10_15, CHANGE);
		hal.gpio->pinMode(input_channel_ch2, INPUT);
		hal.scheduler->delay(100);
		}

	    if (input_channel_ch3 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch3, rxIntPPM10_15, CHANGE);
		hal.gpio->pinMode(input_channel_ch3, INPUT);
		hal.scheduler->delay(100);
		}
	    if (input_channel_ch4 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch4, rxIntPPM10_15, CHANGE);
		hal.gpio->pinMode(input_channel_ch4, INPUT);
		hal.scheduler->delay(100);
		}

	    if (input_channel_ch5 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch5, rxIntPPM5_9, CHANGE);
		hal.gpio->pinMode(input_channel_ch5, INPUT);
		hal.scheduler->delay(100);
		}

	    if (input_channel_ch6 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch6, rxIntPPM5_9, CHANGE);
		hal.gpio->pinMode(input_channel_ch6, INPUT);
		hal.scheduler->delay(100);
		}

	    if (input_channel_ch7 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch7, rxIntPPM5_9, CHANGE);
		hal.gpio->pinMode(input_channel_ch7, INPUT);
		hal.scheduler->delay(100);
		}

	    if (input_channel_ch8 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch8, rxIntPPM, CHANGE);
		hal.gpio->pinMode(input_channel_ch8, INPUT);
		hal.scheduler->delay(100);
		}
#else
	    if (input_channel_ch1 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch1, rxIntPPM, CHANGE);
		hal.gpio->pinMode(input_channel_ch1, INPUT);
		hal.scheduler->delay(100);
		}

	    if (input_channel_ch2 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch2, rxIntPPM, CHANGE);
		hal.gpio->pinMode(input_channel_ch2, INPUT);
		hal.scheduler->delay(100);
		}

	    if (input_channel_ch3 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch3, rxIntPPM, CHANGE);
		hal.gpio->pinMode(input_channel_ch3, INPUT);
		hal.scheduler->delay(100);
		}
	    if (input_channel_ch4 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch4, rxIntPPM, CHANGE);
		hal.gpio->pinMode(input_channel_ch4, INPUT);
		hal.scheduler->delay(100);
		}

	    if (input_channel_ch5 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch5, rxIntPPM, CHANGE);
		hal.gpio->pinMode(input_channel_ch5, INPUT);
		hal.scheduler->delay(100);
		}

	    if (input_channel_ch6 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch6, rxIntPPM, CHANGE);
		hal.gpio->pinMode(input_channel_ch6, INPUT);
		hal.scheduler->delay(100);
		}

	    if (input_channel_ch7 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch7, rxIntPPM, CHANGE);
		hal.gpio->pinMode(input_channel_ch7, INPUT);
		hal.scheduler->delay(100);
		}

	    if (input_channel_ch8 != 0)
		{
		hal.gpio->attach_interrupt(input_channel_ch8, rxIntPPM, CHANGE);
		hal.gpio->pinMode(input_channel_ch8, INPUT);
		hal.scheduler->delay(100);
		}

#endif

	    for (byte channel = 0; channel < 8; channel++)
		{
		if(receiverPin[channel] != 0)
		pinData[receiverPin[channel]].edge = FALLING_EDGE;
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

		// MP32V3F3
		input_channel_ch1 = 22;
		input_channel_ch2 = 63;
		input_channel_ch3 = 66;
		input_channel_ch4 = 89;
		input_channel_ch5 = 59;
		input_channel_ch6 = 62;
		input_channel_ch8 = 12;
		input_channel_ch7 = 60;
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
	    if (_iboard < 10)
		data = rcPinValue[ch];
	    else
		{
		data = rcValue[rcChannel[ch + 1]];
		}
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
	    _iboard = 2;
	    if (_iboard < 10)
		{
		// Init Radio In
		hal.console->println("Init Default PPM");
		InitDefaultPPM(_iboard);
		hal.console->println("Init PPM HWD");
		InitPPM();
		}
	    else
		{
		// Init Radio In
		hal.console->println("Init Default PPMSUM");
		InitDefaultPPMSUM(_iboard);
		hal.console->println("Init PPMSUM HWD");
		InitPPMSUM();

		}

	    }

uint8_t VRBRAINRCInput::valid()
	    {
	    return 1;
	    }

uint16_t VRBRAINRCInput::read(uint8_t ch)
	    {
	    uint16_t data;
	    noInterrupts();
	    if (_iboard < 10){
		data = rcPinValue[ch];
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
		    periods[i] = rcPinValue[i];
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

