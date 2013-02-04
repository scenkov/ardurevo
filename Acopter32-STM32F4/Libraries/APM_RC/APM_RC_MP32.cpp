/*
	APM_RC_MP32.cpp - Radio Control Library for Ardupilot Mega. Arduino
	Roberto Navoni. virtualrobotix.com DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	RC Input : PPM signal on IC4 pin
	RC Output : 11 Servo outputs (standard 20ms frame)

	Methods:
		Init() : Initialization of interrupts an Timers
		OutpuCh(ch,pwm) : Output value to servos (range : 900-2100us) ch=0..10
		InputCh(ch) : Read a channel input value.  ch=0..7
		GetState() : Returns the state of the input. 1 => New radio frame to process
		             Automatically resets when we call InputCh to read channels
		
*/
#include "APM_RC_MP32.h"

#include <wirish.h>
#include <exti.h>
#include <timer.h>
#include <wirish.h>

#define MINCHECK 900
#define MAXCHECK 2100

// STANDARD PPM VARIABLE

volatile uint16_t rcPinValue[12] = {0,0,1000,0,1500,1500,1500,1000,0,0,0,0};; // interval [1000;2000]
static byte receiverPin[12] = {PPM_IN_CH1,PPM_IN_CH2,PPM_IN_CH3,PPM_IN_CH4,PPM_IN_CH5,PPM_IN_CH6,PPM_IN_CH7,PPM_IN_CH8};

// ***PPM SUM SIGNAL***
	static uint8_t rcChannel[12];
	volatile uint16_t rcValue[20] = {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500}; // interval [1000;2000]
	volatile uint16_t rcTmpValue[20] = {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500}; // interval [1000;2000]

// Variable definition for Input Capture interrupt
volatile uint16_t APM_RC_MP32::_PWM_RAW[NUM_CHANNELS] = {2400,2400,2400,2400,2400,2400,2400,2400};
volatile uint8_t APM_RC_MP32::_radio_status=0;

	volatile unsigned char radio_status_rc=0;
	volatile unsigned char sync=0;
	static int analogOutPin[20];
	volatile unsigned int currentChannel = 0;
	static unsigned int last = 0;

	unsigned int uiRcErrCnt1 = 0;
	unsigned int uiRcErrCnt2 = 0;
	unsigned int uiRcErrCnt3 = 0;

typedef struct {
  byte edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int  lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[9];


// PE5 is PIN69
#define PIN69	69
#define PIN69IN	(*((unsigned long int *) 0x42420214))
#define PIN69OUT	(*((unsigned long int *) 0x42420294))

// PE6 is PIN70
#define PIN70	70
#define PIN70IN	(*((unsigned long int *) 0x42420218))
#define PIN70OUT	(*((unsigned long int *) 0x42420298))

void rxIntPPMSUM(void) {
volatile unsigned int now;
volatile unsigned int diff;
int i;

  now = micros();
  diff = now - last;
  last = now;
if(diff>4000  &&  diff<21000) // Sincro del frame 
      {
      currentChannel = 0;
	  radio_status_rc=0;
	  if (uiRcErrCnt1==0) {					// if the frame is error free, copy it to rcValue Array
		  for (i=0;i<10;i++) {
			  rcValue[i] =rcTmpValue[i]; // THE PPMSUM VALUE START FROM 10 ' STANDARD PPM channel < 10
		  }
	  }
	  sync=1;
	  uiRcErrCnt1=0;	// Reset Error counter
      }
  else 
	 if ((diff>2400) || (diff<650)) {	// the signal from my jeti receiver goes around 740 to 1550 ms, with <650 or >2000 bad data will be recorded
		 uiRcErrCnt1++;
	 }
     if (sync==1) 
     {
          //rcValue[currentChannel] = diff;
		  rcTmpValue[currentChannel] = diff;
          currentChannel++;
		  if(diff<=MAXCHECK && diff>=MINCHECK)radio_status_rc++;
	}
	 if (currentChannel>9)
	 {
		 //currentChannel=0;	
		 sync=0;
		 radio_status_rc=0;
	 }

}

// PE7 is PIN71
#define PIN71	71
#define PIN71IN	(*((unsigned long int *) 0x4242021C))
#define PIN71OUT	(*((unsigned long int *) 0x4242029C))


void rxIntPPM(void) {
uint32_t currentTime;
uint32_t time;
uint8_t pin;
uint32_t mask, pending;

	//byte channel=0;
    pending = EXTI->PR;
    currentTime = micros();

    for (byte channel = 0; channel < 8; channel++) {
      pin = receiverPin[channel];

	  mask = BIT(PIN_MAP[pin].gpio_bit);

	  if (mask & pending){
		EXTI->PR |= mask; // clear pending

		if (digitalRead(pin)){
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
        else {
          time = currentTime - pinData[channel].riseTime;
          pinData[channel].fallTime = currentTime;
          if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[channel].edge == RISING_EDGE)) {
            pinData[channel].lastGoodWidth = time;
            //Serial4.print("0");
			radio_status_rc=channel;
			rcPinValue[channel] = time;
			sync=1;
			pinData[channel].edge = FALLING_EDGE;
          }
        }
      }  
    }
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
static void rxIntPPM5_9(void) {

uint32_t currentTime;
uint32_t time;
uint32_t channel;
uint8_t pin;


   if(EXTI_GetITStatus(EXTI_Line5) != RESET)
   {

       //tGetCnt[1] = TIM2->CNT;
       EXTI_ClearITPendingBit(EXTI_Line5);
   }
   if(EXTI_GetITStatus(EXTI_Line6) != RESET)
      {

	    //============================================================
	    //PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
	   	pin=receiverPin[4];
	   	channel=4;
	   	if (digitalRead(pin)){
							currentTime = micros();
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
      else {
		currentTime = micros();
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
	   	EXTI_ClearITPendingBit(EXTI_Line6);
      }

   if(EXTI_GetITStatus(EXTI_Line7) != RESET)
   {

	    //============================================================
	    //PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
	   	pin=receiverPin[5];
	   	channel=5;
	   	if (digitalRead(pin)){
							currentTime = micros();
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
      else {
		currentTime = micros();
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
	   	if (digitalRead(pin)){
							currentTime = micros();
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
      else {
		currentTime = micros();
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
	   	if (digitalRead(pin)){
							currentTime = micros();
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
       else {
		 currentTime = micros();
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


static void rxIntPPM10_15(void) {
	uint32_t currentTime;
	uint32_t time;
	uint32_t channel;
	uint8_t pin;


	   if(EXTI_GetITStatus(EXTI_Line10) != RESET)
	   {

	       //tGetCnt[1] = TIM2->CNT;
	       //EXTI_ClearITPendingBit(EXTI_Line8);
	   }

	   if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	      {

		    //============================================================
		    //PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
		   	pin=receiverPin[1];
		   	channel=1;
		   	if (digitalRead(pin)){
								currentTime = micros();
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
	      else {
			currentTime = micros();
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
		   	//============exti===================================
		   	EXTI_ClearITPendingBit(EXTI_Line10);
	      }

	   if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	   {
		   EXTI_ClearITPendingBit(EXTI_Line12);
	   }

	   if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	      {

		    //============================================================
		    //PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
		   	pin=receiverPin[2];
		   	channel=2;
		   	if (digitalRead(pin)){
								currentTime = micros();
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
	      else {
			currentTime = micros();
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
		   	EXTI_ClearITPendingBit(EXTI_Line13);
	      }

	   if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	   {

		    //============================================================
		    //PE9		75	PWM_IN0		 IRQ 5-9  * Conflict  PPM1
		   	pin=receiverPin[3];
		   	channel=3;
		   	if (digitalRead(pin)){
								currentTime = micros();
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
	       else {
		     currentTime = micros();
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
		   	EXTI_ClearITPendingBit(EXTI_Line15);
	   }





}

#endif

// Constructors ////////////////////////////////////////////////////////////////

APM_RC_MP32::APM_RC_MP32()
{
}



/* ADD ON PIN NORMALLY AVAILABLE ON RX BUT IF PPM SUM ACTIVE AVAILABLE AS SERVO OUTPUT */
void APM_RC_MP32::InitDefaultPWM(void)
{
	output_channel_ch1=48;
	output_channel_ch2=49;
	output_channel_ch3=50;
	output_channel_ch4=36;
	output_channel_ch5=46;
	output_channel_ch6=45;
	output_channel_ch7=301;
	output_channel_ch8=225;
	/*
	output_channel_ch9=23;
	output_channel_ch10=24;
	output_channel_ch11=89;
	output_channel_ch12=60;
	*/
}

void APM_RC_MP32::Init( char board,Arduino_Mega_ISR_Registry * isr_reg , FastSerial * _serial, bool esc_passthrough )
{
    bool esc_pass = esc_passthrough;
iboard=board;
if (board < 10)
{
// Init Radio In
_serial->println("Init Default PPM");
InitDefaultPPM(board);
//_serial->println("Init PPM HWD");
InitPPM();
}
else
{
// Init Radio In
_serial->println("Init Default PPMSUM");
InitDefaultPPMSUM(board);
//_serial->println("Init PPMSUM HWD");
InitPPMSUM();

}
// Init Pwm Out
_serial->println("Init DefaultPWM");

InitDefaultPWM();
//_serial->println("Init PWM HWD");

InitPWM(_serial, esc_pass);


}

void APM_RC_MP32::InitPWM(FastSerial * _serial, bool esc_passthrough)
{
unsigned int valout=0;
bool esc_pass = esc_passthrough;
  analogOutPin[MOTORID1] = output_channel_ch1; 
  analogOutPin[MOTORID2] = output_channel_ch2; 
  analogOutPin[MOTORID3] = output_channel_ch3; 
  analogOutPin[MOTORID4] = output_channel_ch4; 
  analogOutPin[MOTORID5] = output_channel_ch5; 
  analogOutPin[MOTORID6] = output_channel_ch6; 
  analogOutPin[MOTORID7] = output_channel_ch7; 
  analogOutPin[MOTORID8] = output_channel_ch8; 

  for(int i=MOTORID1;i<MOTORID8+1;i++)
	{
	   	
	if ((analogOutPin[i] > 200) || esc_pass)
		{
		valout= (analogOutPin[i]>200) ? analogOutPin[i]-200 : analogOutPin[i];
		InitFQUpdate(valout, _serial);
		pinMode(valout,PWM);
		
		_serial->print("Servo 50Hz: ");
		_serial->print(i);
		_serial->print(": ");
		_serial->println(analogOutPin[i]);
		
		}
		else
		{
		pinMode(analogOutPin[i],PWM);
		_serial->print("Motor 490Hz: ");
		_serial->print(i);
		_serial->print(": ");
		_serial->println(analogOutPin[i]);
		}
	}
  
}

unsigned short APM_RC_MP32::GetTimerReloadValue(unsigned short uFreq){
  unsigned int uiReload;
  if (uFreq>550) uFreq=550;
  if (uFreq<50) uFreq=50;
  uiReload=0xFFFF*50/uFreq;
  return (unsigned short)uiReload;
}


void APM_RC_MP32::enable_out(uint8_t ch)
{
}


void APM_RC_MP32::disable_out(uint8_t ch)
{
}

void APM_RC_MP32::InitFQUpdate(unsigned char channel, FastSerial * _serial)
{
	unsigned char timer_select = 0;
	unsigned short Reload;
	timer_dev *ccr_select;
	ccr_select = PIN_MAP[channel].timer_device;
	if (ccr_select == TIMER1)
	{
		//_serial->println("Motor SERVO: TIMER 1");
		timer_select=1;
	}
	if (ccr_select == TIMER2)
	{
		//_serial->println("Motor SERVO: TIMER 2");
		timer_select=2;
	}
	if (ccr_select == TIMER3)
	{
		//_serial->println("Motor SERVO: TIMER 3");
		timer_select=3;
	}
	if (ccr_select == TIMER4)
	{
		//_serial->println("Motor SERVO: TIMER 4");
		timer_select=4;
	}
	if (ccr_select == TIMER5)
	{
		//_serial->println("Motor SERVO: TIMER 5");
		timer_select=5;
	}
	if (ccr_select == TIMER8)
	{
		//_serial->println("Motor SERVO: TIMER 8");
		timer_select=8;
	}
	
	//timer_select=4;

	switch (timer_select)
	{
		case 1:
			//_serial->println("Motor ESC: TIMER 1");
			//timer_init(TIMER1);
			timer_set_prescaler(TIMER1, 21);
			Reload=GetTimerReloadValue(MOTOR_PWM_FREQ);
			timer_pause(TIMER1);
			timer_set_count(TIMER1,0);
			timer_set_reload(TIMER1,Reload);
			timer_resume(TIMER1);
			break;
		case 2:
			//_serial->println("Motor ESC: TIMER 2");
			//timer_init(TIMER2);
			timer_set_prescaler(TIMER2, 21);
			Reload=GetTimerReloadValue(MOTOR_PWM_FREQ);
			timer_pause(TIMER2);
			timer_set_count(TIMER2,0);
			timer_set_reload(TIMER2,Reload);
			timer_resume(TIMER2);
			break;
		case 3:
			//_serial->println("Motor ESC: TIMER 3");
			//timer_init(TIMER3);
			timer_set_prescaler(TIMER3, 21);
			Reload=GetTimerReloadValue(MOTOR_PWM_FREQ);
			timer_pause(TIMER3);
			timer_set_count(TIMER3,0);
			timer_set_reload(TIMER3,Reload);
			timer_resume(TIMER3);
			break;
		case 4:
			//_serial->println("Motor ESC: TIMER 4");
			//timer_init(TIMER4);
			timer_set_prescaler(TIMER4, 21);
			Reload=GetTimerReloadValue(MOTOR_PWM_FREQ);
			timer_pause(TIMER4);
			timer_set_count(TIMER4,0);
			timer_set_reload(TIMER4,Reload);
			timer_resume(TIMER4);
			break;
		case 5:
			//_serial->println("Motor ESC: TIMER 5");
			//timer_init(TIMER5);
			timer_set_prescaler(TIMER5, 21);
			Reload=GetTimerReloadValue(MOTOR_PWM_FREQ);
			timer_pause(TIMER5);
			timer_set_count(TIMER5,0);
			timer_set_reload(TIMER5,Reload);
			timer_resume(TIMER5);
			break;
		case 8:
			//_serial->println("Motor ESC: TIMER 8");
			//timer_init(TIMER8);
			timer_set_prescaler(TIMER8, 21);
			Reload=GetTimerReloadValue(MOTOR_PWM_FREQ);
			timer_pause(TIMER8);
			timer_set_count(TIMER8,0);
			timer_set_reload(TIMER8,Reload);
			timer_resume(TIMER8);
			break;
	}

}

void APM_RC_MP32::InitDefaultPPM(char board)
{
switch (board)
{
case 0:

// MP32V1F1
//#define PPM_IN_CH1 22
//#define PPM_IN_CH2 23  // PA8
//#define PPM_IN_CH3 24  //
//#define PPM_IN_CH4 89
//#define PPM_IN_CH5 59
//#define PPM_IN_CH6 62
//#define PPM_IN_CH7 60
//#define PPM_IN_CH8 0
input_channel_ch1=22;
input_channel_ch2=23;
input_channel_ch3=24;
input_channel_ch4=89;
input_channel_ch5=59;
input_channel_ch6=62;
input_channel_ch7=60;
input_channel_ch8=0;
break;
case 1:

// MP32V3F3
//#define PPM_IN_CH1 22
//#define PPM_IN_CH2 63  // PA8
//#define PPM_IN_CH3 66  //
//#define PPM_IN_CH4 89
//#define PPM_IN_CH5 59
//#define PPM_IN_CH6 62
//#define PPM_IN_CH7 60
//#define PPM_IN_CH8 0
input_channel_ch1=22;
input_channel_ch2=63;
input_channel_ch3=66;
input_channel_ch4=89;
input_channel_ch5=59;
input_channel_ch6=62;
input_channel_ch8=12;
input_channel_ch7=60;
/*
input_channel_ch7=12;
input_channel_ch8=60;
*/

break;
case 2:

// MP32V3F3
//#define PPM_IN_CH1 22
//#define PPM_IN_CH2 63  // PA8
//#define PPM_IN_CH3 66  //
//#define PPM_IN_CH4 89
//#define PPM_IN_CH5 59
//#define PPM_IN_CH6 62
//#define PPM_IN_CH7 60
//#define PPM_IN_CH8 0

	//input_channel_ch1=12;
	//input_channel_ch2=13;
	//input_channel_ch3=14;
	//input_channel_ch4=15;
    //PIN 13 freeze board (was USB DISC)
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

input_channel_ch1=75;
input_channel_ch2=80;
input_channel_ch3=86;
input_channel_ch4=89;
input_channel_ch5=12;
input_channel_ch6=13;
input_channel_ch7=14;
input_channel_ch8=0;


//input_channel_ch5=12;
//input_channel_ch6=13;
//input_channel_ch7=14;
//input_channel_ch8=15;
break;


}
}

void APM_RC_MP32::InitDefaultPPMSUM(char board)
{
switch (board)
{
case 10:
ppm_sum_channel=75;
rcChannel[0]=0;
rcChannel[1]=1;
rcChannel[2]=2;
rcChannel[3]=3;
rcChannel[4]=4;
rcChannel[5]=5;
rcChannel[6]=6;
rcChannel[7]=7;
break;

case 11:
ppm_sum_channel=75;
rcChannel[0]=0;
rcChannel[1]=1;
rcChannel[2]=2;
rcChannel[3]=3;
rcChannel[4]=4;
rcChannel[5]=5;
rcChannel[6]=6;
rcChannel[7]=7;


break;
}
}



void APM_RC_MP32::InitPPM(void)
{

receiverPin[0]=input_channel_ch1;
receiverPin[1]=input_channel_ch2;
receiverPin[2]=input_channel_ch3;
receiverPin[3]=input_channel_ch4;
receiverPin[4]=input_channel_ch5;
receiverPin[5]=input_channel_ch6;
receiverPin[6]=input_channel_ch7;
receiverPin[7]=input_channel_ch8;
receiverPin[8]=input_channel_ch9;
receiverPin[9]=input_channel_ch10;
receiverPin[10]=input_channel_ch11;
receiverPin[11]=input_channel_ch12;
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
attachInterrupt(input_channel_ch1, rxIntPPM5_9, CHANGE);
//pinMode(input_channel_ch1, INPUT);
delay(100);
}

if (input_channel_ch2 != 0)
{
attachInterrupt(input_channel_ch2, rxIntPPM10_15, CHANGE);
//pinMode(input_channel_ch2, INPUT);
delay(100);
}


if (input_channel_ch3 != 0)
{
attachInterrupt(input_channel_ch3, rxIntPPM10_15, CHANGE);
//pinMode(input_channel_ch3, INPUT);
delay(100);
}
if (input_channel_ch4 != 0)
{
attachInterrupt(input_channel_ch4, rxIntPPM10_15, CHANGE);
//pinMode(input_channel_ch4, INPUT);
delay(100);
}

if (input_channel_ch5 != 0)
{
attachInterrupt(input_channel_ch5, rxIntPPM5_9, CHANGE);
//pinMode(input_channel_ch5, INPUT);
delay(100);
}


if (input_channel_ch6 != 0)
{
attachInterrupt(input_channel_ch6, rxIntPPM5_9, CHANGE);
//pinMode(input_channel_ch6, INPUT);
delay(100);
}


if (input_channel_ch7 != 0)
{
attachInterrupt(input_channel_ch7, rxIntPPM5_9, CHANGE);
//pinMode(input_channel_ch7, INPUT);
delay(100);
}


if (input_channel_ch8 != 0)
{
attachInterrupt(input_channel_ch8, rxIntPPM, CHANGE);
pinMode(input_channel_ch8, INPUT);
delay(100);
}
#else
if (input_channel_ch1 != 0)
{
attachInterrupt(input_channel_ch1, rxIntPPM, CHANGE);
pinMode(input_channel_ch1, INPUT);
delay(100);
}

if (input_channel_ch2 != 0)
{
attachInterrupt(input_channel_ch2, rxIntPPM, CHANGE);
pinMode(input_channel_ch2, INPUT);
delay(100);
}


if (input_channel_ch3 != 0)
{
attachInterrupt(input_channel_ch3, rxIntPPM, CHANGE);
pinMode(input_channel_ch3, INPUT);
delay(100);
}
if (input_channel_ch4 != 0)
{
attachInterrupt(input_channel_ch4, rxIntPPM, CHANGE);
pinMode(input_channel_ch4, INPUT);
delay(100);
}

if (input_channel_ch5 != 0)
{
attachInterrupt(input_channel_ch5, rxIntPPM, CHANGE);
pinMode(input_channel_ch5, INPUT);
delay(100);
}


if (input_channel_ch6 != 0)
{
attachInterrupt(input_channel_ch6, rxIntPPM, CHANGE);
pinMode(input_channel_ch6, INPUT);
delay(100);
}


if (input_channel_ch7 != 0)
{
attachInterrupt(input_channel_ch7, rxIntPPM, CHANGE);
pinMode(input_channel_ch7, INPUT);
delay(100);
}


if (input_channel_ch8 != 0)
{
attachInterrupt(input_channel_ch8, rxIntPPM, CHANGE);
pinMode(input_channel_ch8, INPUT);
delay(100);
}

#endif

for(byte channel = 0; channel < 11; channel++) 
       pinData[receiverPin[channel]].edge = FALLING_EDGE;


}



// Public Methods //////////////////////////////////////////////////////////////
void APM_RC_MP32::InitPPMSUM(void)
{
pinMode(ppm_sum_channel,INPUT);
attachInterrupt(ppm_sum_channel, rxIntPPMSUM, RISING);
}


void APM_RC_MP32::OutputCh(unsigned char ch, uint16_t pwm)
{
	
	if (analogOutPin[ch]>200)
	{
		pwm = map(pwm, 1000, 2000, 3300, 8000); // PATCH FOR GIMBAL CONTROLL 69 HZ
		analogWrite(analogOutPin[ch]-200, pwm);
	}
	else
	{
		pwm = map(pwm, 1000, 2000, 28000, 57141);	// MP32F4 PWM 490 HZ 
		analogWrite(analogOutPin[ch], pwm);
	}
}
uint16_t APM_RC_MP32::OutputCh_current(uint8_t ch)
{
	return analogOutPin[ch];

}


void APM_RC_MP32::Force_Out(void) { }
//void APM_RC_MP32::Force_Out0_Out1(void) { }
//void APM_RC_MP32::Force_Out2_Out3(void) { }
//void APM_RC_MP32::Force_Out6_Out7(void) { }


void APM_RC_MP32::SetFastOutputChannels(uint32_t chmask, uint16_t speed_hz)
{
}
uint16_t APM_RC_MP32::InputCh(unsigned char ch)
{
  uint16_t data;
  if (iboard <10)
  data = rcPinValue[ch]; 
  else
{  
  data = rcValue[rcChannel[ch+1]]; 
}
  return data; // We return the value correctly copied when the IRQ's where disabled
}

unsigned char APM_RC_MP32::GetState(void)
{
  return(radio_status_rc);
}

// InstantPWM implementation
// This function forces the PWM output (reset PWM) on Out0 and Out1 (Timer5). For quadcopters use
void APM_RC_MP32::Force_Out0_Out1(void)
{
  //if (TCNT5>5000)  // We take care that there are not a pulse in the output
  //  TCNT5=39990;   // This forces the PWM output to reset in 5us (10 counts of 0.5us). The counter resets at 40000
}
// This function forces the PWM output (reset PWM) on Out2 and Out3 (Timer1). For quadcopters use
void APM_RC_MP32::Force_Out2_Out3(void)
{
  //if (TCNT1>5000)
  //  TCNT1=39990;
}
// This function forces the PWM output (reset PWM) on Out6 and Out7 (Timer3). For quadcopters use
void APM_RC_MP32::Force_Out6_Out7(void)
{
  //if (TCNT3>5000)
  //  TCNT3=39990;
}

// allow HIL override of RC values
// A value of -1 means no change
// A value of 0 means no override, use the real RC values
bool APM_RC_MP32::setHIL(int16_t v[NUM_CHANNELS])
{
	uint8_t sum = 0;
	for (unsigned char i=0; i<NUM_CHANNELS; i++) {
		if (v[i] != -1) {
			_HIL_override[i] = v[i];
		}
		if (_HIL_override[i] != 0) {
			sum++;
		}
	}
	radio_status_rc = 1;
	if (sum == 0) {
		return 0;
	} else {
		return 1;
	}
}

void APM_RC_MP32::clearOverride(void)
{
	for (unsigned char i=0; i<NUM_CHANNELS; i++) {
		_HIL_override[i] = 0;
	}
}


// make one instance for the user to use
//APM_RC_Class APM_RC;
