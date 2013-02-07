
#include "RCOutput.h"

extern const AP_HAL::HAL& hal;

static int analogOutPin[20];

static inline long map(long value, long fromStart, long fromEnd,
                long toStart, long toEnd) {
    return (value - fromStart) * (toEnd - toStart) / (fromEnd - fromStart) +
        toStart;
}

using namespace VRBRAIN;
void VRBRAINRCOutput::InitDefaultPWM(void)
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

void VRBRAINRCOutput::init(void* machtnichts)
{
    // Init Pwm Out
    //_serial->println("Init DefaultPWM");

    InitDefaultPWM();
    //_serial->println("Init PWM HWD");

    InitPWM();


}

unsigned short VRBRAINRCOutput::GetTimerReloadValue(unsigned short uFreq){
  unsigned int uiReload;
  if (uFreq>550) uFreq=550;
  if (uFreq<50) uFreq=50;
  uiReload=0xFFFF*50/uFreq;
  return (unsigned short)uiReload;
}


void VRBRAINRCOutput::InitPWM()
{
unsigned int valout=0;

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

	if (analogOutPin[i] > 200)
		{
		valout=analogOutPin[i]-200;
		InitFQUpdate(valout);
		hal.gpio->pinMode(valout,PWM);
/*
		_serial->print("Motor ESC:");
		_serial->print(i);
		_serial->print(":");
		_serial->println(analogOutPin[i]);
*/
		}
		else
		{
		hal.gpio->pinMode(analogOutPin[i],PWM);
/*
		_serial->print("Motor PWM:");
		_serial->print(i);
		_serial->print(":");
		_serial->println(analogOutPin[i]);
*/
		}
	}

}

void VRBRAINRCOutput::InitFQUpdate(unsigned char channel)
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

	timer_select=4;

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


void VRBRAINRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t VRBRAINRCOutput::get_freq(uint8_t ch) {
    return 50;
}

void VRBRAINRCOutput::enable_ch(uint8_t ch)
{}

void VRBRAINRCOutput::enable_mask(uint32_t chmask)
{}

void VRBRAINRCOutput::disable_ch(uint8_t ch)
{}

void VRBRAINRCOutput::disable_mask(uint32_t chmask)
{}

void VRBRAINRCOutput::write(uint8_t ch, uint16_t period_us)
{
	if (analogOutPin[ch]>200)
	{
		period_us = map(period_us, 1000, 2000, 3300, 8000); // PATCH FOR GIMBAL CONTROLL 69 HZ
		analogWrite(analogOutPin[ch]-200, period_us);
	}
	else
	{
		period_us = map(period_us, 1000, 2000, 28000, 57141);	// MP32F4 PWM 490 HZ
		analogWrite(analogOutPin[ch], period_us);
	}

}

void VRBRAINRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{}

uint16_t VRBRAINRCOutput::read(uint8_t ch) {
    return 900;
}

void VRBRAINRCOutput::read(uint16_t* period_us, uint8_t len)
{}

