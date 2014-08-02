/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


#include "RCOutput.h"
#include <boards.h>
#include <pwm_in.h>
#include <timer.h>

extern const AP_HAL::HAL& hal;
using namespace VRBRAIN;

static int outPin[VRBRAIN_MAX_OUTPUT_CHANNELS];


static inline long map(long value, long fromStart, long fromEnd,
                long toStart, long toEnd) {
    return (value - fromStart) * (toEnd - toStart) / (fromEnd - fromStart) +
        toStart;
}


void VRBRAINRCOutput::init(void* implspecific)
{

    if(g_is_ppmsum)
	hal.console->print_P("is PPMSUM");

    out_ch1=48;  //Timer2 ch2
    out_ch2=49;  //Timer2 ch3
    out_ch3=50;  //Timer2 ch4
    out_ch4=36;  //Timer3 ch2
    out_ch5=46;  //Timer3 ch3
    out_ch6=45;  //Timer3 ch4

    if(g_ext_mag_detect){
	    if(g_is_ppmsum == 3) { //if we have SBUS enabled use INPUT1-4 as outputs
		out_ch7=75;  //Timer1 ch1 PWM_IN_1
		out_ch8=80; //Timer1 ch2 PWM_IN_2
		out_ch9=86; //Timer1 ch3 PWM_IN_3
		out_ch10=89; //Timer1 ch4 PWM_IN_4
		out_ch11=200; //disabled
		out_ch12=200;//disabled
	    }else if (g_is_ppmsum == 1) {  //if we have PPMSUM enabled use INPUT5-8 as outputs
		out_ch7=12;  //Timer8 ch1 PWM_IN_5
		out_ch8=13; //Timer8 ch2 PWM_IN_6
		out_ch9=14; //Timer8 ch3 PWM_IN_7
		out_ch10=15; //Timer8 ch4 PWM_IN_8
		out_ch11=200;//disabled
		out_ch12=200;//disabled
	    } else {
		out_ch7=200;//disabled
		out_ch8=200;//disabled
		out_ch9=200;//disabled
		out_ch10=200;//disabled
		out_ch11=200;//disabled
		out_ch12=200;//disabled
	    }
    } else {
	    out_ch7=101; //Timer4 ch3
	    out_ch8=25;  //Timer4 ch4
	    if(g_is_ppmsum == 3) { //if we have SBUS enabled use INPUT1-4 as outputs
		out_ch9=75;  //Timer1 ch1 PWM_IN_1
		out_ch10=80; //Timer1 ch2 PWM_IN_2
		out_ch11=86; //Timer1 ch3 PWM_IN_3
		out_ch12=89; //Timer1 ch4 PWM_IN_4
	    }else if (g_is_ppmsum == 1) {  //if we have PPMSUM enabled use INPUT5-8 as outputs
		out_ch9=12;  //Timer8 ch1 PWM_IN_5
		out_ch10=13; //Timer8 ch2 PWM_IN_6
		out_ch11=14; //Timer8 ch3 PWM_IN_7
		out_ch12=15; //Timer8 ch4 PWM_IN_8
	    }
    }



    outPin[MOTORID1] = out_ch1;
    outPin[MOTORID2] = out_ch2;
    outPin[MOTORID3] = out_ch3;
    outPin[MOTORID4] = out_ch4;
    outPin[MOTORID5] = out_ch5;
    outPin[MOTORID6] = out_ch6;
    _num_motors = 6;

    /*Enable CH1 to CH3 outputs*/
    timerDefaultConfig(TIMER2);

    /*Enable CH4 to CH6 outputs*/
    timerDefaultConfig(TIMER3);

    /*If external mag is detected then switch off TIMER4 to enable I2C on those channels */
    if (g_ext_mag_detect){
	timer_disable(TIMER4);

	if(g_is_ppmsum == 1){
	    //enable timer on the pwm in ch5-8
	   timerDefaultConfig(TIMER8);
	} else if (g_is_ppmsum == 3) {
	    //enable timer on pmn in ch1-4
	   timerDefaultConfig(TIMER1);
	}
	    /*enable 4 outputs if PPMSUM is detected*/
	    outPin[MOTORID7] = out_ch9;
	    outPin[MOTORID8] = out_ch10;
	    outPin[MOTORID9] = out_ch11;
	    outPin[MOTORID10] = out_ch12;

	    _num_motors = 10;

    } else { //no external mag detected so we use ch7 and ch8 as normal outputs

	outPin[MOTORID7] = out_ch7;
	outPin[MOTORID8] = out_ch8;
	/*else enable CH7 and CH8 on TIMER4*/
	timerDefaultConfig(TIMER4);
	_num_motors = 8;

	if(g_is_ppmsum == 1){ //PPMSUM
	    //enable timer on the pwm in ch5-8
	    timerDefaultConfig(TIMER8);
	} else if (g_is_ppmsum == 3) { //SBUS
	    //enable timer on pmn in ch1-4
	    timerDefaultConfig(TIMER1);
	}
	if(g_is_ppmsum > 0) { //PPMSUM or SBUS
	    outPin[MOTORID9] = out_ch9;
	    outPin[MOTORID10] = out_ch10;
	    outPin[MOTORID11] = out_ch11;
	    outPin[MOTORID12] = out_ch12;
	    _num_motors = 12;
	}
    }

    for(int8_t i = MOTORID1; i <= (_num_motors -1); i++) {
	hal.gpio->pinMode(outPin[i],PWM);
    }

}

#define _BV(bit) (1 << (bit))

void VRBRAINRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
    {
    uint32_t icr = _timer_period(freq_hz);

    if ((chmask & ( _BV(CH_1) | _BV(CH_2) | _BV(CH_3))) != 0) {
	TIM2->ARR = icr;
    }

    if ((chmask & ( _BV(CH_4) | _BV(CH_5) | _BV(CH_6))) != 0) {
	TIM3->ARR = icr;
    }
    if(g_ext_mag_detect) {
	if(g_is_ppmsum == 1) {
	    if ((chmask & ( _BV(CH_7) | _BV(CH_8) | _BV(CH_9) | _BV(CH_10))) != 0) {
		TIM8->ARR = icr;
	    }
	}
	if (g_is_ppmsum == 3) {
	    if ((chmask & ( _BV(CH_7) | _BV(CH_8) | _BV(CH_9) | _BV(CH_10))) != 0) {
		TIM1->ARR = icr;
	    }
	}
    } else {
	if ((chmask & ( _BV(CH_7) | _BV(CH_8))) != 0) {
	    TIM4->ARR = icr;
	}
	if (g_is_ppmsum ==1) {
	    if ((chmask & ( _BV(CH_9) | _BV(CH_10) | _BV(CH_11) | _BV(CH_12))) != 0) {
		TIM8->ARR = icr;
	    }
	}
	if (g_is_ppmsum == 3) {
	    if ((chmask & ( _BV(CH_9) | _BV(CH_10) | _BV(CH_11) | _BV(CH_12))) != 0) {
		TIM1->ARR = icr;
	    }
	}
    }

}

uint16_t VRBRAINRCOutput::get_freq(uint8_t ch) {
    uint32_t icr;
    switch (ch) {
        case CH_1:
        case CH_2:
        case CH_3:
            icr =(TIMER2->regs)->ARR;
            break;
        case CH_4:
        case CH_5:
        case CH_6:
            icr = (TIMER3->regs)->ARR;
            break;
        case CH_7:
        case CH_8:
            icr = (TIMER4->regs)->ARR;
            break;
        case CH_9:
        case CH_10:
        case CH_11:
        case CH_12:
            if(g_is_ppmsum == 1) {
        	icr = (TIMER8->regs)->ARR;
            } else if (g_is_ppmsum == 3) {
        	icr = (TIMER1->regs)->ARR;
            }
            break;
        default:
            return 0;
    }


    /* transform to period by inverse of _time_period(icr). */
    return (uint16_t)(2000000UL / icr);
}

void VRBRAINRCOutput::enable_ch(uint8_t ch)
{
    timer_dev *dev;
    switch(ch)
	{
	case 0: (TIMER2->regs)->CCER |= (uint16_t)TIM_CCER_CC2E; break; // CH_1 : OC1B
	case 1: (TIMER2->regs)->CCER |= (uint16_t)TIM_CCER_CC3E; break; // CH_2 : OC1A
	case 2: (TIMER2->regs)->CCER |= (uint16_t)TIM_CCER_CC4E; break; // CH_3 : OC4C
	case 3: (TIMER3->regs)->CCER |= (uint16_t)TIM_CCER_CC2E; break; // CH_4 : OC4B
	case 4: (TIMER3->regs)->CCER |= (uint16_t)TIM_CCER_CC3E; break; // CH_5 : OC4A
	case 5: (TIMER3->regs)->CCER |= (uint16_t)TIM_CCER_CC4E; break; // CH_6 : OC3C
	case 6:
	    if(_num_motors > 6) {
	   		dev = PIN_MAP[out_ch7].timer_device;
	   		if(g_ext_mag_detect){
	   		    (dev->regs)->CCER |= (uint16_t)TIM_CCER_CC1E;
	   		} else {
	   		    (dev->regs)->CCER |= (uint16_t)TIM_CCER_CC3E;
	   		}
	   	    }
	   	    break; // CH_7 : OC3B
	case 7:
	    if(_num_motors > 6) {
		dev = PIN_MAP[out_ch8].timer_device;
		if(g_ext_mag_detect){
		    (dev->regs)->CCER |= (uint16_t)TIM_CCER_CC2E;
		} else {
		    (dev->regs)->CCER |= (uint16_t)TIM_CCER_CC4E;
		}
	    }
	    break; // CH_8 : OC3A
	case 8:
	    if(_num_motors > 8) {
		dev = PIN_MAP[out_ch9].timer_device;
		if(g_ext_mag_detect){
		    (dev->regs)->CCER |= (uint16_t)TIM_CCER_CC3E;
		} else {
		    (dev->regs)->CCER |= (uint16_t)TIM_CCER_CC1E;
		}

	    }
	    break; // CH_9 : OC3B
	case 9:
	    if(_num_motors > 8) {
		dev = PIN_MAP[out_ch10].timer_device;
		if(g_ext_mag_detect){
		    (dev->regs)->CCER |= (uint16_t)TIM_CCER_CC4E;
		} else {
		    (dev->regs)->CCER |= (uint16_t)TIM_CCER_CC2E;
		}

	    }
	    break; // CH_10 : OC3A
	case 10:
	    if(_num_motors > 10) {
		dev = PIN_MAP[out_ch11].timer_device;
		(dev->regs)->CCER |= (uint16_t)TIM_CCER_CC3E;
	    }
	    break; // CH_11 : OC5B
	case 11:
	    if(_num_motors > 10) {
		dev = PIN_MAP[out_ch12].timer_device;
		(dev->regs)->CCER |= (uint16_t)TIM_CCER_CC4E;
	    }
	    break; // CH_12 : OC5C
	}
}

void VRBRAINRCOutput::disable_ch(uint8_t ch)
{
    timer_dev *dev;
    switch(ch)
	{
	case 0: (TIMER2->regs)->CCER &= (uint16_t)~TIM_CCER_CC2E; break; // CH_1 : OC1B
	case 1: (TIMER2->regs)->CCER &= (uint16_t)~TIM_CCER_CC3E; break; // CH_2 : OC1A
	case 2: (TIMER2->regs)->CCER &= (uint16_t)~TIM_CCER_CC4E; break; // CH_3 : OC4C
	case 3: (TIMER3->regs)->CCER &= (uint16_t)~TIM_CCER_CC2E; break; // CH_4 : OC4B
	case 4: (TIMER3->regs)->CCER &= (uint16_t)~TIM_CCER_CC3E; break; // CH_5 : OC4A
	case 5: (TIMER3->regs)->CCER &= (uint16_t)~TIM_CCER_CC4E; break; // CH_6 : OC3C
	case 6:
	    if(_num_motors > 6) {
		dev = PIN_MAP[out_ch7].timer_device;
		if(g_ext_mag_detect){
		    (dev->regs)->CCER &= (uint16_t)~TIM_CCER_CC1E;
		} else {
		    (dev->regs)->CCER &= (uint16_t)~TIM_CCER_CC3E;
		}
	    }
	    break; // CH_7 : OC3B
	case 7:
	    if(_num_motors > 6) {
		dev = PIN_MAP[out_ch8].timer_device;
		if(g_ext_mag_detect){
		    (dev->regs)->CCER &= (uint16_t)~TIM_CCER_CC2E;
		} else {
		    (dev->regs)->CCER &= (uint16_t)~TIM_CCER_CC4E;
		}
	    }
	    break; // CH_8 : OC3A
	case 8:
	    if(_num_motors > 8) {
		dev = PIN_MAP[out_ch9].timer_device;
		if(g_ext_mag_detect){
		    (dev->regs)->CCER &= (uint16_t)~TIM_CCER_CC3E;
		} else {
		    (dev->regs)->CCER &= (uint16_t)~TIM_CCER_CC1E;
		}

	    }
	    break; // CH_9 : OC3B
	case 9:
	    if(_num_motors > 8) {
		dev = PIN_MAP[out_ch10].timer_device;
		if(g_ext_mag_detect){
		    (dev->regs)->CCER &= (uint16_t)~TIM_CCER_CC4E;
		} else {
		    (dev->regs)->CCER &= (uint16_t)~TIM_CCER_CC2E;
		}

	    }
	    break; // CH_10 : OC3A
	case 10:
	    if(_num_motors > 10) {
		dev = PIN_MAP[out_ch11].timer_device;
		(dev->regs)->CCER &= (uint16_t)~TIM_CCER_CC3E;
	    }
	    break; // CH_11 : OC5B
	case 11:
	    if(_num_motors > 10) {
		dev = PIN_MAP[out_ch12].timer_device;
		(dev->regs)->CCER &= (uint16_t)~TIM_CCER_CC4E;
	    }
	    break; // CH_12 : OC5C
	}
}


void VRBRAINRCOutput::set_safety_pwm(uint32_t chmask, uint16_t period_us)
{
    for (uint8_t i=0; i< _num_motors; i++) {
        if ((1UL<<i) & chmask) {
            write(i, period_us);
        }
    }
}
/* constrain pwm to be between min and max pulsewidth. */
static inline uint16_t constrain_period(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

void VRBRAINRCOutput::write(uint8_t ch, uint16_t period_us)
{
    uint16_t pwm = constrain_period(period_us) << 1;


    uint8_t pin = outPin[ch];
    timer_dev *dev = PIN_MAP[pin].timer_device;

    if (pin >= BOARD_NR_GPIO_PINS || dev == NULL || dev->type == TIMER_BASIC)
	{
	return;
	}

    timer_set_compare(dev, PIN_MAP[pin].timer_channel, pwm);
    TIM_Cmd(dev->regs, ENABLE);
}


void VRBRAINRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        write(i + ch, period_us[i]);
    }
}

uint16_t VRBRAINRCOutput::read(uint8_t ch) 
{

    uint16_t pin = outPin[ch];
    timer_dev *dev = PIN_MAP[pin].timer_device;
    if (pin >= BOARD_NR_GPIO_PINS || dev == NULL || dev->type == TIMER_BASIC)
	{
	return RC_INPUT_MIN_PULSEWIDTH;
	}

    uint16_t pwm;
    pwm =     timer_get_compare(dev, PIN_MAP[pin].timer_channel);
    return pwm >> 1;
}

void VRBRAINRCOutput::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

uint32_t VRBRAINRCOutput::_timer_period(uint16_t speed_hz) {
    return (uint32_t)(2000000UL / speed_hz);
}
