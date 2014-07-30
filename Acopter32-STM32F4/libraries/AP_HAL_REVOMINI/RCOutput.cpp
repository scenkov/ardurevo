/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


#include "RCOutput.h"
#include <boards.h>
#include <pwm_in.h>
#include <timer.h>

extern const AP_HAL::HAL& hal;
using namespace REVOMINI;

static int outPin[REVOMINI_MAX_OUTPUT_CHANNELS];


static inline long map(long value, long fromStart, long fromEnd,
                long toStart, long toEnd) {
    return (value - fromStart) * (toEnd - toStart) / (fromEnd - fromStart) +
        toStart;
}


void REVOMINIRCOutput::init(void* implspecific)
{

    if(g_is_ppmsum)
	hal.console->print_P("is PPMSUM");

    out_ch1=46; //Timer3/3
    out_ch2=45; //Timer3/4
    out_ch3=50; //Timer9/2
    out_ch4=49; //Timer2/3
    out_ch5=48; //Timer5/2
    out_ch6=47; //Timer5/1




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

    timerDefaultConfig(TIMER5);

    timerDefaultConfig(TIMER9);


    for(int8_t i = MOTORID1; i <= (_num_motors -1); i++) {
	hal.gpio->pinMode(outPin[i],PWM);
    }

}

#define _BV(bit) (1 << (bit))

void REVOMINIRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
    {
    uint32_t icr = _timer_period(freq_hz);

    if ((chmask & ( _BV(CH_1) | _BV(CH_2) )) != 0) {
	TIM3->ARR = icr;
    }

    if ((chmask & ( _BV(CH_3))) != 0) {
	TIM9->ARR = icr;
    }

    if ((chmask & ( _BV(CH_4))) != 0) {
	TIM2->ARR = icr;
    }

    if ((chmask & ( _BV(CH_5) | _BV(CH_6))) != 0) {
	TIM5->ARR = icr;
    }
}

uint16_t REVOMINIRCOutput::get_freq(uint8_t ch) {
    uint32_t icr;
    switch (ch) {
    case CH_1:
    case CH_2:
        icr = (TIMER3->regs)->ARR;
        break;
    case CH_3:
        icr = (TIMER9->regs)->ARR;
        break;
    case CH_4:
        icr = (TIMER2->regs)->ARR;
        break;
    case CH_5:
    case CH_6:
        icr = (TIMER5->regs)->ARR;
        break;
    default:
        return 0;
    }
    /* transform to period by inverse of _time_period(icr). */
    return (uint16_t)(2000000UL / icr);
}

void REVOMINIRCOutput::enable_ch(uint8_t ch)
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

	}
}

void REVOMINIRCOutput::disable_ch(uint8_t ch)
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
	}
}


void REVOMINIRCOutput::set_safety_pwm(uint32_t chmask, uint16_t period_us)
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

void REVOMINIRCOutput::write(uint8_t ch, uint16_t period_us)
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


void REVOMINIRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        write(i + ch, period_us[i]);
    }
}

uint16_t REVOMINIRCOutput::read(uint8_t ch)
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

void REVOMINIRCOutput::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

uint32_t REVOMINIRCOutput::_timer_period(uint16_t speed_hz) {
    return (uint32_t)(2000000UL / speed_hz);
}
