/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 *  @brief Arduino-style PWM implementation.
 */

#include "pwm_in.h"

#include "hal_types.h"
#include "timer.h"

#include <boards.h>

#define MINONWIDTH 920 * 2
#define MAXONWIDTH 2120 * 2
// PATCH FOR FAILSAFE AND FRSKY
#define MINOFFWIDTH 1000 * 2
#define MAXOFFWIDTH 22000 * 2

typedef void (*rcc_clockcmd)(uint32_t, FunctionalState);
/**************** PWM INPUT **************************************/

// Forward declaration
static inline void pwmIRQHandler(TIM_TypeDef *tim);
static void (*pwm_capture_callback)(uint8_t, uint8_t, int16_t);

// local vars
static struct TIM_Channel {
    TIM_TypeDef * tim;
    uint32_t      tim_clk;
    rcc_clockcmd  tim_clkcmd;
    IRQn_Type     tim_irq;

    uint16_t 	  tim_channel;
    uint16_t 	  tim_cc;

    GPIO_TypeDef * gpio_port;
    uint32_t       gpio_clk;
    rcc_clockcmd   gpio_clkcmd;

    uint16_t 	   gpio_pin;
    uint8_t 	   gpio_af;
    uint8_t 	   gpio_af_tim;
} Channels[] = {
    { TIM1, RCC_APB2Periph_TIM1, RCC_APB2PeriphClockCmd, TIM1_CC_IRQn, TIM_Channel_1, TIM_IT_CC1, GPIOE, RCC_AHB1Periph_GPIOE, RCC_AHB1PeriphClockCmd, GPIO_Pin_9,  GPIO_PinSource9,  GPIO_AF_TIM1 },
    { TIM1, RCC_APB2Periph_TIM1, RCC_APB2PeriphClockCmd, TIM1_CC_IRQn, TIM_Channel_2, TIM_IT_CC2, GPIOE, RCC_AHB1Periph_GPIOE, RCC_AHB1PeriphClockCmd, GPIO_Pin_11, GPIO_PinSource11, GPIO_AF_TIM1 },
    { TIM1, RCC_APB2Periph_TIM1, RCC_APB2PeriphClockCmd, TIM1_CC_IRQn, TIM_Channel_3, TIM_IT_CC3, GPIOE, RCC_AHB1Periph_GPIOE, RCC_AHB1PeriphClockCmd, GPIO_Pin_13, GPIO_PinSource13, GPIO_AF_TIM1 },
    { TIM1, RCC_APB2Periph_TIM1, RCC_APB2PeriphClockCmd, TIM1_CC_IRQn, TIM_Channel_4, TIM_IT_CC4, GPIOE, RCC_AHB1Periph_GPIOE, RCC_AHB1PeriphClockCmd, GPIO_Pin_14, GPIO_PinSource14, GPIO_AF_TIM1 },
    { TIM8, RCC_APB2Periph_TIM8, RCC_APB2PeriphClockCmd, TIM8_CC_IRQn, TIM_Channel_1, TIM_IT_CC1, GPIOC, RCC_AHB1Periph_GPIOC, RCC_AHB1PeriphClockCmd, GPIO_Pin_6,  GPIO_PinSource6,  GPIO_AF_TIM8 },
    { TIM8, RCC_APB2Periph_TIM8, RCC_APB2PeriphClockCmd, TIM8_CC_IRQn, TIM_Channel_2, TIM_IT_CC2, GPIOC, RCC_AHB1Periph_GPIOC, RCC_AHB1PeriphClockCmd, GPIO_Pin_7,  GPIO_PinSource7,  GPIO_AF_TIM8 },
    { TIM8, RCC_APB2Periph_TIM8, RCC_APB2PeriphClockCmd, TIM8_CC_IRQn, TIM_Channel_3, TIM_IT_CC3, GPIOC, RCC_AHB1Periph_GPIOC, RCC_AHB1PeriphClockCmd, GPIO_Pin_8,  GPIO_PinSource8,  GPIO_AF_TIM8 },
    { TIM8, RCC_APB2Periph_TIM8, RCC_APB2PeriphClockCmd, TIM8_CC_IRQn, TIM_Channel_4, TIM_IT_CC4, GPIOC, RCC_AHB1Periph_GPIOC, RCC_AHB1PeriphClockCmd, GPIO_Pin_9,  GPIO_PinSource9,  GPIO_AF_TIM8 },
};

static struct PWM_State {
    uint8_t state;
    uint16_t rise;
    uint16_t fall;
    uint16_t capture;
    uint16_t error;
} Inputs[8];

static TIM_ICInitTypeDef  TIM_ICInitStructure;

void attachPWMCaptureCallback(void (*callback)(uint8_t, uint8_t, int16_t))
{
	pwm_capture_callback = callback;
}

void TIM1_CC_IRQHandler(void) {
	pwmIRQHandler(TIM1);
}

void TIM8_CC_IRQHandler(void) {
    pwmIRQHandler(TIM8);
}

static inline void pwmIRQHandler(TIM_TypeDef *tim)
{
    uint8_t i;
    uint16_t val = 0;
    uint16_t time_on = 0;
    //uint16_t time_off = 0;

    for (i = 0; i < 8; i++) {
        struct TIM_Channel channel = Channels[i];
        struct PWM_State *input = &Inputs[i];

        if (channel.tim == tim && (TIM_GetITStatus(tim, channel.tim_cc) == SET)) {
            TIM_ClearITPendingBit(channel.tim, channel.tim_cc);

            switch (channel.tim_channel) {
                case TIM_Channel_1:
                    val = TIM_GetCapture1(channel.tim);
                    break;
                case TIM_Channel_2:
                    val = TIM_GetCapture2(channel.tim);
                    break;
                case TIM_Channel_3:
                    val = TIM_GetCapture3(channel.tim);
                    break;
                case TIM_Channel_4:
                    val = TIM_GetCapture4(channel.tim);
                    break;
            }


            if (input->state == 0) {
        	input->rise = val;
        	//if(input->rise > input->fall){
        	//    time_off = input->rise - input->fall;
        	//}else{
        	//    time_off = ((0xFFFF - input->fall) + input->rise);
		//}
        	    input->state = 1;
                    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
                    TIM_ICInitStructure.TIM_Channel = channel.tim_channel;
                    TIM_ICInit(channel.tim, &TIM_ICInitStructure);

            } else {
        	input->fall = val;
        	if (input->fall > input->rise)
                    time_on = (input->fall - input->rise);
                else
                    time_on = ((0xFFFF - input->rise) + input->fall);

        	if ((time_on >= MINONWIDTH) && (time_on <= MAXONWIDTH))
        	    {
		    // compute capture. 1 tick value is 0.5us
		    input->capture = time_on >> 1;

		    //reset error count
		    input->error = 0;
		    }

		    // switch state
		    input->state = 0;
		    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		    TIM_ICInitStructure.TIM_Channel = channel.tim_channel;
		    TIM_ICInit(channel.tim, &TIM_ICInitStructure);

            }
        }
    }
}


static void pwmInitializeInput(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

	uint8_t i;

	for (i = 0; i < 8; i++)
	{
		struct TIM_Channel channel = Channels[i];
// timer_reset ******************************************************************/
		channel.tim_clkcmd(channel.tim_clk, ENABLE);
// timer_pause ******************************************************************/
		TIM_Cmd(channel.tim, DISABLE);
// gpio_set_mode ****************************************************************/
		channel.gpio_clkcmd(channel.gpio_clk, ENABLE);
		GPIO_InitStructure.GPIO_Pin   = channel.gpio_pin;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
		GPIO_Init(channel.gpio_port, &GPIO_InitStructure);
// gpio_set_af_mode *************************************************************/
		GPIO_PinAFConfig(channel.gpio_port, channel.gpio_af, channel.gpio_af_tim);
// enable the TIM global interrupt
		NVIC_InitStructure.NVIC_IRQChannel = channel.tim_irq;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
// TIM configuration base *******************************************************/

		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_TimeBaseStructure.TIM_Prescaler = 83; //200KHz
		TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(channel.tim, &TIM_TimeBaseStructure);

// PWM input capture ************************************************************/
		TIM_ICInitStructure.TIM_Channel = channel.tim_channel;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
		TIM_ICInit(channel.tim, &TIM_ICInitStructure);
// timer_enable *****************************************************************/
		TIM_Cmd(channel.tim, ENABLE);
// enable the CC interrupt request **********************************************/
		TIM_ITConfig(channel.tim, channel.tim_cc, ENABLE);
	}
}

void pwmInit(void)
{
    uint8_t i;

    // preset channels to center
    for (i = 0; i < 8; i++)
	{
        Inputs[i].state = 0;
	Inputs[i].capture = 1500;
	Inputs[i].rise = 0;
	Inputs[i].fall = 0;
	Inputs[i].error = 0;
	}

	pwmInitializeInput();

}

uint16_t pwmRead(uint8_t channel)
{
    return Inputs[channel].capture;
}
