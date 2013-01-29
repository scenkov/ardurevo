
#include "GPIO.h"
#include "ext_interrupts.h"
#include "exti.h"
#include "boards.h"

static inline exti_trigger_mode exti_out_mode(ExtIntTriggerMode mode);

using namespace VRBRAIN;

VRBRAINGPIO::VRBRAINGPIO()
{
}

void VRBRAINGPIO::init()
{
}

void VRBRAINGPIO::pinMode(uint8_t pin, uint8_t output)
{
    gpio_pin_mode outputMode;
    bool pwm = false;

    if (pin >= BOARD_NR_GPIO_PINS) {
        return;
    }

    switch(output) {
    case OUTPUT:
        outputMode = GPIO_OUTPUT_PP;
        break;
    case OUTPUT_OPEN_DRAIN:
        outputMode = GPIO_OUTPUT_OD;
        break;
    case INPUT:
    case INPUT_FLOATING:
        outputMode = GPIO_INPUT_FLOATING;
        break;
    case INPUT_ANALOG:
        outputMode = GPIO_INPUT_ANALOG;
        break;
    case INPUT_PULLUP:
        outputMode = GPIO_INPUT_PU;
        break;
    case INPUT_PULLDOWN:
        outputMode = GPIO_INPUT_PD;
        break;
    case PWM:
        outputMode = GPIO_AF_OUTPUT_PP;
        pwm = true;
        break;
    case PWM_OPEN_DRAIN:
        outputMode = GPIO_AF_OUTPUT_OD;
        pwm = true;
        break;
    default:
        assert_param(0);
        return;
    }

    gpio_set_mode(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, outputMode);

    if (PIN_MAP[pin].timer_device != NULL) {

		if (pwm)
		{
			if (PIN_MAP[pin].timer_device->regs == TIM1)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM1);
			else if (PIN_MAP[pin].timer_device->regs == TIM2)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM2);
			else if (PIN_MAP[pin].timer_device->regs == TIM3)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM3);
			else if (PIN_MAP[pin].timer_device->regs == TIM4)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM4);
			else if (PIN_MAP[pin].timer_device->regs == TIM5)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM5);
			else if (PIN_MAP[pin].timer_device->regs == TIM8)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM8);
			else if (PIN_MAP[pin].timer_device->regs == TIM13)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM13);
			else if (PIN_MAP[pin].timer_device->regs == TIM14)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM14);

			timer_set_mode(PIN_MAP[pin].timer_device, PIN_MAP[pin].timer_channel, TIMER_PWM);
		}
        /* Enable/disable timer channels if we're switching into or out of PWM. */
        //timer_set_mode(PIN_MAP[pin].timer_device, PIN_MAP[pin].timer_channel, pwm ? TIMER_PWM : TIMER_DISABLED);
    }

}

uint8_t VRBRAINGPIO::read(uint8_t pin)
{
    if (pin >= BOARD_NR_GPIO_PINS) {
        return 0;
    }

    return gpio_read_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit) ?
        HIGH : LOW;
}

void VRBRAINGPIO::write(uint8_t pin, uint8_t value)
{
    if (pin >= BOARD_NR_GPIO_PINS) {
        return;
    }

    gpio_write_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, value);

}

/* Alternative interface: */
AP_HAL::DigitalSource* VRBRAINGPIO::channel(uint16_t n) {
    return new VRBRAINDigitalSource(0);
}

/* Interrupt interface: */
bool VRBRAINGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
                                  uint8_t mode)
{
    if (interrupt_num >= BOARD_NR_GPIO_PINS || !p) {
        return;
    }

    exti_trigger_mode outMode = exti_out_mode(mode);

    exti_attach_interrupt((afio_exti_num)(PIN_MAP[interrupt_num].gpio_bit),
                          gpio_exti_port(PIN_MAP[interrupt_num].gpio_device),
                          p,
                          outMode);

    return true;
}

VRBRAINDigitalSource::VRBRAINDigitalSource(uint8_t v) :
  _v(v)
{
}

void VRBRAINDigitalSource::mode(uint8_t output)
{
}

uint8_t VRBRAINDigitalSource::read()
{
  return _v;
}

void VRBRAINDigitalSource::write(uint8_t value)
{
  _v = value;
}


static inline exti_trigger_mode exti_out_mode(ExtIntTriggerMode mode) {
    switch (mode) {
    case RISING:
        return EXTI_RISING;
    case FALLING:
        return EXTI_FALLING;
    case CHANGE:
        return EXTI_RISING_FALLING;
    }
    // Can't happen
    assert_param(0);
    return (exti_trigger_mode)0;
}
