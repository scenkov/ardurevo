
#include "GPIO.h"
#include <gpio_hal.h>
#include <ext_interrupts.h>
#include <exti.h>
#include <boards.h>

static inline exti_trigger_mode exti_out_mode(ExtIntTriggerMode mode);

using namespace VRBRAIN;

/**
 * Specifies a GPIO pin behavior.
 * @see pinMode()
 */
typedef enum WiringPinMode {
    OUTPUT, /* Basic digital output: when the pin is HIGH, the
               voltage is held at +3.3v (Vcc) and when it is LOW, it
               is pulled down to ground. */

    OUTPUT_OPEN_DRAIN, /**< In open drain mode, the pin indicates
                          "low" by accepting current flow to ground
                          and "high" by providing increased
                          impedance. An example use would be to
                          connect a pin to a bus line (which is pulled
                          up to a positive voltage by a separate
                          supply through a large resistor). When the
                          pin is high, not much current flows through
                          to ground and the line stays at positive
                          voltage; when the pin is low, the bus
                          "drains" to ground with a small amount of
                          current constantly flowing through the large
                          resistor from the external supply. In this
                          mode, no current is ever actually sourced
                          from the pin. */

    INPUT, /**< Basic digital input. The pin voltage is sampled; when
              it is closer to 3.3v (Vcc) the pin status is high, and
              when it is closer to 0v (ground) it is low. If no
              external circuit is pulling the pin voltage to high or
              low, it will tend to randomly oscillate and be very
              sensitive to noise (e.g., a breath of air across the pin
              might cause the state to flip). */

    INPUT_ANALOG, /**< This is a special mode for when the pin will be
                     used for analog (not digital) reads.  Enables ADC
                     conversion to be performed on the voltage at the
                     pin. */

    INPUT_PULLUP, /**< The state of the pin in this mode is reported
                     the same way as with INPUT, but the pin voltage
                     is gently "pulled up" towards +3.3v. This means
                     the state will be high unless an external device
                     is specifically pulling the pin down to ground,
                     in which case the "gentle" pull up will not
                     affect the state of the input. */

    INPUT_PULLDOWN, /**< The state of the pin in this mode is reported
                       the same way as with INPUT, but the pin voltage
                       is gently "pulled down" towards 0v. This means
                       the state will be low unless an external device
                       is specifically pulling the pin up to 3.3v, in
                       which case the "gentle" pull down will not
                       affect the state of the input. */

    INPUT_FLOATING, /**< Synonym for INPUT. */

    PWM, /**< This is a special mode for when the pin will be used for
            PWM output (a special case of digital output). */

    PWM_OPEN_DRAIN, /**< Like PWM, except that instead of alternating
                       cycles of LOW and HIGH, the voltage on the pin
                       consists of alternating cycles of LOW and
                       floating (disconnected). */
} WiringPinMode;

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

    if ((pin < 0) || (pin >= BOARD_NR_GPIO_PINS)) {
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

int8_t  VRBRAINGPIO::analogPinToDigitalPin(uint8_t pin){
    return 1;
}

uint8_t VRBRAINGPIO::read(uint8_t pin)
{
    if ((pin < 0) || (pin >= BOARD_NR_GPIO_PINS)) {
        return 0;
    }

    return gpio_read_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit) ?
        HIGH : LOW;
}

void VRBRAINGPIO::write(uint8_t pin, uint8_t value)
{
    if ((pin < 0) || (pin >= BOARD_NR_GPIO_PINS)) {
        return;
    }

    gpio_write_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, value);

}

void VRBRAINGPIO::toggle(uint8_t pin)
{
    if ((pin < 0) || (pin >= BOARD_NR_GPIO_PINS)) {
        return;
    }
    gpio_toggle_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit);

}



/* Alternative interface: */
AP_HAL::DigitalSource* VRBRAINGPIO::channel(uint16_t pin) {

    if ((pin < 0) || (pin >= BOARD_NR_GPIO_PINS)) {
        return NULL;
    }

    uint8_t bit;
    gpio_dev *device;
    bit = PIN_MAP[pin].gpio_bit;
    device = PIN_MAP[pin].gpio_device;

    return new VRBRAINDigitalSource(device, bit);
}

/* Interrupt interface: */
bool VRBRAINGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
                                  uint8_t mode)
{
    if ((interrupt_num < 0) || (interrupt_num >= BOARD_NR_GPIO_PINS) || !p) {
        return false;
    }

    exti_trigger_mode outMode = exti_out_mode((ExtIntTriggerMode)mode);

    exti_attach_interrupt((afio_exti_num)(PIN_MAP[interrupt_num].gpio_bit),
                          gpio_exti_port(PIN_MAP[interrupt_num].gpio_device),
                          p,
                          outMode);

    return true;
}

void VRBRAINDigitalSource::mode(uint8_t output)
{
    gpio_pin_mode outputMode;

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
       default:
           assert_param(0);
           return;
       }
    gpio_set_mode(_device, _bit, outputMode);
}

uint8_t VRBRAINDigitalSource::read()
{
    return gpio_read_bit(_device, _bit) ?
        HIGH : LOW;
}

void VRBRAINDigitalSource::write(uint8_t value)
{
    gpio_write_bit(_device, _bit, value);
}

void VRBRAINDigitalSource::toggle()
{
    gpio_toggle_bit(_device, _bit);
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
