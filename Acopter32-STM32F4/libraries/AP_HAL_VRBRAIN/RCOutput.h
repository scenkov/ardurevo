
#ifndef __AP_HAL_VRBRAIN_RCOUTPUT_H__
#define __AP_HAL_VRBRAIN_RCOUTPUT_H__

#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL.h>
#include <timer.h>

#define VRBRAIN_MAX_OUTPUT_CHANNELS 12

#define MOTORID1 0
#define MOTORID2 1
#define MOTORID3 2
#define MOTORID4 3
#define MOTORID5 4
#define MOTORID6 5
#define MOTORID7 6
#define MOTORID8 7
#define MOTORID9 8
#define MOTORID10 9
#define MOTORID11 10
#define MOTORID12 11
class VRBRAIN::VRBRAINRCOutput : public AP_HAL::RCOutput {
    void     init(void* implspecific);
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

    /*
      set PWM to send to a set of channels when the safety switch is
      in the safe state
     */
    void     set_safety_pwm(uint32_t chmask, uint16_t period_us);
private:
    uint32_t _timer_period(uint16_t speed_hz);
    uint8_t _num_motors;

    uint8_t out_ch1;
    uint8_t out_ch2;
    uint8_t out_ch3;
    uint8_t out_ch4;
    uint8_t out_ch5;
    uint8_t out_ch6;
    uint8_t out_ch7;
    uint8_t out_ch8;
    uint8_t out_ch9;
    uint8_t out_ch10;
    uint8_t out_ch11;
    uint8_t out_ch12;
    uint32_t output_channel_raw[VRBRAIN_MAX_OUTPUT_CHANNELS];

};

#endif // __AP_HAL_VRBRAIN_RCOUTPUT_H__
