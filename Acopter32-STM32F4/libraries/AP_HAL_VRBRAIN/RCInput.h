/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


#ifndef __AP_HAL_VRBRAIN_RCINPUT_H__
#define __AP_HAL_VRBRAIN_RCINPUT_H__

#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL.h>
#include <utility/SBUS.h>

#define VRBRAIN_RC_INPUT_MIN_CHANNELS 4
#define VRBRAIN_RC_INPUT_NUM_CHANNELS 8
#define PPM_SUM_CHANNEL 75

class VRBRAIN::VRBRAINRCInput : public AP_HAL::RCInput {
public:
    VRBRAINRCInput();
    void init(void* machtnichts);
    bool new_input();
    uint8_t  num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();

    enum RC_type {
        SBUS,
        PPMSUM,
        DSM,
        PWM
    };
private:
    static void rxIntPPMSUM(uint8_t state, uint16_t value);
    static void rxIntPWM(uint8_t chan, uint16_t value);
    void InitDefaultPPM(char board);
    void _detect_rc(void);
    bool _sbus_dct();
    bool _ppmsum_dct();

    unsigned int ppm_sum_channel;
    unsigned int input_channel_ch1;
    unsigned int input_channel_ch2;
    unsigned int input_channel_ch3;
    unsigned int input_channel_ch4;
    unsigned int input_channel_ch5;
    unsigned int input_channel_ch6;
    unsigned int input_channel_ch7;
    unsigned int input_channel_ch8;

    static volatile uint8_t  _valid;

    /* override state */
    uint16_t _override[VRBRAIN_RC_INPUT_NUM_CHANNELS];

    /* private variables to communicate with input capture isr */
    static volatile uint16_t _channel[VRBRAIN_RC_INPUT_NUM_CHANNELS];
    static volatile uint32_t _last_pulse[VRBRAIN_RC_INPUT_NUM_CHANNELS];
    static volatile uint8_t  _valid_channels;
    static volatile uint32_t _last_input_interrupt_time;

    SBUSClass *_sbus;

    RC_type _rc_type;
    uint8_t _detected;

};

#endif // __AP_HAL_VRBRAIN_RCINPUT_H__
