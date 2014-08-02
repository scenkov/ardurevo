/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef SBUS_h
#define SBUS_h

#include <AP_HAL.h>
//#include "Arduino.h"

#define SBUS_FAILSAFE_INACTIVE 0
#define SBUS_FAILSAFE_ACTIVE   1
#define SBUS_STARTBYTE         0x0f
#define SBUS_ENDBYTE           0x00

#define SBUS_FRAME_SIZE		25
#define SBUS_INPUT_CHANNELS	16
#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2


/*
  Measured values with Futaba FX-30/R6108SB:
    -+100% on TX:  PCM 1.100/1.520/1.950ms -> SBus raw values: 350/1024/1700  (100% ATV)
    -+140% on TX:  PCM 0.930/1.520/2.112ms -> SBus raw values:  78/1024/1964  (140% ATV)
    -+152% on TX:  PCM 0.884/1.520/2.160ms -> SBus raw values:   1/1024/2047  (140% ATV plus dirty tricks)
*/

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

class SBUSClass {
	public:
		SBUSClass(AP_HAL::UARTDriver * serial) : _serial (serial){}
		void begin();
		uint16_t getChannel(int channel);
		uint16_t getFailsafeStatus();
		uint16_t getFrameLoss();
		uint64_t getGoodFrames();
		uint64_t getLostFrames();
		uint64_t getDecoderErrorFrames();
	private:
		void _process();
		AP_HAL::UARTDriver * _serial;
		uint16_t _channels[18];
		uint16_t _failsafe;
		uint64_t _goodFrames;
		uint64_t _lostFrames;
		uint64_t _decoderErrorFrames;
		uint32_t _last_update;
};


#endif
