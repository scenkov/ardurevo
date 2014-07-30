/*
 * SSS.h
 *
 *  Created on: 20/apr/2014
 *      Author: Sandro
 */

#ifndef SSS_h
#define SSS_h

#include <AP_HAL.h>
//#include "Arduino.h"

#define SSS_FAILSAFE_INACTIVE 0
#define SSS_FAILSAFE_ACTIVE   1
#define SSS_STARTBYTE         0x03
#define SSS_ENDBYTE           0xff

#define SSS_FRAME_SIZE		16
#define SSS_INPUT_CHANNELS	7

//#define SBUS_FLAGS_BYTE		23
//#define SBUS_FAILSAFE_BIT	3
//#define SBUS_FRAMELOST_BIT	2


/*
  Measured values with Futaba FX-30/R6108SB:
    -+100% on TX:  PCM 1.100/1.520/1.950ms -> SBus raw values: 350/1024/1700  (100% ATV)
    -+140% on TX:  PCM 0.930/1.520/2.112ms -> SBus raw values:  78/1024/1964  (140% ATV)
    -+152% on TX:  PCM 0.884/1.520/2.160ms -> SBus raw values:   1/1024/2047  (140% ATV plus dirty tricks)
*/

/* define range mapping here, -+100% -> 1000..2000 */
#define SSS_RANGE_MIN 200.0f
#define SSS_RANGE_MAX 1800.0f

#define SSS_TARGET_MIN 1000.0f
#define SSS_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SSS_SCALE_FACTOR ((SSS_TARGET_MAX - SSS_TARGET_MIN) / (SSS_RANGE_MAX - SSS_RANGE_MIN))
#define SSS_SCALE_OFFSET (int)(SSS_TARGET_MIN - (SSS_SCALE_FACTOR * SSS_RANGE_MIN + 0.5f))

class SSSClass {
	public:
		SSSClass(AP_HAL::UARTDriver * serial) : _serial (serial){}
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

#endif /* SSS_H_ */
