/*
 * SSS.cpp
 *
 *  Created on: 20/apr/2014
 *      Author: Sandro
 */

#include <AP_HAL.h>
#include "SSS.h"
#include <usart.h>
#include <gpio_hal.h>
#include <math.h>

extern const AP_HAL::HAL& hal;

/*
 * S.bus decoder matrix.
 *
 * Each channel value can come from up to 3 input bytes. Each row in the
 * matrix describes up to three bytes, and each entry gives:
 *
 * - byte offset in the data portion of the frame
 * - right shift applied to the data byte
 * - mask for the data byte
 * - left shift applied to the result into the channel value
 */

void SSSClass::begin() {

	for (byte i = 0; i<18; i++) {
		_channels[i]      = 0;
	}
	_failsafe = 0;
	_last_update = 0;

	_serial->begin(100000,1);

	hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&SSSClass::_process));
}

void SSSClass::_process() {
	static byte buffer[25];
	static byte buffer_index = 0;

	while (_serial->available()) {
		byte rx = _serial->read();
		if (buffer_index == 0 && rx != SSS_STARTBYTE) {
			//incorrect start byte, out of sync
			_decoderErrorFrames++;
			continue;
		}

		buffer[buffer_index++] = rx;

		if (buffer_index == 25) {
			buffer_index = 0;
			if (buffer[24] != SSS_ENDBYTE) {
				//incorrect end byte, out of sync
				_decoderErrorFrames++;
				continue;
			}
			_goodFrames++;

			_channels[0]  = ((buffer[1]    |buffer[2]<<8)                 & 0x07FF);
			_channels[1]  = ((buffer[2]>>3 |buffer[3]<<5)                 & 0x07FF);
			_channels[2]  = ((buffer[3]>>6 |buffer[4]<<2 |buffer[5]<<10)  & 0x07FF);
			_channels[3]  = ((buffer[5]>>1 |buffer[6]<<7)                 & 0x07FF);
			_channels[4]  = ((buffer[6]>>4 |buffer[7]<<4)                 & 0x07FF);
			_channels[5]  = ((buffer[7]>>7 |buffer[8]<<1 |buffer[9]<<9)   & 0x07FF);
			_channels[6]  = ((buffer[9]>>2 |buffer[10]<<6)                & 0x07FF);
			_channels[7]  = ((buffer[10]>>5|buffer[11]<<3)                & 0x07FF);
			_channels[8]  = ((buffer[12]   |buffer[13]<<8)                & 0x07FF);
			_channels[9]  = ((buffer[13]>>3|buffer[14]<<5)                & 0x07FF);
			_channels[10] = ((buffer[14]>>6|buffer[15]<<2|buffer[16]<<10) & 0x07FF);
			_channels[11] = ((buffer[16]>>1|buffer[17]<<7)                & 0x07FF);
			_channels[12] = ((buffer[17]>>4|buffer[18]<<4)                & 0x07FF);
			_channels[13] = ((buffer[18]>>7|buffer[19]<<1|buffer[20]<<9)  & 0x07FF);
			_channels[14] = ((buffer[20]>>2|buffer[21]<<6)                & 0x07FF);
			_channels[15] = ((buffer[21]>>5|buffer[22]<<3)                & 0x07FF);

			((buffer[23])      & 0x0001) ? _channels[16] = 2047: _channels[16] = 0;
			((buffer[23] >> 1) & 0x0001) ? _channels[17] = 2047: _channels[17] = 0;

			if ((buffer[23] >> 3) & 0x0001) {
				_failsafe = SSS_FAILSAFE_ACTIVE;
			} else {
				_failsafe = SSS_FAILSAFE_INACTIVE;
			}

			if ((buffer[23] >> 2) & 0x0001) {
				_lostFrames++;
			}

			_last_update = hal.scheduler->millis();
		}
	}
}

uint16_t SSSClass::getChannel(int channel) {
	if (channel < 0 or channel > 17) {
		return 0;
	} else {
	    if(hal.scheduler->millis() - _last_update > 500) {
		_failsafe = 1;
	    }
	    if (channel == 2 && _failsafe) { //hardcoded failsafe action if RX is in failsafe, put throttle to 900
		return 900;
	    } else {
		return (uint16_t)(_channels[channel] * SSS_SCALE_FACTOR +.5f) + SSS_SCALE_OFFSET;
	    }
	}
}

uint16_t SSSClass::getFailsafeStatus() {
	return _failsafe;
}

uint16_t SSSClass::getFrameLoss() {
	return (int) ((_lostFrames + _decoderErrorFrames) * 100 / (_goodFrames + _lostFrames + _decoderErrorFrames));
}

uint64_t SSSClass::getGoodFrames() {
	return _goodFrames;
}

uint64_t SSSClass::getLostFrames() {
	return _lostFrames;
}

uint64_t SSSClass::getDecoderErrorFrames() {
	return _decoderErrorFrames;
}
