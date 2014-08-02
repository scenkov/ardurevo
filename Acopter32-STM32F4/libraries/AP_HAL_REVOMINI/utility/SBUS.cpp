/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "SBUS.h"
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
struct sbus_bit_pick {
	uint8_t byte;
	uint8_t rshift;
	uint8_t mask;
	uint8_t lshift;
};
static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] = {
	/*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
	/*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
	/*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
	/*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
	/* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
	/* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

void SBUSClass::begin() {

	for (byte i = 0; i<18; i++) {
		_channels[i]      = 0;
	}
	_failsafe = 0;
	_last_update = 0;

	_serial->begin(100000,1);

	hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&SBUSClass::_process));
}

void SBUSClass::_process() {
	static byte buffer[25];
	static byte buffer_index = 0;
	
	while (_serial->available()) {
		byte rx = _serial->read();
		if (buffer_index == 0 && rx != SBUS_STARTBYTE) {
			//incorrect start byte, out of sync
			_decoderErrorFrames++;
			continue;
		}
		
		buffer[buffer_index++] = rx;

		if (buffer_index == 25) {
			buffer_index = 0;
			switch (buffer[24]){
			    case 0x00:
				/* this is S.BUS 1 */
				break;
			    case 0x03:
				/* S.BUS 2 SLOT0: RX battery and external voltage */
				break;
			    case 0x83:
				/* S.BUS 2 SLOT1 */
				break;
			    case 0x43:
			    case 0xC3:
			    case 0x23:
			    case 0xA3:
			    case 0x63:
			    case 0xE3:
				break;
			    default:
				/* we expect one of the bits above, but there are some we don't know yet */
				_decoderErrorFrames++;
				continue;
				break;
				//incorrect end byte, out of sync
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
				_failsafe = SBUS_FAILSAFE_ACTIVE;
			} else {
				_failsafe = SBUS_FAILSAFE_INACTIVE;
			}

			if ((buffer[23] >> 2) & 0x0001) {
				_lostFrames++;
			}

			_last_update = hal.scheduler->millis();
		}
	}
}

uint16_t SBUSClass::getChannel(int channel) {
	if (channel < 0 or channel > 17) {
		return 0;
	} else {
	    if(hal.scheduler->millis() - _last_update > 500) {
		_failsafe = SBUS_FAILSAFE_ACTIVE;
	    }
	    if (channel == 2 && _failsafe) { //hardcoded failsafe action if RX is in failsafe, put throttle to 900
		return 900;
	    } else {
		return (uint16_t)(_channels[channel] * SBUS_SCALE_FACTOR +.5f) + SBUS_SCALE_OFFSET;
	    }
	}
}

uint16_t SBUSClass::getFailsafeStatus() {
	return _failsafe;
}

uint16_t SBUSClass::getFrameLoss() {
	return (int) ((_lostFrames + _decoderErrorFrames) * 100 / (_goodFrames + _lostFrames + _decoderErrorFrames));
}

uint64_t SBUSClass::getGoodFrames() {
	return _goodFrames;
}

uint64_t SBUSClass::getLostFrames() {
	return _lostFrames;
}

uint64_t SBUSClass::getDecoderErrorFrames() {
	return _decoderErrorFrames;
}

