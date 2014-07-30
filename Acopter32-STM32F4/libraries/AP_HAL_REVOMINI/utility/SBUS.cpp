/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Code partially taken from the PX4 NuttX firmware */

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
		_channels[i]      = 1500;
	}
	_failsafe = 0;
	_last_update = hal.scheduler->micros();
	_last_frame = hal.scheduler->micros();

	_serial->begin(100000,1);

	_partial_frame_count = 0;


	hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&SBUSClass::_process));
}



void SBUSClass::_process() {

    uint32_t now = hal.scheduler->micros();
    uint8_t bytes = 0;

    if ((now - _last_update) > 3000)
	{
	if (_partial_frame_count > 0)
	    {
	    _decoderErrorFrames++;
	    _partial_frame_count = 0;
	    }
	}

    for (uint8_t i = 0; i < SBUS_FRAME_SIZE - _partial_frame_count; i++) {
	if (_serial->available())
	    {
	    frame[_partial_frame_count] = _serial->read();
	    _partial_frame_count++;
	    bytes++;
	    }
	else {
	    continue;
	}
    }

    if (bytes < 1)
	return;

    _last_update = now;

    if (_partial_frame_count < SBUS_FRAME_SIZE)
	return;

    _partial_frame_count = 0;


	    if (frame[0] != SBUS_STARTBYTE) {
		//incorrect start byte, out of sync
		_decoderErrorFrames++;
		_serial->flush();
		return;
	    }
	    switch (frame[24]){
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
		    break;
		    //incorrect end byte, out of sync
	    }

	    _last_frame = now;

	    for (uint8_t channel = 0; channel < SBUS_INPUT_CHANNELS; channel++) {
		    uint16_t value = 0;

		    for (uint8_t pick = 0; pick < 3; pick++) {
			    const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];

			    if (decode->mask != 0) {
				    unsigned piece = frame[1 + decode->byte];
				    piece >>= decode->rshift;
				    piece &= decode->mask;
				    piece <<= decode->lshift;

				    value |= piece;
			    }
		    }


		    /* convert 0-2048 values to 1000-2000 ppm encoding in a not too sloppy fashion */
		    _channels[channel] = (uint16_t)(value * SBUS_SCALE_FACTOR +.5f) + SBUS_SCALE_OFFSET;
	    }
	    /* channel 17 (index 16) */
	    _channels[16] = (frame[SBUS_FLAGS_BYTE] & (1 << 0)) * 1000 + 998;
	    /* channel 18 (index 17) */
	    _channels[17] = (frame[SBUS_FLAGS_BYTE] & (1 << 1)) * 1000 + 998;

	    if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
		_failsafe = SBUS_FAILSAFE_ACTIVE;
		_lostFrames++;
	    } else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) {
		_failsafe = SBUS_FAILSAFE_INACTIVE;
		_lostFrames++;
	    } else {
		 _failsafe = SBUS_FAILSAFE_INACTIVE;
	    }


}

uint16_t SBUSClass::getChannel(uint8_t channel) {
	if (channel < 0 || channel > 17) {
		return 0;
	} else {
	    /*if(hal.scheduler->micros() - _last_frame > 500000) {
		_failsafe = SBUS_FAILSAFE_ACTIVE;
	    }*/
	    if (channel == 2 && _failsafe) { //hardcoded failsafe action if RX is in failsafe, put throttle to 900
		return 900;
	    } else {
		return (uint16_t)(_channels[channel]);
	    }
	}
}

uint16_t SBUSClass::getFailsafeStatus() {
	return _failsafe;
}


