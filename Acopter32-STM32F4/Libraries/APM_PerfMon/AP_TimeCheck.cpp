/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <FastSerial.h>
#include "AP_TimeCheck.h"

//static FastSerial *serPort;

// constructor
AP_TimeCheck::AP_TimeCheck(uint32_t target, uint32_t step_size, FastSerial * serPort) :
	time_taken_min(0),
	time_taken_max(0),
	time_target(target),
	time_step(step_size),
	first_iteration(true)
{
	_serial = serPort;
	// setup-up the time_value (i.e. buckets)
	for(int16_t i=0; i<AP_TIMEARRAY_SIZE; i++){
		time_value[i] = target + time_step * i;
		time_count[i] = 0;
	}
}

// clear - clear all timing counts
void AP_TimeCheck::clear()
{
	// clear min and max
	time_taken_min = 0;
	time_taken_max = 0;

	// clear counts
	for(int16_t i=0; i<AP_TIMEARRAY_SIZE; i++){
		time_count[i] = 0;
	}

	// reset first iteration
	first_iteration = true;
}

// dump_results - print timer results to the console
void AP_TimeCheck::dump_results()
{
	if(_serial)
	{
		_serial->println();
		_serial->println("AP_Timer Results:");
		_serial->print("min:");
		_serial->println(time_taken_min);
		_serial->print("max:");
		_serial->println(time_taken_max);
		for(int16_t i=0; i<AP_TIMEARRAY_SIZE; i++){
			_serial->printf("%lu ~ %lu: %lu\n",(unsigned long)time_value[i], (unsigned long)(time_value[i]+time_step-1), (unsigned long)time_count[i]);
		}
	}
}

// addTime - increment the count for nearest time bucket
void AP_TimeCheck::addTime(uint32_t time_taken)
{
	// throw away first iteration because it's often flawed
	if( first_iteration ) {
		first_iteration = false;
		return;
	}

	// check min/max and update if necessary
	if( time_taken_max == 0 || time_taken > time_taken_max ){
		time_taken_max = time_taken;
	}
	if( time_taken_min == 0 || time_taken < time_taken_min ){
		time_taken_min = time_taken;
	}

	// find which time bucket's count should be increased
	for(int16_t i=AP_TIMEARRAY_SIZE-1; i>=0; i--){
		if( i==0 || time_taken >= time_value[i] ){
			// if the correct bucket's count is not full, increase it's count
			if( time_count[i] < AP_TIMEARRAY_MAX_COUNT_VALUE ){
				time_count[i]++;
				// debug
				//Serial.printf("b:%d %lu\n",i,time_count[i]);
			}else{
				// if this bucket's count is full, decrement all other buckets
				for(int16_t j=0; i<AP_TIMEARRAY_SIZE; j++) {
					if( j != i && time_count[j] > 0 ){
						time_count[j]--;
					}
				}
				// debug
				//Serial.printf("ra\n");
			}

			// we must have updated a time_count so exit function
			return;
		}
	}
}

/// total_count - total number of timing results (i.e. sum of each time bucket's count)
uint32_t AP_TimeCheck::total_count()
{
    uint32_t total = 0;
    for(int16_t i=0; i<AP_TIMEARRAY_SIZE; i++){
		total += time_count[i];
	}
    return total;
}
