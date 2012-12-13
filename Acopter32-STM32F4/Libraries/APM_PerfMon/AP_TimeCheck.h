/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_TIMECHECK_H
#define AP_TIMECHECK_H
#include <inttypes.h>

// definitions
#define AP_TIMEARRAY_SIZE 20
#define AP_TIMEARRAY_MAX_COUNT_VALUE 4294967286U //(2^32-10)

class AP_TimeCheck
{
public:
    /// Constructor
    ///
    AP_TimeCheck(uint32_t target, uint32_t step_size = 500, FastSerial * serPort = NULL);

	/// clear - clear all timing counts
	void clear();

	/// dump_results - print timer results to the console
	void dump_results();

	/// addTime - increment the count for nearest time bucket
	void addTime(uint32_t time_taken);

    /// total_count - total number of timing results (i.e. sum of each time bucket's count)
	uint32_t total_count();

protected:
	uint32_t time_taken_min;					/// minimum recorded time taken
	uint32_t time_taken_max;					/// maximum recorded time taken
	uint32_t time_target;						/// target time of the loop (also used for lowest time bucket)
	uint32_t time_step;							/// size of each time bucket (in millis or micros)
	uint32_t time_value[AP_TIMEARRAY_SIZE];		/// array holding the lowest time value of each bucket
	uint32_t time_count[AP_TIMEARRAY_SIZE];		/// array holding the latest count for the bucket
	bool first_iteration;
private:
	FastSerial * _serial;
};
#endif
