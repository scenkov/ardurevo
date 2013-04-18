/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_HMC5843_H
#define AP_Compass_HMC5843_H

#include <wirish.h>

#include <AP_Common.h>
#include <AP_Math.h>

#include "Compass.h"

//#define AP_COMPASS_HMC5843_DEBUG_ENABLE

// orientations for DIYDrones magnetometer
#define AP_COMPASS_COMPONENTS_UP_PINS_FORWARD ROTATION_NONE
#define AP_COMPASS_COMPONENTS_UP_PINS_FORWARD_RIGHT ROTATION_YAW_45
#define AP_COMPASS_COMPONENTS_UP_PINS_RIGHT ROTATION_YAW_90
#define AP_COMPASS_COMPONENTS_UP_PINS_BACK_RIGHT ROTATION_YAW_135
#define AP_COMPASS_COMPONENTS_UP_PINS_BACK ROTATION_YAW_180
#define AP_COMPASS_COMPONENTS_UP_PINS_BACK_LEFT ROTATION_YAW_225
#define AP_COMPASS_COMPONENTS_UP_PINS_LEFT ROTATION_YAW_270
#define AP_COMPASS_COMPONENTS_UP_PINS_FORWARD_LEFT ROTATION_YAW_315
#define AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD ROTATION_ROLL_180
#define AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD_RIGHT ROTATION_ROLL_180_YAW_45
#define AP_COMPASS_COMPONENTS_DOWN_PINS_RIGHT ROTATION_ROLL_180_YAW_90
#define AP_COMPASS_COMPONENTS_DOWN_PINS_BACK_RIGHT ROTATION_ROLL_180_YAW_135
#define AP_COMPASS_COMPONENTS_DOWN_PINS_BACK ROTATION_PITCH_180
#define AP_COMPASS_COMPONENTS_DOWN_PINS_BACK_LEFT ROTATION_ROLL_180_YAW_225
#define AP_COMPASS_COMPONENTS_DOWN_PINS_LEFT ROTATION_ROLL_180_YAW_270
#define AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD_LEFT ROTATION_ROLL_180_YAW_315
#define AP_COMPASS_MP32_SHIELD ROTATION_NONE

// orientations for Sparkfun magnetometer
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_FORWARD ROTATION_YAW_270
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_FORWARD_RIGHT ROTATION_YAW_315
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_RIGHT ROTATION_NONE
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_BACK_RIGHT ROTATION_YAW_45
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_BACK ROTATION_YAW_90
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_BACK_LEFT ROTATION_YAW_135
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_LEFT ROTATION_YAW_180
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_FORWARD_LEFT ROTATION_YAW_225
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_FORWARD ROTATION_ROLL_180_YAW_90
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_FORWARD_RIGHT ROTATION_ROLL_180_YAW_135
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_RIGHT ROTATION_PITCH_180
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_BACK_RIGHT ROTATION_ROLL_180_YAW_225
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_BACK ROTATION_ROLL_180_YAW_270
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_BACK_LEFT ROTATION_ROLL_180_YAW_315
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_LEFT ROTATION_ROLL_180
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_FORWARD_LEFT ROTATION_ROLL_180_YAW_45

class AP_Compass_HMC5843 : public Compass
{
private:
	float calibration[3];
    bool _initialised;
	//virtual bool read_raw(void);
    uint8_t _base_config;
	virtual bool re_initialise(void);
    bool read_register(uint8_t address, uint8_t *value);
    bool write_register(uint8_t address, byte value);
    uint32_t _retry_time; // when unhealthy the millis() value to retry at

    HardwareI2C *_I2Cx;
    int16_t			    _mag_x;
    int16_t			    _mag_y;
    int16_t			    _mag_z;
    int16_t             _mag_x_accum;
    int16_t             _mag_y_accum;
    int16_t             _mag_z_accum;
    uint8_t			    _accum_count;
    uint32_t            _last_accum_time;

public:
	// Constructor
    AP_Compass_HMC5843(HardwareI2C *i2c_d, FastSerial *ser_port);
	virtual bool init(void);
	virtual bool read(void);
    virtual void accumulate(void);
	virtual void set_orientation(enum Rotation rotation);
	//made public for debug:
	virtual bool read_raw(void);
	
	int16_t         raw_x;          ///< magnetic field strength along the X axis
	int16_t         raw_y;          ///< magnetic field strength along the Y axis
	int16_t         raw_z;          ///< magnetic field strength along the Z axis
	
	int16_t         cal_x;          ///< magnetic field strength along the X axis
	int16_t         cal_y;          ///< magnetic field strength along the Y axis
	int16_t         cal_z;          ///< magnetic field strength along the Z axis

};
#endif
