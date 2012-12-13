/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
 *  ArduCopter parameter definitions
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */

#define GSCALAR(v, name) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, class::var_info }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, class::var_info }

const AP_Param::Info var_info[]  = {
	GSCALAR(format_version, "SYSID_SW_MREV"),
	GSCALAR(software_type,  "SYSID_SW_TYPE"),

	GSCALAR(sysid_this_mav,	"SYSID_THISMAV"),
	GSCALAR(sysid_my_gcs,	"SYSID_MYGCS"),
	GSCALAR(serial_aux_port,"SER_AUX_PORT"),
	GSCALAR(serial_aux_baud,"SER_AUX_BAUD"),
	GSCALAR(serial_gps_port,"SER_GPS_PORT"),
	GSCALAR(serial_gps_baud,"SER_GPS_BAUD"),
	GSCALAR(serial_cli_port,"SER_CLI_PORT"),
	GSCALAR(serial_cli_baud,"SER_CLI_BAUD"),
    
	GSCALAR(board_id,"BOARD_ID"),
	GSCALAR(motor_config,"MOTOR_CONF"),
    
    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Standard
    // @Units: seconds
    // @Range: 0 10
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY"),
	
    // @Param: ALT_RTL
    // @DisplayName: RTL Altitude
    // @Description: The minimum altitude the model will move to before Returning to Launch.  Set to zero to return at current altitude.
    // @Units: centimeters
    // @Range: 0 4000
    // @Increment: 1
    // @User: Standard
	GSCALAR(rtl_altitude,	"RTL_ALT"),

    // @Param: SONAR_ENABLE
    // @DisplayName: Enable Sonar
    // @Description: Setting this to Enabled(1) will enable the sonar. Setting this to Disabled(0) will disable the sonar
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
	GSCALAR(sonar_enabled,	"SONAR_ENABLE"),

	GSCALAR(sonar_type,	"SONAR_TYPE"),
	GSCALAR(battery_monitoring, "BATT_MONITOR"),

    // @Param: BATT_FAILSAFE
    // @DisplayName: Battery Failsafe Enable
    // @Description: Controls whether failsafe will be invoked when battery voltage or current runs low
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(battery_fs_enabled, "BATT_FAILSAFE"),
    // @Param: VOLT_DIVIDER
    // @DisplayName: Voltage Divider
    // @Description: TODO
	GSCALAR(volt_div_ratio,	"VOLT_DIVIDER"),

	GSCALAR(curr_amp_per_volt,	"AMP_PER_VOLT"),
	GSCALAR(input_voltage,	"INPUT_VOLTS"),

    // @Param: BATT_CAPACITY
    // @DisplayName: Battery Capacity
    // @Description: Battery capacity in milliamp-hours (mAh)
    // @Units: mAh
	GSCALAR(pack_capacity,	"BATT_CAPACITY"),

    // @Param: MAG_ENABLE
    // @DisplayName: Enable Compass
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
	GSCALAR(compass_enabled,	"MAG_ENABLE"),

    // @Param: FLOW_ENABLE
    // @DisplayName: Enable Optical Flow
    // @Description: Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
	GSCALAR(optflow_enabled,	"FLOW_ENABLE"),

    // @Param: LOW_VOLT
    // @DisplayName: Low Voltage
    // @Description: Set this to the voltage you want to represent low voltage
    // @Range: 0 20
    // @Increment: .1
    // @User: Standard
	GSCALAR(low_voltage,	"LOW_VOLT"),

    // @Param: SUPER_SIMPLE
    // @DisplayName: Enable Super Simple Mode
    // @Description: Setting this to Enabled(1) will enable Super Simple Mode. Setting this to Disabled(0) will disable Super Simple Mode
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
	GSCALAR(super_simple,	"SUPER_SIMPLE"),

    // @Param: RTL_ALT_FINAL
    // @DisplayName: RTL Final Altitude
    // @Description: This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to -1 to disable, zero to land.
    // @Units: centimeters
    // @Range: -1 1000
    // @Increment: 1
    // @User: Standard
	GSCALAR(rtl_alt_final,	"RTL_ALT_FINAL"),

	// @Param: TILT
    // @DisplayName: Auto Tilt Compensation
    // @Description: This is a feed-forward compensation which helps the aircraft achieve target waypoint speed.
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
	GSCALAR(tilt_comp,	"TILT"),

    // @Param: BATT_VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 0:A0, 1:A1, 13:A13
    // @User: Standard
    GSCALAR(battery_volt_pin,    "BATT_VOLT_PIN"),

    // @Param: BATT_CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 1:A1, 2:A2, 13:A13
    // @User: Standard
    GSCALAR(battery_curr_pin,    "BATT_CURR_PIN"),

    // @Param: RSSI_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: This selects an analog pin for the receiver RSSI voltage. It assumes the voltage is 5V for max rssi, 0V for minimum
    // @Values: -1:Disabled, 0:A0, 1:A1, 2:A2, 13:A13
    // @User: Standard
    GSCALAR(rssi_pin,            "RSSI_PIN"),

    // @Param: THR_ACC_ENABLE
    // @DisplayName: Enable Accel based throttle controller
    // @Description: This allows enabling and disabling the accelerometer based throttle controller.  If disabled a velocity based controller is used.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(throttle_accel_enabled,  "THR_ACC_ENABLE"),

	GSCALAR(waypoint_mode,	"WP_MODE"),
	GSCALAR(command_total,	"WP_TOTAL"),
	GSCALAR(command_index,	"WP_INDEX"),
	GSCALAR(command_nav_index,	"WP_MUST_INDEX"),

    // @Param: WP_RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: Meters
    // @Range: 1 127
    // @Increment: 1
    // @User: Standard
	GSCALAR(waypoint_radius,	"WP_RADIUS"),

    // @Param: WP_LOITER_RAD
    // @DisplayName: Waypoint Loiter Radius
    // @Description: Defines the distance from the waypoint center, the vehicle will maintain during a loiter
    // @Units: Meters
    // @Range: 1 127
    // @Increment: 1
    // @User: Standard
	GSCALAR(loiter_radius,	"WP_LOITER_RAD"),

	// @Param: WP_SPEED_MAX
    // @DisplayName: Waypoint Max Speed Target
    // @Description: Defines the speed which the aircraft will attempt to maintain during a WP mission.
    // @Units: Centimeters/Second
    // @Increment: 100
    // @User: Standard
	GSCALAR(waypoint_speed_max,	"WP_SPEED_MAX"),

	// @Param: XTRK_GAIN_SC
    // @DisplayName: Cross-Track Gain
    // @Description: This controls the rate that the Auto Controller will attempt to return original track
    // @Units: Dimensionless
	// @User: Standard
	GSCALAR(crosstrack_gain,	"XTRK_GAIN_SC"),

    // @Param: XTRK_MIN_DIST
    // @DisplayName: Crosstrack mininum distance
    // @Description: Minimum distance in meters between waypoints to do crosstrack correction.
    // @Units: Meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(crosstrack_min_distance, "XTRK_MIN_DIST"),

    // @Param: RTL_LOITER_TIME
    // @DisplayName: RTL loiter time
    // @Description: Time (in milliseconds) to loiter above home before begining final descent
    // @Units: ms
    // @Range: 0 60000
    // @Increment: 1000
    // @User: Standard
    GSCALAR(rtl_loiter_time,      "RTL_LOIT_TIME"),

    // @Param: LAND_SPEED
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in cm/s
    // @Units: cm/s
    // @Range: 10 200
    // @Increment: 10
    // @User: Standard
	GSCALAR(land_speed,	"LAND_SPEED"),

    // @Param: THR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: The minimum throttle which the autopilot will apply.
    // @Units: ms
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_min,	"THR_MIN"),

    // @Param: THR_MAX
    // @DisplayName: Maximum Throttle
    // @Description: The maximum throttle which the autopilot will apply.
    // @Units: ms
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_max,	"THR_MAX"),

    // @Param: THR_FAILSAFE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
	GSCALAR(throttle_fs_enabled,	"THR_FAILSAFE"),

	GSCALAR(throttle_fs_action,	"THR_FS_ACTION"),

    // @Param: THR_FS_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers
    // @User: Standard
	GSCALAR(throttle_fs_value, "THR_FS_VALUE"),

	GSCALAR(throttle_cruise,	"TRIM_THROTTLE"),

	GSCALAR(flight_mode1, "FLTMODE1"),
	GSCALAR(flight_mode2, "FLTMODE2"),
	GSCALAR(flight_mode3, "FLTMODE3"),
	GSCALAR(flight_mode4, "FLTMODE4"),
	GSCALAR(flight_mode5, "FLTMODE5"),
	GSCALAR(flight_mode6, "FLTMODE6"),
	GSCALAR(simple_modes, "SIMPLE"),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: bitmap of log fields to enable
    // @User: Advanced
	GSCALAR(log_bitmask,	"LOG_BITMASK"),
	GSCALAR(log_last_filenumber, "LOG_LASTFILE"),
    // THOR
    // Added to allow change of Rate in the Mission planner
	GSCALAR(toy_yaw_rate, "TOY_RATE"),

	GSCALAR(esc_calibrate, "ESC"),
	GSCALAR(radio_tuning, "TUNE"),
	GSCALAR(radio_tuning_low, "TUNE_LOW"),
	GSCALAR(radio_tuning_high, "TUNE_HIGH"),
	GSCALAR(frame_orientation, "FRAME"),

    // @Param: CH7_OPT
    // @DisplayName: Channel 7 option
    // @Description: Select which function if performed when CH7 is high
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger
    // @User: Standard
	GSCALAR(ch7_option, "CH7_OPT"),

	// @Param: AUTO_SLEW
    // @DisplayName: Auto Slew Rate
    // @Description: This restricts the rate of change of the roll and pitch attitude commanded by the auto pilot
    // @Units: Degrees/Second
	// @Range: 1 45
    // @Increment: 1
    // @User: Advanced
	GSCALAR(auto_slew_rate, "AUTO_SLEW"),

#if FRAME_CONFIG ==     HELI_FRAME
    GGROUP(heli_servo_1,    "HS1_", RC_Channel),
    GGROUP(heli_servo_2,    "HS2_", RC_Channel),
    GGROUP(heli_servo_3,    "HS3_", RC_Channel),
    GGROUP(heli_servo_4,    "HS4_", RC_Channel),
	GSCALAR(heli_pitch_ff, "H_PITCH_FF",            HELI_PITCH_FF),
	GSCALAR(heli_roll_ff, "H_ROLL_FF",            HELI_ROLL_FF),
	GSCALAR(heli_yaw_ff, "H_YAW_FF",            HELI_YAW_FF),
#endif

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GGROUP(camera,                  "CAM_", AP_Camera),
#endif

    // RC channel
    //-----------
    // @Group: RC1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_1,    "RC1_", RC_Channel),
    // @Group: RC2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_2,    "RC2_", RC_Channel),
    // @Group: RC3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_3,    "RC3_", RC_Channel),
    // @Group: RC4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_4,    "RC4_", RC_Channel),
    // @Group: RC5_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_5,    "RC5_", RC_Channel_aux),
    // @Group: RC6_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_6,    "RC6_", RC_Channel_aux),
    // @Group: RC7_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_7,    "RC7_", RC_Channel_aux),
    // @Group: RC8_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_8,    "RC8_", RC_Channel_aux),

#if MOUNT == ENABLED
    // @Group: RC10_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_10,                    "RC10_", RC_Channel_aux),

    // @Group: RC11_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_11,                    "RC11_", RC_Channel_aux),
#endif

    // @Param: RC_SPEED
    // @DisplayName: ESC Update Speed
    // @Description: This is the speed in Hertz that your ESCs will receive updates
    // @Units: Hertz (Hz)
    // @Values: 125,400,490
    // @User: Advanced
	GSCALAR(rc_speed, "RC_SPEED"),

    // variable
    //---------
    //GSCALAR(stabilize_d,                    "STAB_D",  STABILIZE_D),

    // @Param: STAB_D_S
    // @DisplayName: Stabilize D Schedule
    // @Description: This value is a percentage of scheduling applied to the Stabilize D term.
    // @Range: 0 1
    // @Increment: .01
    // @User: Advanced
    //GSCALAR(stabilize_d_schedule, "STAB_D_S",  STABILIZE_D_SCHEDULE),

    // Acro parameters
	GSCALAR(acro_p, 			"ACRO_P"),
	GSCALAR(axis_lock_p, 		"AXIS_P"),
	GSCALAR(axis_enabled, 		"AXIS_ENABLE"),
    // @Param: ACRO_BAL_ROLL
    // @DisplayName: Acro Balance Roll
    // @Description: rate at which roll angle returns to level in acro mode
    // @Range: 0 300
    // @Increment: 1
    // @User: Advanced
    GSCALAR(acro_balance_roll,      "ACRO_BAL_ROLL"),

    // @Param: ACRO_BAL_PITCH
    // @DisplayName: Acro Balance Pitch
    // @Description: rate at which pitch angle returns to level in acro mode
    // @Range: 0 300
    // @Increment: 1
    // @User: Advanced
    GSCALAR(acro_balance_pitch,     "ACRO_BAL_PITCH"),

	GSCALAR(copter_leds_mode,	"LED_MODE"),

    // PID controller
    //---------------
    GGROUP(pid_rate_roll,     "RATE_RLL_", AC_PID),
    GGROUP(pid_rate_pitch,    "RATE_PIT_", AC_PID),
    GGROUP(pid_rate_yaw,      "RATE_YAW_", AC_PID),


    GGROUP(pid_loiter_rate_lat,      "LOITER_LAT_",  AC_PID),
    GGROUP(pid_loiter_rate_lon,      "LOITER_LON_",  AC_PID),

    GGROUP(pid_nav_lat,             "NAV_LAT_",  AC_PID),
    GGROUP(pid_nav_lon,             "NAV_LON_",  AC_PID),

    GGROUP(pid_throttle,      "THR_RATE_", AC_PID),
    GGROUP(pid_throttle_accel,"THR_ACCEL_", AC_PID),

    GGROUP(pid_optflow_roll,  "OF_RLL_",   AC_PID),
    GGROUP(pid_optflow_pitch, "OF_PIT_",   AC_PID),

    // PI controller
    //--------------
    GGROUP(pi_stabilize_roll,       "STB_RLL_", APM_PI),
    GGROUP(pi_stabilize_pitch,      "STB_PIT_", APM_PI),
    GGROUP(pi_stabilize_yaw,        "STB_YAW_", APM_PI),

    GGROUP(pi_alt_hold,             "THR_ALT_", APM_PI),
    GGROUP(pi_loiter_lat,   "HLD_LAT_", APM_PI),
    GGROUP(pi_loiter_lon,   "HLD_LON_", APM_PI),

    // variables not in the g class which contain EEPROM saved variables

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
#if HIL_MODE == HIL_MODE_DISABLED
    GOBJECT(ins,            "INS_", AP_InertialSensor),
#endif

#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
    // @Group: INAV_
    // @Path: ../libraries/AP_InertialNav/AP_InertialNav.cpp
    GOBJECT(inertial_nav,           "INAV_",    AP_InertialNav),
#endif

    GOBJECT(gcs0,                   "SR0_",     GCS_MAVLINK),
    GOBJECT(gcs3,                   "SR3_",     GCS_MAVLINK),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if MOUNT == ENABLED
    // @Group: MNT_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT_", AP_Mount),
#endif

#if MOUNT2 == ENABLED
    // @Group: MNT2_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount2,           "MNT2_",       AP_Mount),
#endif

#ifdef DESKTOP_BUILD
    GOBJECT(sitl, "SIM_", SITL),
#endif

    //@Group: LIM_
    //@Path: ../libraries/AP_Limits/AP_Limits.cpp,../libraries/AP_Limits/AP_Limit_GPSLock.cpp, ../libraries/AP_Limits/AP_Limit_Geofence.cpp, ../libraries/AP_Limits/AP_Limit_Altitude.cpp, ../libraries/AP_Limits/AP_Limit_Module.cpp
    GOBJECT(limits,                 "LIM_",    AP_Limits),
    GOBJECT(gpslock_limit,          "LIM_",    AP_Limit_GPSLock),
    GOBJECT(geofence_limit,         "LIM_",    AP_Limit_Geofence),
    GOBJECT(altitude_limit,         "LIM_",    AP_Limit_Altitude),

#if FRAME_CONFIG ==     HELI_FRAME
    // @Group: H_
    // @Path: ../libraries/AP_Motors/AP_MotorsHeli.cpp
    GOBJECT(motors, "H_",           AP_MotorsHeli),
#else
    GOBJECT(motors, "MOT_",         AP_Motors),
#endif
};


static void load_parameters(void)
{
	// setup the AP_Var subsystem for storage to EEPROM
	if (!AP_Param::setup(var_info, sizeof(var_info)/sizeof(var_info[0]), WP_START_BYTE,&Serial)) {
		// this can only happen on startup, and its a definate coding
		// error. Best not to continue so the programmer catches it
		while (1) {
			Serial.println_P(PSTR("ERROR: Failed to setup AP_Param"));
			delay(1000);
		}
	}


	if (!g.format_version.load() ||
	    g.format_version != Parameters::k_format_version) {

		// erase all parameters
		Serial.printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
		AP_Param::erase_all();

		// save the current format version
		g.format_version.set_and_save(Parameters::k_format_version);
		default_dead_zones();
		// change the default for the AHRS_GPS_GAIN for ArduCopter
		// if it hasn't been set by the user
		ahrs.gps_gain.set_and_save(0.0);
		
		// setup different AHRS gains for ArduCopter than the default
		// but allow users to override in their config
		ahrs._kp.set_and_save(0.1);
		ahrs._kp_yaw.set_and_save(0.1);
		
	#if SECONDARY_DMP_ENABLED == ENABLED
    if (!ahrs2._kp.load()) {
        ahrs2._kp.set(0.1);
    }
    if (!ahrs2._kp_yaw.load()) {
        ahrs2._kp_yaw.set(0.1);
    }
	#endif
		Serial.println_P(PSTR("done."));
	} else {
		uint32_t before = micros();
		// Load all auto-loaded EEPROM variables
		AP_Param::load_all();

		Serial.printf_P(PSTR("load_all took %luus\n"), micros() - before);
	}
}
