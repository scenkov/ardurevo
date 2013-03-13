#line 1 "./Firmware/APMRover2/APMrover2.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduRover v2.30"

// This is the APMrover firmware derived from the Arduplane v2.32 by Jean-Louis Naudin (JLN) 
/*
Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Jean-Louis Naudin
Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier 
Please contribute your ideas!
APMrover alpha version tester: Franco Borasio, Daniel Chapelat... 

This firmware is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
//
// JLN updates: last update 2012-06-21
// DOLIST:
//-------------------------------------------------------------------------------------------------------------------------
// Dev Startup : 2012-04-21
//
//  2012-06-21: Update for HIL mode with mavlink 1.0 (new lib)
//  2012-06-13: use RangeFinder optical SharpGP2Y instead of ultrasonic sonar
//  2012-06-13: added Test sonar
//  2012-05-17: added speed_boost during straight line
//  2012-05-17: New update about the throttle rate control based on the field test done by Franco Borasio (Thanks Franco..)
//  2012-05-15: The Throttle rate can be controlled by the THROTTLE_SLEW_LIMIT (the value give the step increase, 1 = 0.1)
//  2012-05-14: Update about mavlink library (now compatible with the latest version of mavlink)
//  2012-05-14: Added option (hold roll to full right + SW7 ON/OFF) to init_home during the wp_list reset
//  2012-05-13: Add ROV_SONAR_TRIG (default = 200 cm)
//  2012-05-13: Restart_nav() added and heading bug correction, tested OK in the field
//  2012-05-12: RTL then stop update - Tested in the field
//  2012-05-11: The rover now STOP after the RTL... (special update for Franco...)
//  2012-05-11: Added SONAR detection for obstacle avoidance (alpha version for SONAR testing)
//  2012-05-04: Added #define LITE ENABLED  for the APM1280 or APM2560 CPU IMUless version
//  2012-05-03: Successful missions tests with a full APM2560 kit (GPS MT3329 + magnetometer HMC5883L)
//  2012-05-03: removing stick mixing in auto mode
//  2012-05-01: special update for rover about ground_course if compass is enabled
//  2012-04-30: Successfully tested in autonomous nav with a waypoints list recorded in live mode
//  2012-04-30: Now a full version for APM v1 or APM v2 with magnetometer
//  2012-04-27: Cosmetic changes
//  2012-04-26: Only one PID (pidNavRoll) for steering the wheel with nav_steer
//  2012-04-26: Added ground_speed and ground_course variables in Update_GPS
//  2012-04-26: Set GPS to 10 Hz (updated in the AP_GPS lib)
//  2012-04-22: Tested on Traxxas Monster Jam Grinder XL-5 3602
//  2012-04-21: Roll set to wheels control and Throttle neutral to 50% (0 -100)  - Forward>50, Backward<50
//
// Radio setup:
// APM INPUT (Rec = receiver)
// Rec ch1: Roll 
// Rec ch2: Throttle
// Rec ch3: Pitch
// Rec ch4: Yaw
// Rec ch5: not used
// Rec ch6: not used
// Rec ch7: Option channel to 2 positions switch
// Rec ch8: Mode channel to 3 positions switch
// APM OUTPUT
// Ch1: Wheel servo (direction)
// Ch2: not used
// Ch3: to the motor ESC
// Ch4: not used
//
// more infos about this experimental version: http://diydrones.com/profile/JeanLouisNaudin
// =======================================================================================================
*/

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

// Libraries
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <AP_GPS.h>         // ArduPilot GPS library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor.h> // Inertial Sensor (uncalibated IMU) Library
#include <AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <PID.h>            // PID library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder.h>	// Range finder library
#include <Filter.h>			// Filter library
#include <Butter.h>			// Filter library - butterworth filter
#include <AP_Buffer.h>      // FIFO buffer library
#include <ModeFilter.h>		// Mode Filter from Filter library
#include <AverageFilter.h>	// Mode Filter from Filter library
#include <AP_Relay.h>       // APM relay
#include <AP_Mount.h>		// Camera/Antenna mount
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Airspeed.h>    // needed for AHRS build
#include <memcheck.h>
#include <DataFlash.h>
#include <SITL.h>
#include <stdarg.h>

#include <AP_HAL_VRBRAIN.h>
#include "compat.h"

// Configuration
#include "config.h"

// Local modules
#include "defines.h"
#include "Parameters.h"
#include "GCS.h"

#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library

  void setup() ;
  void loop() ;
 static void fast_loop() ;
  static void medium_loop() ;
  static void slow_loop() ;
  static void one_second_loop() ;
  static void update_GPS(void) ;
  static void update_current_mode(void) ;
  static void update_navigation() ;
  static NOINLINE void send_heartbeat(mavlink_channel_t chan) ;
  static NOINLINE void send_attitude(mavlink_channel_t chan) ;
  static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops) ;
  static void NOINLINE send_meminfo(mavlink_channel_t chan) ;
  static void NOINLINE send_location(mavlink_channel_t chan) ;
  static void NOINLINE send_nav_controller_output(mavlink_channel_t chan) ;
  static void NOINLINE send_gps_raw(mavlink_channel_t chan) ;
  static void NOINLINE send_servo_out(mavlink_channel_t chan) ;
  static void NOINLINE send_radio_in(mavlink_channel_t chan) ;
  static void NOINLINE send_radio_out(mavlink_channel_t chan) ;
  static void NOINLINE send_vfr_hud(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu1(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu3(mavlink_channel_t chan) ;
  static void NOINLINE send_ahrs(mavlink_channel_t chan) ;
 static void NOINLINE send_simstate(mavlink_channel_t chan) ;
  static void NOINLINE send_hwstatus(mavlink_channel_t chan) ;
  static void NOINLINE send_current_waypoint(mavlink_channel_t chan) ;
  static void NOINLINE send_statustext(mavlink_channel_t chan) ;
 static bool telemetry_delayed(mavlink_channel_t chan) ;
 static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops) ;
 static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops) ;
  void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str) ;
 static void mavlink_delay_cb() ;
 static void gcs_send_message(enum ap_message id) ;
 static void gcs_data_stream_send(void) ;
 static void gcs_update(void) ;
  static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str) ;
  static bool print_log_menu(void) ;
   void erase_callback(unsigned long t) ;
  static void do_erase_logs(void) ;
 static void Log_Write_Attitude(int16_t log_roll, int16_t log_pitch, uint16_t log_yaw) ;
 static void Log_Read_Attitude() ;
 static void Log_Write_Performance() ;
 static void Log_Read_Performance() ;
 static void Log_Write_Cmd(uint8_t num, struct Location *wp) ;
 static void Log_Read_Cmd() ;
  static void Log_Write_Startup(uint8_t type) ;
  static void Log_Read_Startup() ;
 static void Log_Write_Control_Tuning() ;
 static void Log_Read_Control_Tuning() ;
 static void Log_Write_Nav_Tuning() ;
 static void Log_Read_Nav_Tuning() ;
 static void Log_Write_Mode(uint8_t mode) ;
 static void Log_Read_Mode() ;
 static void Log_Write_GPS(	uint32_t log_Time, int32_t log_Lattitude, int32_t log_Longitude, int32_t log_gps_alt, int32_t log_mix_alt,                             uint32_t log_Ground_Speed, int32_t log_Ground_Course, uint8_t log_Fix, uint8_t log_NumSats) ;
 static void Log_Read_GPS() ;
 static void Log_Write_IMU() ;
 static void Log_Read_IMU() ;
  static void Log_Write_Current() ;
 static void Log_Read_Current() ;
 static void Log_Read(int16_t start_page, int16_t end_page) ;
 static void log_callback(uint8_t msgid) ;
 static void Log_Write_Mode(uint8_t mode) ;
 static void Log_Write_Startup(uint8_t type) ;
 static void Log_Write_Cmd(uint8_t num, struct Location *wp) ;
 static void Log_Write_Current() ;
 static void Log_Write_Nav_Tuning() ;
 static void Log_Write_GPS(	uint32_t log_Time, int32_t log_Lattitude, int32_t log_Longitude, int32_t log_gps_alt, int32_t log_mix_alt,                             uint32_t log_Ground_Speed, int32_t log_Ground_Course, uint8_t log_Fix, uint8_t log_NumSats) ;
 static void Log_Write_Performance() ;
 static void Log_Write_Attitude(int16_t log_roll, int16_t log_pitch, uint16_t log_yaw) ;
 static void Log_Write_Control_Tuning() ;
 static void Log_Write_IMU() ;
   static void load_parameters(void) ;
 static void throttle_slew_limit(int16_t last_throttle) ;
  static void calc_throttle() ;
  static void calc_nav_steer() ;
 static void set_servos(void) ;
  static void demo_servos(uint8_t i) ;
  static void init_commands() ;
 static struct Location get_cmd_with_index(int i) ;
 static void set_cmd_with_index(struct Location temp, int i) ;
 static void set_next_WP(struct Location *wp) ;
  static void set_guided_WP(void) ;
 void init_home() ;
  static void restart_nav() ;
 static void handle_process_nav_cmd() ;
  static void handle_process_condition_command() ;
  static void handle_process_do_command() ;
  static void handle_no_commands() ;
  static void do_RTL(void) ;
  static void do_takeoff() ;
  static void do_nav_wp() ;
 static bool verify_takeoff() ;
  static bool verify_nav_wp() ;
  static bool verify_RTL() ;
  static void do_wait_delay() ;
  static void do_change_alt() ;
  static void do_within_distance() ;
  static bool verify_wait_delay() ;
  static bool verify_change_alt() ;
  static bool verify_within_distance() ;
  static void do_jump() ;
  static void do_change_speed() ;
  static void do_set_home() ;
  static void do_set_servo() ;
  static void do_set_relay() ;
  static void do_repeat_servo() ;
  static void do_repeat_relay() ;
 static void change_command(uint8_t cmd_index) ;
 static void update_commands(void) ;
  static void verify_commands(void) ;
   static void process_next_command() ;
 static void process_nav_cmd() ;
  static void process_non_nav_command() ;
  void delay(uint32_t ms) ;
  void mavlink_delay(uint32_t ms) ;
  uint32_t millis() ;
  uint32_t micros() ;
  void pinMode(uint8_t pin, uint8_t output) ;
  void digitalWrite(uint8_t pin, uint8_t out) ;
  uint8_t digitalRead(uint8_t pin) ;
  static void read_control_switch() ;
  static uint8_t readSwitch(void);
  static void reset_control_switch() ;
 static void read_trim_switch() ;
   static void failsafe_long_on_event(int fstype) ;
 void failsafe_check(uint32_t tnow) ;
 static void navigate() ;
   static void calc_bearing_error() ;
  static long wrap_360(long error) ;
  static long wrap_180(long error) ;
  static void update_crosstrack(void) ;
  static void reset_crosstrack() ;
  void reached_waypoint() ;
   static void init_rc_in() ;
  static void init_rc_out() ;
  static void read_radio() ;
  static void control_failsafe(uint16_t pwm) ;
  static void trim_control_surfaces() ;
  static void trim_radio() ;
 static void init_sonar(void) ;
  void ReadSCP1000(void) ;
  static void read_battery(void) ;
  static void report_batt_monitor() ;
 static void report_radio() ;
  static void report_gains() ;
  static void report_xtrack() ;
  static void report_throttle() ;
  static void report_compass() ;
  static void report_modes() ;
  static void print_PID(PID * pid) ;
  static void print_radio_values() ;
  static void print_switch(uint8_t p, uint8_t m) ;
  static void print_done() ;
  static void print_blanks(int num) ;
  static void print_divider(void) ;
  static int8_t radio_input_switch(void) ;
   static void zero_eeprom(void) ;
  static void print_enabled(bool b) ;
  static void init_ardupilot() ;
 static void startup_ground(void) ;
  static void set_mode(enum mode mode) ;
  static void check_long_failsafe() ;
 static void startup_INS_ground(bool force_accel_level) ;
  static void update_GPS_light(void) ;
   static void resetPerfData(void) ;
 static uint32_t map_baudrate(int8_t rate, uint32_t default_baud) ;
 static void check_usb_mux(void) ;
 void flash_leds(bool on) ;
 uint16_t board_voltage(void) ;
  static void print_mode(uint8_t mode) ;
 static void reboot_apm(void) ;
  static void print_hit_enter() ;
  static void test_wp_print(struct Location *cmd, uint8_t wp_index) ;
#line 120 "./Firmware/APMRover2/APMrover2.pde"
AP_HAL::BetterStream* cliSerial;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// this sets up the parameter table, and sets the default values. This
// must be the first AP_Param variable declared to ensure its
// constructor runs before the constructors of the other AP_Param
// variables
AP_Param param_loader(var_info, WP_START_BYTE);

////////////////////////////////////////////////////////////////////////////////
// the rate we run the main loop at
////////////////////////////////////////////////////////////////////////////////
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_50HZ;

////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters      g;


////////////////////////////////////////////////////////////////////////////////
// prototypes
static void update_events(void);

////////////////////////////////////////////////////////////////////////////////
// DataFlash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
DataFlash_SITL DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
DataFlash_MP32 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
DataFlash_Empty DataFlash;
#endif


////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal flight mode.  Real sensors are used.
// - HIL Attitude mode.  Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode.  Synthetic sensors are configured that
//   supply data from the simulation.
//

// All GPS access should be through this pointer.
static GPS         *g_gps;

// flight modes convenience array
static AP_Int8		*modes = &g.mode1;

#if HIL_MODE == HIL_MODE_DISABLED

// real sensors
#if CONFIG_ADC == ENABLED
static AP_ADC_ADS7844          adc;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
static AP_Compass_HIL compass;
static SITL sitl;
#else
static AP_Compass_HMC5843      compass;
#endif

// real GPS selection
#if   GPS_PROTOCOL == GPS_PROTOCOL_AUTO
AP_GPS_Auto     g_gps_driver(&g_gps);

#elif GPS_PROTOCOL == GPS_PROTOCOL_NMEA
AP_GPS_NMEA     g_gps_driver();

#elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
AP_GPS_SIRF     g_gps_driver();

#elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
AP_GPS_UBLOX    g_gps_driver();

#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
AP_GPS_MTK      g_gps_driver();

#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK19
AP_GPS_MTK19    g_gps_driver();

#elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_None     g_gps_driver();

#else
 #error Unrecognised GPS_PROTOCOL setting.
#endif // GPS PROTOCOL

# if CONFIG_INS_TYPE == CONFIG_INS_MPU6000
  AP_InertialSensor_MPU6000 ins;
# elif CONFIG_INS_TYPE == CONFIG_INS_SITL
  AP_InertialSensor_Stub ins;
#else
 AP_InertialSensor_MPU6000 ins;
#endif // CONFIG_INS_TYPE

AP_AHRS_DCM  ahrs(&ins, g_gps);

#elif HIL_MODE == HIL_MODE_SENSORS
// sensor emulators
AP_ADC_HIL              adc;
AP_Compass_HIL          compass;
AP_GPS_HIL              g_gps_driver(NULL);
AP_InertialSensor_Stub  ins;
AP_AHRS_DCM  ahrs(&ins, g_gps);

#elif HIL_MODE == HIL_MODE_ATTITUDE
AP_ADC_HIL              adc;
AP_InertialSensor_Stub  ins;
AP_AHRS_HIL             ahrs(&ins, g_gps);
AP_GPS_HIL              g_gps_driver(NULL);
AP_Compass_HIL          compass; // never used
#else
 #error Unrecognised HIL_MODE setting.
#endif // HIL MODE

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
//
GCS_MAVLINK	gcs0;
GCS_MAVLINK	gcs3;

// a pin for reading the receiver RSSI voltage. The scaling by 0.25 
// is to take the 0 to 1024 range down to an 8 bit range for MAVLink
AP_HAL::AnalogSource *rssi_analog_source;

AP_HAL::AnalogSource *vcc_pin;

AP_HAL::AnalogSource * batt_volt_pin;
AP_HAL::AnalogSource * batt_curr_pin;

////////////////////////////////////////////////////////////////////////////////
// SONAR selection
////////////////////////////////////////////////////////////////////////////////
//
ModeFilterInt16_Size5 sonar_mode_filter(2);
#if CONFIG_SONAR == ENABLED
    AP_HAL::AnalogSource *sonar_analog_source;
    AP_RangeFinder_MaxsonarXL *sonar;
#endif

// relay support
AP_Relay relay;

// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
AP_Mount camera_mount(g_gps, &dcm);
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

// APM2 only
#if USB_MUX_PIN > 0
static bool usb_connected;
#endif

/* Radio values
		Channel assignments
			1   Steering
			2   ---
			3   Throttle
			4   ---
			5   Aux5
			6   Aux6
			7   Aux7
			8   Aux8/Mode
		Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
		See libraries/RC_Channel/RC_Channel_aux.h for more information
*/

////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as MANUAL, FBW-A, AUTO
enum mode   control_mode        = INITIALISING;
// Used to maintain the state of the previous control switch position
// This is set to -1 when we need to re-read the switch
uint8_t 	oldSwitchPosition;
// These are values received from the GCS if the user is using GCS joystick
// control and are substituted for the values coming from the RC radio
static int16_t rc_override[8] = {0,0,0,0,0,0,0,0};
// A flag if GCS joystick control is in use
static bool rc_override_active = false;

////////////////////////////////////////////////////////////////////////////////
// Failsafe
////////////////////////////////////////////////////////////////////////////////
// A tracking variable for type of failsafe active
// Used for failsafe based on loss of RC signal or GCS signal
static int16_t 	failsafe;					
// Used to track if the value on channel 3 (throtttle) has fallen below the failsafe threshold
// RC receiver should be set up to output a low throttle value when signal is lost
static bool 	ch3_failsafe;

// A timer used to track how long since we have received the last GCS heartbeat or rc override message
static uint32_t rc_override_fs_timer = 0;
// A timer used to track how long we have been in a "short failsafe" condition due to loss of RC signal
static uint32_t ch3_failsafe_timer = 0;

////////////////////////////////////////////////////////////////////////////////
// LED output
////////////////////////////////////////////////////////////////////////////////
// state of the GPS light (on/off)
static bool GPS_light;							

////////////////////////////////////////////////////////////////////////////////
// GPS variables
////////////////////////////////////////////////////////////////////////////////
// This is used to scale GPS values for EEPROM storage
// 10^7 times Decimal GPS means 1 == 1cm
// This approximation makes calculations integer and it's easy to read
static const 	float t7			= 10000000.0;	
// We use atan2 and other trig techniques to calaculate angles

// A counter used to count down valid gps fixes to allow the gps estimate to settle
// before recording our home position (and executing a ground start if we booted with an air start)
static uint8_t 	ground_start_count	= 5;
// Used to compute a speed estimate from the first valid gps fixes to decide if we are 
// on the ground or in the air.  Used to decide if a ground start is appropriate if we
// booted with an air start.
static int16_t     ground_start_avg;
static int32_t          gps_base_alt;		

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// Constants
const	float radius_of_earth 	= 6378100;	// meters


// true if we have a position estimate from AHRS
static bool have_position;

// This is the currently calculated direction to fly.  
// deg * 100 : 0 to 360
static int32_t nav_bearing;
// This is the direction to the next waypoint
// deg * 100 : 0 to 360
static int32_t target_bearing;	
//This is the direction from the last waypoint to the next waypoint 
// deg * 100 : 0 to 360
static int32_t crosstrack_bearing;
// A gain scaler to account for ground speed/headwind/tailwind
static float	nav_gain_scaler 		= 1;		
static bool rtl_complete = false;

// There may be two active commands in Auto mode.  
// This indicates the active navigation command by index number
static uint8_t	nav_command_index;					
// This indicates the active non-navigation command by index number
static uint8_t	non_nav_command_index;				
// This is the command type (eg navigate to waypoint) of the active navigation command
static uint8_t	nav_command_ID		= NO_COMMAND;	
static uint8_t	non_nav_command_ID	= NO_COMMAND;	

// ground speed error in m/s
static float	groundspeed_error;	
// 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel
static int16_t     throttle_nudge = 0;

// receiver RSSI
static uint8_t receiver_rssi;

// the time when the last HEARTBEAT message arrived from a GCS
static uint32_t last_heartbeat_ms;

// The distance as reported by Sonar in cm – Values are 20 to 700 generally.
static int16_t		sonar_dist;
static bool obstacle = false;

////////////////////////////////////////////////////////////////////////////////
// Ground speed
////////////////////////////////////////////////////////////////////////////////
// The amount current ground speed is below min ground speed.  meters per second
static float 	ground_speed = 0;
static int16_t throttle_last = 0, throttle = 500;

////////////////////////////////////////////////////////////////////////////////
// Location Errors
////////////////////////////////////////////////////////////////////////////////
// Difference between current bearing and desired bearing.  in centi-degrees
static int32_t bearing_error_cd;
// Difference between current altitude and desired altitude.  Centimeters
static int32_t altitude_error;
// Distance perpandicular to the course line that we are off trackline.  Meters 
static float	crosstrack_error;

////////////////////////////////////////////////////////////////////////////////
// CH7 control
////////////////////////////////////////////////////////////////////////////////

// Used to track the CH7 toggle state.
// When CH7 goes LOW PWM from HIGH PWM, this value will have been set true
// This allows advanced functionality to know when to execute
static bool ch7_flag;
// This register tracks the current Mission Command index when writing
// a mission using CH7 in flight
static int8_t CH7_wp_index;
float tuning_value;

////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
// Battery pack 1 voltage.  Initialized above the low voltage threshold to pre-load the filter and prevent low voltage events at startup.
static float 	battery_voltage1 	= LOW_VOLTAGE * 1.05;
// Battery pack 1 instantaneous currrent draw.  Amperes
static float	current_amps1;
// Totalized current (Amp-hours) from battery 1
static float	current_total1;									

// To Do - Add support for second battery pack
//static float 	battery_voltage2 	= LOW_VOLTAGE * 1.05;		// Battery 2 Voltage, initialized above threshold for filter
//static float	current_amps2;									// Current (Amperes) draw from battery 2
//static float	current_total2;									// Totalized current (Amp-hours) from battery 2

// JLN Update
uint32_t  timesw                  = 0;

////////////////////////////////////////////////////////////////////////////////
// Navigation control variables
////////////////////////////////////////////////////////////////////////////////
// The instantaneous desired bank angle.  Hundredths of a degree
static int32_t nav_steer;

////////////////////////////////////////////////////////////////////////////////
// Waypoint distances
////////////////////////////////////////////////////////////////////////////////
// Distance between plane and next waypoint.  Meters
static float wp_distance;
// Distance between previous and next waypoint.  Meters
static int32_t wp_totalDistance;

////////////////////////////////////////////////////////////////////////////////
// repeating event control
////////////////////////////////////////////////////////////////////////////////
// Flag indicating current event type
static uint8_t 		event_id;
// when the event was started in ms
static int32_t 		event_timer;
// how long to delay the next firing of event in millis
static uint16_t 	event_delay;					
// how many times to cycle : -1 (or -2) = forever, 2 = do one cycle, 4 = do two cycles
static int16_t 		event_repeat = 0;
// per command value, such as PWM for servos
static int16_t 		event_value; 
// the value used to cycle events (alternate value to event_value)
static int16_t 		event_undo_value;

////////////////////////////////////////////////////////////////////////////////
// Conditional command
////////////////////////////////////////////////////////////////////////////////
// A value used in condition commands (eg delay, change alt, etc.)
// For example in a change altitude command, it is the altitude to change to.
static int32_t 	condition_value;
// A starting value used to check the status of a conditional command.
// For example in a delay command the condition_start records that start time for the delay
static int32_t 	condition_start;
// A value used in condition commands.  For example the rate at which to change altitude.
static int16_t 		condition_rate;

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
// Location structure defined in AP_Common
////////////////////////////////////////////////////////////////////////////////
// The home location used for RTL.  The location is set when we first get stable GPS lock
static struct 	Location home;
// Flag for if we have g_gps lock and have set the home location
static bool	home_is_set;
// The location of the previous waypoint.  Used for track following and altitude ramp calculations
static struct 	Location prev_WP;
// The plane's current location
static struct 	Location current_loc;
// The location of the current/active waypoint.  Used for track following
static struct 	Location next_WP;
// The location of the active waypoint in Guided mode.
static struct  	Location guided_WP;

// The location structure information from the Nav command being processed
static struct 	Location next_nav_command;	
// The location structure information from the Non-Nav command being processed
static struct 	Location next_nonnav_command;

////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// The main loop execution time.  Seconds
//This is the time between calls to the DCM algorithm and is the Integration time for the gyros.
static float G_Dt						= 0.02;		

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
// Timer used to accrue data and trigger recording of the performanc monitoring log message
static int32_t 	perf_mon_timer;
// The maximum main loop execution time recorded in the current performance monitoring interval
static int16_t 	G_Dt_max = 0;
// The number of gps fixes recorded in the current performance monitoring interval
static uint8_t 	gps_fix_count = 0;
// A variable used by developers to track performanc metrics.
// Currently used to record the number of GCS heartbeat messages received
static int16_t pmTest1 = 0;


////////////////////////////////////////////////////////////////////////////////
// System Timers
////////////////////////////////////////////////////////////////////////////////
// Time in miliseconds of start of main control loop.  Milliseconds
static uint32_t 	fast_loopTimer;
// Time Stamp when fast loop was complete.  Milliseconds
static uint32_t 	fast_loopTimeStamp;
// Number of milliseconds used in last main loop cycle
static uint8_t 		delta_ms_fast_loop;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t			mainLoop_count;

// Time in miliseconds of start of medium control loop.  Milliseconds
static uint32_t 	medium_loopTimer;
// Counters for branching from main control loop to slower loops
static uint8_t 			medium_loopCounter;	
// Number of milliseconds used in last medium loop cycle
static uint8_t			delta_ms_medium_loop;

// Counters for branching from medium control loop to slower loops
static uint8_t 			slow_loopCounter;
// Counter to trigger execution of very low rate processes
static uint8_t 			superslow_loopCounter;
// Counter to trigger execution of 1 Hz processes
static uint8_t				counter_one_herz;

// % MCU cycles used
static float 			load;

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

void setup() {
	memcheck_init();
    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    rssi_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE, 0.25);
    vcc_pin = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);
    batt_volt_pin = hal.analogin->channel(g.battery_volt_pin);
    batt_curr_pin = hal.analogin->channel(g.battery_curr_pin);

#if CONFIG_SONAR == ENABLED
 #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
    sonar_analog_source = new AP_ADC_AnalogSource(
        &adc, CONFIG_SONAR_SOURCE_ADC_CHANNEL, 0.25);
 #elif CONFIG_SONAR_SOURCE == SONAR_SOURCE_ANALOG_PIN
    sonar_analog_source = hal.analogin->channel(
        CONFIG_SONAR_SOURCE_ANALOG_PIN);
 #else
  #warning "Invalid CONFIG_SONAR_SOURCE"
 #endif
    sonar = new AP_RangeFinder_MaxsonarXL(sonar_analog_source,
                                          &sonar_mode_filter);
#endif

	init_ardupilot();
}

void loop()
{
    // We want this to execute at 50Hz, but synchronised with the gyro/accel
    uint16_t num_samples = ins.num_samples_available();
    if (num_samples >= 1) {
		delta_ms_fast_loop	= millis() - fast_loopTimer;
		load                = (float)(fast_loopTimeStamp - fast_loopTimer)/delta_ms_fast_loop;
		G_Dt                = (float)delta_ms_fast_loop / 1000.f;
		fast_loopTimer      = millis();

		mainLoop_count++;

		// Execute the fast loop
		// ---------------------
		fast_loop();

		// Execute the medium loop
		// -----------------------
		medium_loop();

		counter_one_herz++;
		if(counter_one_herz == 50){
			one_second_loop();
			counter_one_herz = 0;
		}

		if (millis() - perf_mon_timer > 20000) {
			if (mainLoop_count != 0) {
  #if LITE == DISABLED
				if (g.log_bitmask & MASK_LOG_PM)
					#if HIL_MODE != HIL_MODE_ATTITUDE
					Log_Write_Performance();
					#endif
 #endif
				resetPerfData();
			}
		}

		fast_loopTimeStamp = millis();
    } else if (millis() - fast_loopTimeStamp < 19) {
        // less than 19ms has passed. We have at least one millisecond
        // of free time. The most useful thing to do with that time is
        // to accumulate some sensor readings, specifically the
        // compass, which is often very noisy but is not interrupt
        // driven, so it can't accumulate readings by itself
        if (g.compass_enabled) {
            compass.accumulate();
        }
    }
}

// Main loop 50Hz
static void fast_loop()
{
	// This is the fast loop - we want it to execute at 50Hz if possible
	// -----------------------------------------------------------------
	if (delta_ms_fast_loop > G_Dt_max)
		G_Dt_max = delta_ms_fast_loop;

	// Read radio
	// ----------
	read_radio();

    // try to send any deferred messages if the serial port now has
    // some space available
    gcs_send_message(MSG_RETRY_DEFERRED);

	#if HIL_MODE == HIL_MODE_SENSORS
		// update hil before dcm update
		gcs_update();
	#endif

#if LITE == DISABLED
	ahrs.update();
#endif 
	// Read Sonar
	// ----------
#if CONFIG_SONAR == ENABLED
	if(g.sonar_enabled){
		sonar_dist = sonar->read();

	if(sonar_dist <= g.sonar_trigger)  {  // obstacle detected in front 
            obstacle = true;
      } else  { 
            obstacle = false;
            }
	}
#endif

	// uses the yaw from the DCM to give more accurate turns
	calc_bearing_error();

#if LITE == DISABLED
	# if HIL_MODE == HIL_MODE_DISABLED
		if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST)
			Log_Write_Attitude((int)ahrs.roll_sensor, (int)ahrs.pitch_sensor, (uint16_t)ahrs.yaw_sensor);

		if (g.log_bitmask & MASK_LOG_IMU)
			Log_Write_IMU();
	#endif
#endif
	// inertial navigation
	// ------------------
	#if INERTIAL_NAVIGATION == ENABLED
		// TODO: implement inertial nav function
		inertialNavigation();
	#endif

	// custom code/exceptions for flight modes
	// ---------------------------------------
	update_current_mode();

	// apply desired steering if in an auto mode
	if (control_mode >= AUTO) {
        g.channel_steer.servo_out = nav_steer;
    }

	// write out the servo PWM values
	// ------------------------------
	set_servos();


	// XXX is it appropriate to be doing the comms below on the fast loop?

    gcs_update();
    gcs_data_stream_send();
}

static void medium_loop()
{
#if MOUNT == ENABLED
	camera_mount.update_mount_position();
#endif

	// This is the start of the medium (10 Hz) loop pieces
	// -----------------------------------------
	switch(medium_loopCounter) {

		// This case deals with the GPS
		//-------------------------------
		case 0:
			medium_loopCounter++;
            update_GPS();
            
//#if LITE == DISABLED
			#if HIL_MODE != HIL_MODE_ATTITUDE
            if (g.compass_enabled && compass.read()) {
                ahrs.set_compass(&compass);
                // Calculate heading
                compass.null_offsets();
            } else {
                ahrs.set_compass(NULL);
            }
			#endif
//#endif
/*{
cliSerial->print(ahrs.roll_sensor, DEC);	cliSerial->printf_P(PSTR("\t"));
cliSerial->print(ahrs.pitch_sensor, DEC);	cliSerial->printf_P(PSTR("\t"));
cliSerial->print(ahrs.yaw_sensor, DEC);	cliSerial->printf_P(PSTR("\t"));
Vector3f tempaccel = ins.get_accel();
cliSerial->print(tempaccel.x, DEC);	cliSerial->printf_P(PSTR("\t"));
cliSerial->print(tempaccel.y, DEC);	cliSerial->printf_P(PSTR("\t"));
cliSerial->println(tempaccel.z, DEC);
}*/

			break;

		// This case performs some navigation computations
		//------------------------------------------------
		case 1:
			medium_loopCounter++;
            navigate();
			break;

		// command processing
		//------------------------------
		case 2:
			medium_loopCounter++;

			// perform next command
			// --------------------
			update_commands();
			break;

		// This case deals with sending high rate telemetry
		//-------------------------------------------------
		case 3:
			medium_loopCounter++;
#if LITE == DISABLED
			#if HIL_MODE != HIL_MODE_ATTITUDE
				if ((g.log_bitmask & MASK_LOG_ATTITUDE_MED) && !(g.log_bitmask & MASK_LOG_ATTITUDE_FAST))
					Log_Write_Attitude((int)ahrs.roll_sensor, (int)ahrs.pitch_sensor, (uint16_t)ahrs.yaw_sensor);

				if (g.log_bitmask & MASK_LOG_CTUN)
					Log_Write_Control_Tuning();
			#endif

			if (g.log_bitmask & MASK_LOG_NTUN)
				Log_Write_Nav_Tuning();

			if (g.log_bitmask & MASK_LOG_GPS)
				Log_Write_GPS(g_gps->time, current_loc.lat, current_loc.lng, g_gps->altitude, current_loc.alt, g_gps->ground_speed, g_gps->ground_course, g_gps->fix, g_gps->num_sats);
#endif
			break;

		// This case controls the slow loop
		//---------------------------------
		case 4:
			medium_loopCounter = 0;
			delta_ms_medium_loop	= millis() - medium_loopTimer;
			medium_loopTimer      	= millis();

			if (g.battery_monitoring != 0){
				read_battery();
			}

			read_trim_switch();

			slow_loop();
			break;
	}
}

static void slow_loop()
{
	// This is the slow (3 1/3 Hz) loop pieces
	//----------------------------------------
	switch (slow_loopCounter){
		case 0:
			slow_loopCounter++;
			check_long_failsafe();
			superslow_loopCounter++;
			if(superslow_loopCounter >=200) {				//	200 = Execute every minute
#if LITE == DISABLED
				#if HIL_MODE != HIL_MODE_ATTITUDE
					if(g.compass_enabled) {
						compass.save_offsets();
					}
				#endif
#endif
				superslow_loopCounter = 0;
			}
			break;

		case 1:
			slow_loopCounter++;

			// Read 3-position switch on radio
			// -------------------------------
			read_control_switch();

			update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8);

#if MOUNT == ENABLED
			camera_mount.update_mount_type();
#endif
			break;

		case 2:
			slow_loopCounter = 0;
                        
			update_events();

            mavlink_system.sysid = g.sysid_this_mav;		// This is just an ugly hack to keep mavlink_system.sysid sync'd with our parameter

#if USB_MUX_PIN > 0
            check_usb_mux();
#endif

			break;
	}
}

static void one_second_loop()
{
#if LITE == DISABLED
	if (g.log_bitmask & MASK_LOG_CURRENT)
		Log_Write_Current();
#endif
	// send a heartbeat
	gcs_send_message(MSG_HEARTBEAT);
}

static void update_GPS(void)
{        
	g_gps->update();
	update_GPS_light();

    have_position = ahrs.get_position(&current_loc);

	if (g_gps->new_data && g_gps->status() == GPS::GPS_OK) {
		gps_fix_count++;

		if(ground_start_count > 1){
			ground_start_count--;
			ground_start_avg += g_gps->ground_speed;

		} else if (ground_start_count == 1) {
			// We countdown N number of good GPS fixes
			// so that the altitude is more accurate
			// -------------------------------------
			if (current_loc.lat == 0) {
				ground_start_count = 5;

			} else {
                init_home();
				if (g.compass_enabled) {
					// Set compass declination automatically
					compass.set_initial_location(g_gps->latitude, g_gps->longitude);
				}
				ground_start_count = 0;
			}
		}
        ground_speed   = g_gps->ground_speed * 0.01;
	}
}

static void update_current_mode(void)
{ 
    switch (control_mode){
    case AUTO:
    case RTL:
    case GUIDED:
        calc_nav_steer();
        calc_throttle();
        break;

    case LEARNING:
    case MANUAL:
        nav_steer        = 0;
        g.channel_steer.servo_out = g.channel_steer.pwm_to_angle();
        break;

    case INITIALISING:
        break;
	}
}

static void update_navigation()
{
    switch (control_mode) {
    case MANUAL:
    case LEARNING:
    case INITIALISING:
        break;

    case AUTO:
		verify_commands();
        break;

    case RTL:
    case GUIDED:
        // no loitering around the wp with the rover, goes direct to the wp position
        calc_nav_steer();
        calc_bearing_error();
        if (verify_RTL()) {  
            g.channel_throttle.servo_out = g.throttle_min.get();
            set_mode(MANUAL);
        }
        break;
	}
}

AP_HAL_MAIN();
#line 1 "./Firmware/APMRover2/GCS_Mavlink.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;

// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_## id ##_LEN) return false

// prototype this for use inside the GCS class
void gcs_send_text_fmt(const prog_char_t *fmt, ...);

/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */

static NOINLINE void send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;
    
    if (failsafe != FAILSAFE_NONE) {
        system_status = MAV_STATE_CRITICAL;
    }

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (control_mode) {
    case MANUAL:
        base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case LEARNING:
        base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case AUTO:
    case RTL:
    case GUIDED:
        base_mode = MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case INITIALISING:
        system_status = MAV_STATE_CALIBRATING;
        break;
    }

#if ENABLE_STICK_MIXING==ENABLED
    if (control_mode != INITIALISING) {
        // all modes except INITIALISING have some form of manual
        // override if stick mixing is enabled
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }
#endif

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (control_mode != INITIALISING) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_FIXED_WING,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    Vector3f omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        millis(),
        ahrs.roll,
        ahrs.pitch,
        ahrs.yaw,
        omega.x,
        omega.y,
        omega.z);
}

static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops)
{
    uint32_t control_sensors_present = 0;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // first what sensors/controllers we have
    control_sensors_present |= (1<<0); // 3D gyro present
    control_sensors_present |= (1<<1); // 3D accelerometer present
    if (g.compass_enabled) {
        control_sensors_present |= (1<<2); // compass present
    }
    control_sensors_present |= (1<<3); // absolute pressure sensor present
    if (g_gps != NULL && g_gps->status() == GPS::GPS_OK) {
        control_sensors_present |= (1<<5); // GPS present
    }
    control_sensors_present |= (1<<10); // 3D angular rate control
    control_sensors_present |= (1<<11); // attitude stabilisation
    control_sensors_present |= (1<<12); // yaw position
    control_sensors_present |= (1<<13); // altitude control
    control_sensors_present |= (1<<14); // X/Y position control
    control_sensors_present |= (1<<15); // motor control

    // now what sensors/controllers are enabled

    // first the sensors
    control_sensors_enabled = control_sensors_present & 0x1FF;

    // now the controllers
    control_sensors_enabled = control_sensors_present & 0x1FF;

    switch (control_mode) {
    case MANUAL:
        break;

    case LEARNING:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        break;

    case AUTO:
    case RTL:
    case GUIDED:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        control_sensors_enabled |= (1<<12); // yaw position
        control_sensors_enabled |= (1<<13); // altitude control
        control_sensors_enabled |= (1<<14); // X/Y position control
        control_sensors_enabled |= (1<<15); // motor control
        break;

    case INITIALISING:
        break;
    }

    // at the moment all sensors/controllers are assumed healthy
    control_sensors_health = control_sensors_present;

    if (!compass.healthy) {
        control_sensors_health &= ~(1<<2); // compass
    }
    if (!compass.use_for_yaw()) {
        control_sensors_enabled &= ~(1<<2); // compass
    }

    uint16_t battery_current = -1;
    uint8_t  battery_remaining = -1;

    if (current_total1 != 0 && g.pack_capacity != 0) {
        battery_remaining = (100.0 * (g.pack_capacity - current_total1) / g.pack_capacity);
    }
    if (current_total1 != 0) {
        battery_current = current_amps1 * 100;
    }

    if (g.battery_monitoring == 3) {
        /*setting a out-of-range value.
         *  It informs to external devices that
         *  it cannot be calculated properly just by voltage*/
        battery_remaining = 150;
    }

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(load * 1000),
        battery_voltage1 * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);

}

static void NOINLINE send_meminfo(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2
    extern unsigned __brkval;
    mavlink_msg_meminfo_send(chan, __brkval, memcheck_available_memory());
#endif
}

static void NOINLINE send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.    
    if (g_gps->status() == GPS::GPS_OK) {
        fix_time = g_gps->last_fix_time;
    } else {
        fix_time = millis();
    }
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        g_gps->altitude * 10,             // millimeters above sea level
        (current_loc.alt - home.alt) * 10,           // millimeters above ground
        g_gps->velocity_north() * 100,  // X speed cm/s (+ve North)
        g_gps->velocity_east()  * 100,  // Y speed cm/s (+ve East)
        g_gps->velocity_down()  * -100, // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);
}

static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)
{
    int16_t bearing = nav_bearing / 100;
    mavlink_msg_nav_controller_output_send(
        chan,
        nav_steer / 1.0e2,
        0,
        bearing,
        target_bearing / 100,
        wp_distance,
        altitude_error / 1.0e2,
        groundspeed_error,
        crosstrack_error);
}

static void NOINLINE send_gps_raw(mavlink_channel_t chan)
{
    uint8_t fix = g_gps->status();
    if (fix == GPS::GPS_OK) {
        fix = 3;
    }

    mavlink_msg_gps_raw_int_send(
        chan,
        g_gps->last_fix_time*(uint64_t)1000,
        fix,
        g_gps->latitude,      // in 1E7 degrees
        g_gps->longitude,     // in 1E7 degrees
        g_gps->altitude * 10, // in mm
        g_gps->hdop,
        65535,
        g_gps->ground_speed,  // cm/s
        g_gps->ground_course, // 1/100 degrees,
        g_gps->num_sats);
}

static void NOINLINE send_servo_out(mavlink_channel_t chan)
{
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with
    // HIL maintainers
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        10000 * g.channel_steer.norm_output(),
        0,
        10000 * g.channel_throttle.norm_output(),
        0,
        0,
        0,
        0,
        0,
        receiver_rssi);
}

static void NOINLINE send_radio_in(mavlink_channel_t chan)
{
    mavlink_msg_rc_channels_raw_send(
        chan,
        millis(),
        0, // port
        hal.rcin->read(CH_1),
        hal.rcin->read(CH_2),
        hal.rcin->read(CH_3),
        hal.rcin->read(CH_4),
        hal.rcin->read(CH_5),
        hal.rcin->read(CH_6),
        hal.rcin->read(CH_7),
        hal.rcin->read(CH_8),
        receiver_rssi);
}

static void NOINLINE send_radio_out(mavlink_channel_t chan)
{
#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
    mavlink_msg_servo_output_raw_send(
        chan,
        micros(),
        0,     // port
        hal.rcout->read(0),
        hal.rcout->read(1),
        hal.rcout->read(2),
        hal.rcout->read(3),
        hal.rcout->read(4),
        hal.rcout->read(5),
        hal.rcout->read(6),
        hal.rcout->read(7));
#else
    extern RC_Channel* rc_ch[8];
    mavlink_msg_servo_output_raw_send(
        chan,
        micros(),
        0,     // port
        rc_ch[0]->radio_out,
        rc_ch[1]->radio_out,
        rc_ch[2]->radio_out,
        rc_ch[3]->radio_out,
        rc_ch[4]->radio_out,
        rc_ch[5]->radio_out,
        rc_ch[6]->radio_out,
        rc_ch[7]->radio_out);
#endif
}

static void NOINLINE send_vfr_hud(mavlink_channel_t chan)
{
    mavlink_msg_vfr_hud_send(
        chan,
        (float)g_gps->ground_speed / 100.0,
        (float)g_gps->ground_speed / 100.0,
        (ahrs.yaw_sensor / 100) % 360,
        (uint16_t)(100 * g.channel_throttle.norm_output()),
        current_loc.alt / 100.0,
        0);
}

static void NOINLINE send_raw_imu1(mavlink_channel_t chan)
{
    Vector3f accel = ins.get_accel();
    Vector3f gyro = ins.get_gyro();

    mavlink_msg_raw_imu_send(
        chan,
        micros(),
        accel.x * 1000.0 / GRAVITY_MSS,
        accel.y * 1000.0 / GRAVITY_MSS,
        accel.z * 1000.0 / GRAVITY_MSS,
        gyro.x * 1000.0,
        gyro.y * 1000.0,
        gyro.z * 1000.0,
        compass.mag_x,
        compass.mag_y,
        compass.mag_z);
}

static void NOINLINE send_raw_imu3(mavlink_channel_t chan)
{
    Vector3f mag_offsets = compass.get_offsets();
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f gyro_offsets = ins.get_gyro_offsets();

    mavlink_msg_sensor_offsets_send(chan,
                                    mag_offsets.x,
                                    mag_offsets.y,
                                    mag_offsets.z,
                                    compass.get_declination(),
                                    0, 0,
                                    gyro_offsets.x,
                                    gyro_offsets.y,
                                    gyro_offsets.z,
                                    accel_offsets.x,
                                    accel_offsets.y,
                                    accel_offsets.z);
}

static void NOINLINE send_ahrs(mavlink_channel_t chan)
{
    Vector3f omega_I = ahrs.get_gyro_drift();
    mavlink_msg_ahrs_send(
        chan,
        omega_I.x,
        omega_I.y,
        omega_I.z,
        0,
        0,
        ahrs.get_error_rp(),
        ahrs.get_error_yaw());
}

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
// report simulator state
static void NOINLINE send_simstate(mavlink_channel_t chan)
{
    sitl.simstate_send(chan);
}
#endif

static void NOINLINE send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        board_voltage(),
        hal.i2c->lockup_count());
}

static void NOINLINE send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_mission_current_send(
        chan,
        g.command_index);
}

static void NOINLINE send_statustext(mavlink_channel_t chan)
{
    mavlink_statustext_t *s = (chan == MAVLINK_COMM_0?&gcs0.pending_status:&gcs3.pending_status);
    mavlink_msg_statustext_send(
        chan,
        s->severity,
        s->text);
}

// are we still delaying telemetry to try to avoid Xbee bricking?
static bool telemetry_delayed(mavlink_channel_t chan)
{
    uint32_t tnow = millis() >> 10;
    if (tnow > (uint32_t)g.telem_delay) {
        return false;
    }
#if USB_MUX_PIN > 0
    if (chan == MAVLINK_COMM_0 && usb_connected) {
        // this is an APM2 with USB telemetry
        return false;
    }
    // we're either on the 2nd UART, or no USB cable is connected
    // we need to delay telemetry
    return true;
#else
    if (chan == MAVLINK_COMM_0) {
        // we're on the USB port
        return false;
    }
    // don't send telemetry yet
    return true;
#endif
}


// try to send a message, return false if it won't fit in the serial tx buffer
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    int16_t payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;

    if (telemetry_delayed(chan)) {
        return false;
    }

    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        send_heartbeat(chan);
        return true;

    case MSG_EXTENDED_STATUS1:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        send_extended_status1(chan, packet_drops);
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        send_meminfo(chan);
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_location(chan);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        if (control_mode != MANUAL) {
            CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
            send_nav_controller_output(chan);
        }
        break;

    case MSG_GPS_RAW:
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
        send_gps_raw(chan);
        break;

    case MSG_SERVO_OUT:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        send_servo_out(chan);
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_radio_in(chan);
        break;

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_radio_out(chan);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        send_vfr_hud(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu1(chan);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_raw_imu3(chan);
        break;

    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_CURRENT);
        send_current_waypoint(chan);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        if (chan == MAVLINK_COMM_0) {
            gcs0.queued_param_send();
        } else if (gcs3.initialised) {
            gcs3.queued_param_send();
        }
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        if (chan == MAVLINK_COMM_0) {
            gcs0.queued_waypoint_send();
        } else if (gcs3.initialised) {
            gcs3.queued_waypoint_send();
        }
        break;

    case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs(chan);
        break;

    case MSG_SIMSTATE:
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        send_simstate(chan);
#endif
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus(chan);
        break;

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning
	}
    return true;
}


#define MAX_DEFERRED_MESSAGES MSG_RETRY_DEFERRED
static struct mavlink_queue {
    enum ap_message deferred_messages[MAX_DEFERRED_MESSAGES];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;
} mavlink_queue[2];

// send a message using mavlink
static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    uint8_t i, nextid;
    struct mavlink_queue *q = &mavlink_queue[(uint8_t)chan];

    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0) {
        if (!mavlink_try_send_message(chan,
                                      q->deferred_messages[q->next_deferred_message],
                                      packet_drops)) {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_MESSAGES) {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++) {
        if (q->deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_MESSAGES) {
            nextid = 0;
        }
    }

    if (q->num_deferred_messages != 0 ||
        !mavlink_try_send_message(chan, id, packet_drops)) {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_MESSAGES) {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_MESSAGES) {
            nextid -= MAX_DEFERRED_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}

void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str)
{
    if (telemetry_delayed(chan)) {
        return;
    }

    if (severity == SEVERITY_LOW) {
        // send via the deferred queuing system
        mavlink_statustext_t *s = (chan == MAVLINK_COMM_0?&gcs0.pending_status:&gcs3.pending_status);
        s->severity = (uint8_t)severity;
        strncpy((char *)s->text, str, sizeof(s->text));
        mavlink_send_message(chan, MSG_STATUSTEXT, 0);
    } else {
        // send immediately
        mavlink_msg_statustext_send(chan, severity, str);
    }
}

const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRateRawSensors,      0),
	AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRateExtendedStatus,  0),
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRateRCChannels,      0),
	AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRateRawController,   0),
	AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRatePosition,        0),
	AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRateExtra1,          0),
	AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRateExtra2,          0),
	AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRateExtra3,          0),
	AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRateParams,          0),
    AP_GROUPEND
};


GCS_MAVLINK::GCS_MAVLINK() :
    packet_drops(0),
    waypoint_send_timeout(1000), // 1 second
    waypoint_receive_timeout(1000) // 1 second
{
}

void
GCS_MAVLINK::init(AP_HAL::UARTDriver *port)
{
    GCS_Class::init(port);
    if (port == (AP_HAL::BetterStream*)hal.uartA) {
        mavlink_comm_0_port = port;
        chan = MAVLINK_COMM_0;
    }else{
        mavlink_comm_1_port = port;
        chan = MAVLINK_COMM_1;
    }
    _queued_parameter = NULL;
    reset_cli_timeout();
}

void
GCS_MAVLINK::update(void)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
	status.packet_rx_drop_count = 0;

    // process received bytes
    while (comm_get_available(chan))
    {
        uint8_t c = comm_receive_ch(chan);

#if CLI_ENABLED == ENABLED
        /* allow CLI to be started by hitting enter 3 times, if no
         *  heartbeat packets have been received */
        if (mavlink_active == 0 && (millis() - _cli_timeout) < 30000) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
            if (crlf_count == 3) {
                run_cli(_port);
            }
        }
#endif

        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) {
            // we exclude radio packets to make it possible to use the
            // CLI over the radio
            if (msg.msgid != MAVLINK_MSG_ID_RADIO) {
                mavlink_active = true;
            }
            handleMessage(&msg);
        }
    }

    // Update packet drops counter
    packet_drops += status.packet_rx_drop_count;

    if (!waypoint_receiving) {
        return;
    }

    uint32_t tnow = millis();

    if (waypoint_receiving &&
        waypoint_request_i <= waypoint_request_last &&
        tnow > waypoint_timelast_request + 500 + (stream_slowdown*20)) {
        waypoint_timelast_request = tnow;
        send_message(MSG_NEXT_WAYPOINT);
    }

    // stop waypoint receiving if timeout
    if (waypoint_receiving && (millis() - waypoint_timelast_receive) > waypoint_receive_timeout){
        waypoint_receiving = false;
    }
}

// see if we should send a stream now. Called at 50Hz
bool GCS_MAVLINK::stream_trigger(enum streams stream_num)
{
    AP_Int16 *stream_rates = &streamRateRawSensors;
    float rate = (uint8_t)stream_rates[stream_num].get();

    // send at a much lower rate while handling waypoints and
    // parameter sends
    if (waypoint_receiving || _queued_parameter != NULL) {
        rate *= 0.25;
    }

    if (rate <= 0) {
        return false;
    }

    if (stream_ticks[stream_num] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > 50) {
            rate = 50;
        }
        stream_ticks[stream_num] = (50 / rate) + stream_slowdown;
        return true;
    }

    // count down at 50Hz
    stream_ticks[stream_num]--;
    return false;
}

void
GCS_MAVLINK::data_stream_send(void)
{
    if (_queued_parameter != NULL) {
        if (streamRateParams.get() <= 0) {
            streamRateParams.set(50);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
    }

    if (in_mavlink_delay) {
#if HIL_MODE != HIL_MODE_DISABLED
        // in HIL we need to keep sending servo values to ensure
        // the simulator doesn't pause, otherwise our sensor
        // calibration could stall
        if (stream_trigger(STREAM_RAW_CONTROLLER)) {
            send_message(MSG_SERVO_OUT);
        }
        if (stream_trigger(STREAM_RC_CHANNELS)) {
            send_message(MSG_RADIO_OUT);
        }
#endif
        // don't send any other stream types while in the delay callback
        return;
    }

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU3);
    }

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);            // TODO - remove this message after location message is working
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
    }

    if (stream_trigger(STREAM_POSITION)) {
        // sent with GPS read
        send_message(MSG_LOCATION);
    }

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
    }

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_RADIO_OUT);
        send_message(MSG_RADIO_IN);
    }

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
    }

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
    }

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
    }
}



void
GCS_MAVLINK::send_message(enum ap_message id)
{
    mavlink_send_message(chan,id, packet_drops);
}

void
GCS_MAVLINK::send_text_P(gcs_severity severity, const prog_char_t *str)
{
    mavlink_statustext_t m;
    uint8_t i;
    for (i=0; i<sizeof(m.text); i++) {
        m.text[i] = pgm_read_byte((const prog_char *)(str++));
    }
    if (i < sizeof(m.text)) m.text[i] = 0;
    mavlink_send_text(chan, severity, (const char *)m.text);
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    struct Location tell_command = {};                // command for telemetry

    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        {
            // decode
            mavlink_request_data_stream_t packet;
            mavlink_msg_request_data_stream_decode(msg, &packet);

			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

        int16_t freq = 0;     // packet frequency

			if (packet.start_stop == 0)
				freq = 0; // stop sending
			else if (packet.start_stop == 1)
				freq = packet.req_message_rate; // start sending
			else
				break;

            switch(packet.req_stream_id){

                case MAV_DATA_STREAM_ALL:
                    streamRateRawSensors.set_and_save_ifchanged(freq);
                    streamRateExtendedStatus.set_and_save_ifchanged(freq);
                    streamRateRCChannels.set_and_save_ifchanged(freq);
                    streamRateRawController.set_and_save_ifchanged(freq);
                    streamRatePosition.set_and_save_ifchanged(freq);
                    streamRateExtra1.set_and_save_ifchanged(freq);
                    streamRateExtra2.set_and_save_ifchanged(freq);
                    streamRateExtra3.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_RAW_SENSORS:
            if (freq <= 5) {
                streamRateRawSensors.set_and_save_ifchanged(freq);
            } else {
                // We do not set and save this one so that if HIL is shut down incorrectly
														// we will not continue to broadcast raw sensor data at 50Hz.
                streamRateRawSensors = freq;
            }
                    break;

                case MAV_DATA_STREAM_EXTENDED_STATUS:
                    streamRateExtendedStatus.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_RC_CHANNELS:
                    streamRateRCChannels.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_RAW_CONTROLLER:
                    streamRateRawController.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_POSITION:
                    streamRatePosition.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_EXTRA1:
                    streamRateExtra1.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_EXTRA2:
                    streamRateExtra2.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_EXTRA3:
                    streamRateExtra3.set_and_save_ifchanged(freq);
                    break;

                default:
                    break;
            }
            break;
        }

    case MAVLINK_MSG_ID_COMMAND_LONG:
        {
            // decode
            mavlink_command_long_t packet;
            mavlink_msg_command_long_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system, packet.target_component)) break;

        uint8_t result = MAV_RESULT_UNSUPPORTED;

            // do command
            send_text_P(SEVERITY_LOW,PSTR("command received: "));

            switch(packet.command) {

            case MAV_CMD_NAV_RETURN_TO_LAUNCH:
                set_mode(RTL);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_CMD_MISSION_START:
                set_mode(AUTO);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_CMD_PREFLIGHT_CALIBRATION:
                if (packet.param1 == 1 ||
                    packet.param2 == 1 ||
                    packet.param3 == 1) {
            #if LITE == DISABLED                      
                    startup_INS_ground(true);
            #endif
                }
                if (packet.param4 == 1) {
                    trim_radio();
                }
                result = MAV_RESULT_ACCEPTED;
                break;

        case MAV_CMD_DO_SET_MODE:
            switch ((uint16_t)packet.param1) {
            case MAV_MODE_MANUAL_ARMED:
            case MAV_MODE_MANUAL_DISARMED:
                set_mode(MANUAL);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_AUTO_ARMED:
            case MAV_MODE_AUTO_DISARMED:
                set_mode(AUTO);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_STABILIZE_DISARMED:
            case MAV_MODE_STABILIZE_ARMED:
                set_mode(LEARNING);
                result = MAV_RESULT_ACCEPTED;
                break;

            default:
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_DO_SET_SERVO:
            hal.rcout->enable_ch(packet.param1 - 1);
            hal.rcout->write(packet.param1 - 1, packet.param2);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (packet.param1 == 1) {
                reboot_apm();
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        default:
                break;
            }

            mavlink_msg_command_ack_send(
                chan,
                packet.command,
                result);

            break;
        }


    case MAVLINK_MSG_ID_SET_MODE:
		{
            // decode
            mavlink_set_mode_t packet;
            mavlink_msg_set_mode_decode(msg, &packet);

            if (!(packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
                // we ignore base_mode as there is no sane way to map
                // from that bitmap to a APM flight mode. We rely on
                // custom_mode instead.
                break;
            }
            switch (packet.custom_mode) {
            case MANUAL:
            case LEARNING:
            case AUTO:
            case RTL:
                set_mode((enum mode)packet.custom_mode);
                break;
            }

            break;
		}

    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        {
            // decode
            mavlink_mission_request_list_t packet;
            mavlink_msg_mission_request_list_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

            // Start sending waypoints
            mavlink_msg_mission_count_send(
                chan,msg->sysid,
                msg->compid,
                g.command_total + 1); // + home

            waypoint_timelast_send   = millis();
            waypoint_receiving       = false;
            waypoint_dest_sysid      = msg->sysid;
            waypoint_dest_compid     = msg->compid;
            break;
        }


	// XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:
        {
            // decode
            mavlink_mission_request_t packet;
            mavlink_msg_mission_request_decode(msg, &packet);

 			if (mavlink_check_target(packet.target_system, packet.target_component))
 				break;

            // send waypoint
            tell_command = get_cmd_with_index(packet.seq);

            // set frame of waypoint
            uint8_t frame;

			if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
                frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; // reference frame
            } else {
                frame = MAV_FRAME_GLOBAL; // reference frame
            }

            float param1 = 0, param2 = 0 , param3 = 0, param4 = 0;

        // time that the mav should loiter in milliseconds
            uint8_t current = 0; // 1 (true), 0 (false)

			if (packet.seq == (uint16_t)g.command_index)
            	current = 1;

            uint8_t autocontinue = 1; // 1 (true), 0 (false)

            float x = 0, y = 0, z = 0;

            if (tell_command.id < MAV_CMD_NAV_LAST || tell_command.id == MAV_CMD_CONDITION_CHANGE_ALT) {
                // command needs scaling
                x = tell_command.lat/1.0e7; // local (x), global (latitude)
                y = tell_command.lng/1.0e7; // local (y), global (longitude)
            z = tell_command.alt/1.0e2;
            }

			switch (tell_command.id) {				// Switch to map APM command fields inot MAVLink command fields

				case MAV_CMD_NAV_TAKEOFF:
				case MAV_CMD_DO_SET_HOME:
					param1 = tell_command.p1;
					break;

				case MAV_CMD_CONDITION_CHANGE_ALT:
					x=0;	// Clear fields loaded above that we don't want sent for this command
					y=0;
				case MAV_CMD_CONDITION_DELAY:
				case MAV_CMD_CONDITION_DISTANCE:
					param1 = tell_command.lat;
					break;

				case MAV_CMD_DO_JUMP:
					param2 = tell_command.lat;
					param1 = tell_command.p1;
					break;

				case MAV_CMD_DO_REPEAT_SERVO:
					param4 = tell_command.lng;
				case MAV_CMD_DO_REPEAT_RELAY:
				case MAV_CMD_DO_CHANGE_SPEED:
					param3 = tell_command.lat;
					param2 = tell_command.alt;
					param1 = tell_command.p1;
					break;

				case MAV_CMD_DO_SET_PARAMETER:
				case MAV_CMD_DO_SET_RELAY:
				case MAV_CMD_DO_SET_SERVO:
					param2 = tell_command.alt;
					param1 = tell_command.p1;
					break;
			}

			mavlink_msg_mission_item_send(chan,msg->sysid,
										msg->compid,
										packet.seq,
										frame,
										tell_command.id,
										current,
										autocontinue,
										param1,
										param2,
										param3,
										param4,
										x,
										y,
										z);

            // update last waypoint comm stamp
            waypoint_timelast_send = millis();
            break;
        }


    case MAVLINK_MSG_ID_MISSION_ACK:
        {
            // decode
            mavlink_mission_ack_t packet;
            mavlink_msg_mission_ack_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;
            break;
        }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
            // decode
            mavlink_param_request_list_t packet;
            mavlink_msg_param_request_list_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // Start sending parameters - next call to ::update will kick the first one out

            _queued_parameter = AP_Param::first(&_queued_parameter_token, &_queued_parameter_type);
            _queued_parameter_index = 0;
            _queued_parameter_count = _count_parameters();
            break;
        }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        // decode
        mavlink_param_request_read_t packet;
        mavlink_msg_param_request_read_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        enum ap_var_type p_type;
        AP_Param *vp;
        char param_name[AP_MAX_NAME_SIZE+1];
        if (packet.param_index != -1) {
            AP_Param::ParamToken token;
            vp = AP_Param::find_by_index(packet.param_index, &p_type, &token);
            if (vp == NULL) {
                gcs_send_text_fmt(PSTR("Unknown parameter index %d"), packet.param_index);
                break;
            }
            vp->copy_name_token(&token, param_name, AP_MAX_NAME_SIZE, true);
            param_name[AP_MAX_NAME_SIZE] = 0;
        } else {
            strncpy(param_name, packet.param_id, AP_MAX_NAME_SIZE);
            param_name[AP_MAX_NAME_SIZE] = 0;
            vp = AP_Param::find(param_name, &p_type);
            if (vp == NULL) {
                gcs_send_text_fmt(PSTR("Unknown parameter %.16s"), packet.param_id);
                break;
            }
        }

        float value = vp->cast_to_float(p_type);
        mavlink_msg_param_value_send(
            chan,
            param_name,
            value,
            mav_var_type(p_type),
            _count_parameters(),
            packet.param_index);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        {
            // decode
            mavlink_mission_clear_all_t packet;
            mavlink_msg_mission_clear_all_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system, packet.target_component)) break;

            // clear all commands
            g.command_total.set_and_save(0);

            // note that we don't send multiple acks, as otherwise a
            // GCS that is doing a clear followed by a set may see
            // the additional ACKs as ACKs of the set operations
            mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_ACCEPTED);
            break;
        }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        {
            // decode
            mavlink_mission_set_current_t packet;
            mavlink_msg_mission_set_current_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // set current command
            change_command(packet.seq);

            mavlink_msg_mission_current_send(chan, g.command_index);
            break;
        }

    case MAVLINK_MSG_ID_MISSION_COUNT:
        {
            // decode
            mavlink_mission_count_t packet;
            mavlink_msg_mission_count_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // start waypoint receiving
            if (packet.count > MAX_WAYPOINTS) {
                packet.count = MAX_WAYPOINTS;
            }
            g.command_total.set_and_save(packet.count - 1);

            waypoint_timelast_receive = millis();
            waypoint_timelast_request = 0;
            waypoint_receiving   = true;
            waypoint_request_i   = 0;
            waypoint_request_last= g.command_total;
            break;
        }

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    {
        // decode
        mavlink_mission_write_partial_list_t packet;
        mavlink_msg_mission_write_partial_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // start waypoint receiving
        if (packet.start_index > g.command_total ||
            packet.end_index > g.command_total ||
            packet.end_index < packet.start_index) {
            send_text_P(SEVERITY_LOW,PSTR("flight plan update rejected"));
            break;
        }

        waypoint_timelast_receive = millis();
        waypoint_timelast_request = 0;
        waypoint_receiving   = true;
        waypoint_request_i   = packet.start_index;
        waypoint_request_last= packet.end_index;
        break;
    }

#ifdef MAVLINK_MSG_ID_SET_MAG_OFFSETS
    case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
    {
        mavlink_set_mag_offsets_t packet;
        mavlink_msg_set_mag_offsets_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        compass.set_offsets(Vector3f(packet.mag_ofs_x, packet.mag_ofs_y, packet.mag_ofs_z));
        break;
    }
#endif

	// XXX receive a WP from GCS and store in EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:
        {
            // decode
            mavlink_mission_item_t packet;
            uint8_t result = MAV_MISSION_ACCEPTED;

            mavlink_msg_mission_item_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // defaults
            tell_command.id = packet.command;

			switch (packet.frame)
			{
				case MAV_FRAME_MISSION:
				case MAV_FRAME_GLOBAL:
					{
						tell_command.lat = 1.0e7f*packet.x; // in as DD converted to * t7
						tell_command.lng = 1.0e7f*packet.y; // in as DD converted to * t7
						tell_command.alt = packet.z*1.0e2f; // in as m converted to cm
						tell_command.options = 0; // absolute altitude
						break;
					}

#ifdef MAV_FRAME_LOCAL_NED
				case MAV_FRAME_LOCAL_NED: // local (relative to home position)
					{
						tell_command.lat = 1.0e7f*ToDeg(packet.x/
						(radius_of_earth*cosf(ToRad(home.lat/1.0e7f)))) + home.lat;
						tell_command.lng = 1.0e7f*ToDeg(packet.y/radius_of_earth) + home.lng;
						tell_command.alt = -packet.z*1.0e2f;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
						break;
					}
#endif

#ifdef MAV_FRAME_LOCAL
				case MAV_FRAME_LOCAL: // local (relative to home position)
					{
						tell_command.lat = 1.0e7f*ToDeg(packet.x/
						(radius_of_earth*cosf(ToRad(home.lat/1.0e7f)))) + home.lat;
						tell_command.lng = 1.0e7f*ToDeg(packet.y/radius_of_earth) + home.lng;
						tell_command.alt = packet.z*1.0e2f;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
						break;
					}
#endif

				case MAV_FRAME_GLOBAL_RELATIVE_ALT: // absolute lat/lng, relative altitude
					{
						tell_command.lat = 1.0e7f * packet.x; // in as DD converted to * t7
						tell_command.lng = 1.0e7f * packet.y; // in as DD converted to * t7
						tell_command.alt = packet.z * 1.0e2f;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT; // store altitude relative!! Always!!
						break;
					}

            default:
                result = MAV_MISSION_UNSUPPORTED_FRAME;
                break;
			}

            
            if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

            switch (tell_command.id) {                    // Switch to map APM command fields inot MAVLink command fields
            case MAV_CMD_NAV_WAYPOINT:
            case MAV_CMD_NAV_RETURN_TO_LAUNCH:
                break;

            case MAV_CMD_NAV_TAKEOFF:
            case MAV_CMD_DO_SET_HOME:
                tell_command.p1 = packet.param1;
                break;

            case MAV_CMD_CONDITION_CHANGE_ALT:
                tell_command.lat = packet.param1;
                break;

            case MAV_CMD_CONDITION_DELAY:
            case MAV_CMD_CONDITION_DISTANCE:
                tell_command.lat = packet.param1;
                break;

            case MAV_CMD_DO_JUMP:
                tell_command.lat = packet.param2;
                tell_command.p1 = packet.param1;
                break;

            case MAV_CMD_DO_REPEAT_SERVO:
                tell_command.lng = packet.param4;
            case MAV_CMD_DO_REPEAT_RELAY:
            case MAV_CMD_DO_CHANGE_SPEED:
                tell_command.lat = packet.param3;
                tell_command.alt = packet.param2;
                tell_command.p1 = packet.param1;
                break;

            case MAV_CMD_DO_SET_PARAMETER:
            case MAV_CMD_DO_SET_RELAY:
            case MAV_CMD_DO_SET_SERVO:
                tell_command.alt = packet.param2;
                tell_command.p1 = packet.param1;
                break;

            default:
                result = MAV_MISSION_UNSUPPORTED;
                break;
            }

            if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

			if(packet.current == 2){ 				//current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
				guided_WP = tell_command;

				// add home alt if needed
				if (guided_WP.options & MASK_OPTIONS_RELATIVE_ALT){
					guided_WP.alt += home.alt;
				}

				set_mode(GUIDED);

				// make any new wp uploaded instant (in case we are already in Guided mode)
				set_guided_WP();

				// verify we recevied the command
				mavlink_msg_mission_ack_send(
						chan,
						msg->sysid,
						msg->compid,
						0);

			} else {
				// Check if receiving waypoints (mission upload expected)
				if (!waypoint_receiving) {
                    result = MAV_MISSION_ERROR;
                    goto mission_failed;
                }

				// check if this is the requested waypoint
				if (packet.seq != waypoint_request_i) {
                    result = MAV_MISSION_INVALID_SEQUENCE;
                    goto mission_failed;
                }

                set_cmd_with_index(tell_command, packet.seq);

				// update waypoint receiving state machine
				waypoint_timelast_receive = millis();
                waypoint_timelast_request = 0;
				waypoint_request_i++;

                if (waypoint_request_i > waypoint_request_last) {
					mavlink_msg_mission_ack_send(
						chan,
						msg->sysid,
						msg->compid,
						result);

					send_text_P(SEVERITY_LOW,PSTR("flight plan received"));
					waypoint_receiving = false;
					// XXX ignores waypoint radius for individual waypoints, can
					// only set WP_RADIUS parameter
				}
			}
            break;

        mission_failed:
            // we are rejecting the mission/waypoint
            mavlink_msg_mission_ack_send(
                chan,
                msg->sysid,
                msg->compid,
                result);
            break;
        }


    case MAVLINK_MSG_ID_PARAM_SET:
        {
            AP_Param                  *vp;
            enum ap_var_type        var_type;

            // decode
            mavlink_param_set_t packet;
            mavlink_msg_param_set_decode(msg, &packet);

			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

            // set parameter

            char key[AP_MAX_NAME_SIZE+1];
            strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
            key[AP_MAX_NAME_SIZE] = 0;

            // find the requested parameter
            vp = AP_Param::find(key, &var_type);
            if ((NULL != vp) &&                             // exists
                    !isnan(packet.param_value) &&               // not nan
                    !isinf(packet.param_value)) {               // not inf

                // add a small amount before casting parameter values
                // from float to integer to avoid truncating to the
                // next lower integer value.
				float rounding_addition = 0.01;

                // handle variables with standard type IDs
                if (var_type == AP_PARAM_FLOAT) {
                    ((AP_Float *)vp)->set_and_save(packet.param_value);
                } else if (var_type == AP_PARAM_INT32) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    float v = packet.param_value+rounding_addition;
                    v = constrain(v, -2147483648.0f, 2147483647.0f);
					((AP_Int32 *)vp)->set_and_save(v);
                } else if (var_type == AP_PARAM_INT16) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    float v = packet.param_value+rounding_addition;
                    v = constrain(v, -32768, 32767);
					((AP_Int16 *)vp)->set_and_save(v);
                } else if (var_type == AP_PARAM_INT8) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    float v = packet.param_value+rounding_addition;
                    v = constrain(v, -128, 127);
					((AP_Int8 *)vp)->set_and_save(v);
                } else {
                    // we don't support mavlink set on this parameter
                    break;
                }

                // Report back the new value if we accepted the change
                // we send the value we actually set, which could be
                // different from the value sent, in case someone sent
                // a fractional value to an integer type
                mavlink_msg_param_value_send(
                    chan,
                    key,
                    vp->cast_to_float(var_type),
                    mav_var_type(var_type),
                    _count_parameters(),
                    -1); // XXX we don't actually know what its index is...
            }

            break;
        } // end case

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system,packet.target_component))
            break;

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;

        hal.rcin->set_overrides(v, 8);

        rc_override_fs_timer = millis();
        break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT:
        {
            // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
			if(msg->sysid != g.sysid_my_gcs) break;
            last_heartbeat_ms = rc_override_fs_timer = millis();
			pmTest1++;
            break;
        }

#if HIL_MODE != HIL_MODE_DISABLED
	case MAVLINK_MSG_ID_HIL_STATE:
		{
			mavlink_hil_state_t packet;
			mavlink_msg_hil_state_decode(msg, &packet);
			
			float vel = pythagorous2(packet.vx, packet.vy);
            float cog = wrap_360_cd(ToDeg(atan2f(packet.vy, packet.vx)) * 100);
			
            // set gps hil sensor
            g_gps->setHIL(packet.time_usec/1000,
                          packet.lat*1.0e-7f, packet.lon*1.0e-7f, packet.alt*1.0e-3f,
                          vel*1.0e-2f, cog*1.0e-2f, 0, 10);
			
			// rad/sec
            Vector3f gyros;
            gyros.x = packet.rollspeed;
            gyros.y = packet.pitchspeed;
            gyros.z = packet.yawspeed;

            // m/s/s
            Vector3f accels;
            accels.x = packet.xacc * (GRAVITY_MSS/1000.0f);
            accels.y = packet.yacc * (GRAVITY_MSS/1000.0f);
            accels.z = packet.zacc * (GRAVITY_MSS/1000.0f);
            
            ins.set_gyro(gyros);

            ins.set_accel(accels);
			
 #if HIL_MODE == HIL_MODE_ATTITUDE
			// set AHRS hil sensor
            ahrs.setHil(packet.roll,packet.pitch,packet.yaw,packet.rollspeed,
            packet.pitchspeed,packet.yawspeed);
 #endif

			break;
		}
#endif // HIL_MODE

#if MOUNT == ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
		{
			camera_mount.configure_msg(msg);
			break;
		}

    case MAVLINK_MSG_ID_MOUNT_CONTROL:
		{
			camera_mount.control_msg(msg);
			break;
		}

    case MAVLINK_MSG_ID_MOUNT_STATUS:
		{
			camera_mount.status_msg(msg);
			break;
		}
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_RADIO:
        {
            mavlink_radio_t packet;
            mavlink_msg_radio_decode(msg, &packet);
            // use the state of the transmit buffer in the radio to
            // control the stream rate, giving us adaptive software
            // flow control
            if (packet.txbuf < 20 && stream_slowdown < 100) {
                // we are very low on space - slow down a lot
                stream_slowdown += 3;
            } else if (packet.txbuf < 50 && stream_slowdown < 100) {
                // we are a bit low on space, slow down slightly
                stream_slowdown += 1;
            } else if (packet.txbuf > 95 && stream_slowdown > 10) {
                // the buffer has plenty of space, speed up a lot
                stream_slowdown -= 2;
            } else if (packet.txbuf > 90 && stream_slowdown != 0) {
                // the buffer has enough space, speed up a bit
                stream_slowdown--;
            }
            break;
        }

    default:
        // forward unknown messages to the other link if there is one
        if ((chan == MAVLINK_COMM_1 && gcs0.initialised) ||
            (chan == MAVLINK_COMM_0 && gcs3.initialised)) {
            mavlink_channel_t out_chan = (mavlink_channel_t)(((uint8_t)chan)^1);
            // only forward if it would fit in our transmit buffer
            if (comm_get_txspace(out_chan) > ((uint16_t)msg->len) + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
                _mavlink_resend_uart(out_chan, msg);
            }
        }
        break;

    } // end switch
} // end handle mavlink

uint16_t
GCS_MAVLINK::_count_parameters()
{
    // if we haven't cached the parameter count yet...
    if (0 == _parameter_count) {
        AP_Param  *vp;
        AP_Param::ParamToken token;

        vp = AP_Param::first(&token, NULL);
        do {
            _parameter_count++;
        } while (NULL != (vp = AP_Param::next_scalar(&token, NULL)));
    }
    return _parameter_count;
}

/**
* @brief Send the next pending parameter, called from deferred message
* handling code
*/
void
GCS_MAVLINK::queued_param_send()
{
    if (_queued_parameter == NULL) {
        return;
    }

    uint16_t bytes_allowed;
    uint8_t count;
    uint32_t tnow = millis();

    // use at most 30% of bandwidth on parameters. The constant 26 is
    // 1/(1000 * 1/8 * 0.001 * 0.3)
    bytes_allowed = g.serial3_baud * (tnow - _queued_parameter_send_time_ms) * 26;
    if (bytes_allowed > comm_get_txspace(chan)) {
        bytes_allowed = comm_get_txspace(chan);
    }
    count = bytes_allowed / (MAVLINK_MSG_ID_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);

    while (_queued_parameter != NULL && count--) {
    AP_Param      *vp;
    float       value;

    // copy the current parameter and prepare to move to the next
    vp = _queued_parameter;

    // if the parameter can be cast to float, report it here and break out of the loop
    value = vp->cast_to_float(_queued_parameter_type);

    char param_name[AP_MAX_NAME_SIZE];
    vp->copy_name_token(&_queued_parameter_token, param_name, sizeof(param_name), true);

    mavlink_msg_param_value_send(
        chan,
        param_name,
        value,
        mav_var_type(_queued_parameter_type),
        _queued_parameter_count,
        _queued_parameter_index);

    _queued_parameter = AP_Param::next_scalar(&_queued_parameter_token, &_queued_parameter_type);
    _queued_parameter_index++;
}
    _queued_parameter_send_time_ms = tnow;
}

/**
* @brief Send the next pending waypoint, called from deferred message
* handling code
*/
void
GCS_MAVLINK::queued_waypoint_send()
{
    if (waypoint_receiving &&
        waypoint_request_i <= waypoint_request_last) {
        mavlink_msg_mission_request_send(
            chan,
            waypoint_dest_sysid,
            waypoint_dest_compid,
            waypoint_request_i);
    }
}

void GCS_MAVLINK::reset_cli_timeout() {
      _cli_timeout = millis();
}
/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
static void mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs0.initialised) return;

    in_mavlink_delay = true;

    uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs_send_message(MSG_HEARTBEAT);
        gcs_send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_update();
        gcs_data_stream_send();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs_send_text_P(SEVERITY_LOW, PSTR("Initialising APM..."));
    }
#if USB_MUX_PIN > 0
    check_usb_mux();
#endif

    in_mavlink_delay = false;
}

/*
 *  send a message on both GCS links
 */
static void gcs_send_message(enum ap_message id)
{
    gcs0.send_message(id);
    if (gcs3.initialised) {
        gcs3.send_message(id);
    }
}

/*
 *  send data streams in the given rate range on both links
 */
static void gcs_data_stream_send(void)
{
    gcs0.data_stream_send();
    if (gcs3.initialised) {
        gcs3.data_stream_send();
    }
}

/*
 *  look for incoming commands on the GCS links
 */
static void gcs_update(void)
{
	gcs0.update();
    if (gcs3.initialised) {
        gcs3.update();
    }
}

static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str)
{
    gcs0.send_text_P(severity, str);
    if (gcs3.initialised) {
        gcs3.send_text_P(severity, str);
    }
}

/*
 *  send a low priority formatted message to the GCS
 *  only one fits in the queue, so if you send more than one before the
 *  last one gets into the serial buffer then the old one will be lost
 */
void gcs_send_text_fmt(const prog_char_t *fmt, ...)
{
    va_list arg_list;
    gcs0.pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)gcs0.pending_status.text,
            sizeof(gcs0.pending_status.text), fmt, arg_list);
    va_end(arg_list);
    gcs3.pending_status = gcs0.pending_status;
    mavlink_send_message(MAVLINK_COMM_0, MSG_STATUSTEXT, 0);
    if (gcs3.initialised) {
        mavlink_send_message(MAVLINK_COMM_1, MSG_STATUSTEXT, 0);
    }
}

#line 1 "./Firmware/APMRover2/Log.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LITE == DISABLED
#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t	dump_log(uint8_t argc, 			const Menu::arg *argv);
static int8_t	erase_logs(uint8_t argc, 		const Menu::arg *argv);
static int8_t	select_logs(uint8_t argc, 		const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
static const struct Menu::command log_menu_commands[] PROGMEM = {
	{"dump",	dump_log},
	{"erase",	erase_logs},
	{"enable",	select_logs},
	{"disable",	select_logs}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, print_log_menu);

static bool
print_log_menu(void)
{
	int16_t log_start;
	int16_t log_end;
	int16_t temp;	
	int16_t last_log_num = DataFlash.find_last_log();
	
	uint16_t num_logs = DataFlash.get_num_logs();

	cliSerial->printf_P(PSTR("logs enabled: "));

	if (0 == g.log_bitmask) {
		cliSerial->printf_P(PSTR("none"));
	}else{
		// Macro to make the following code a bit easier on the eye.
		// Pass it the capitalised name of the log option, as defined
		// in defines.h but without the LOG_ prefix.  It will check for
		// the bit being set and print the name of the log option to suit.
		#define PLOG(_s)	if (g.log_bitmask & MASK_LOG_ ## _s) cliSerial->printf_P(PSTR(" %S"), PSTR(#_s))
		PLOG(ATTITUDE_FAST);
		PLOG(ATTITUDE_MED);
		PLOG(GPS);
		PLOG(PM);
		PLOG(CTUN);
		PLOG(NTUN);
		PLOG(MODE);
		PLOG(IMU);
		PLOG(CMD);
		PLOG(CURRENT);
		#undef PLOG
	}

	cliSerial->println();

	if (num_logs == 0) {
		cliSerial->printf_P(PSTR("\nNo logs\n\n"));
	}else{
		cliSerial->printf_P(PSTR("\n%d logs\n"), num_logs);

		for(int i=num_logs;i>=1;i--) {
            int last_log_start = log_start, last_log_end = log_end;
			temp = last_log_num-i+1;
			DataFlash.get_log_boundaries(temp, log_start, log_end);
			cliSerial->printf_P(PSTR("Log %d,    start %d,   end %d\n"), temp, log_start, log_end);
            if (last_log_start == log_start && last_log_end == log_end) {
                // we are printing bogus logs
                break;
            }
		}
		cliSerial->println();
	}
	return(true);
}

static int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
	int16_t dump_log;
	int16_t dump_log_start;
	int16_t dump_log_end;
	uint8_t last_log_num;

	// check that the requested log number can be read
	dump_log = argv[1].i;
	last_log_num = DataFlash.find_last_log();
	
	if (dump_log == -2) {
		for(uint16_t count=1; count<=DataFlash.df_NumPages; count++) {
			DataFlash.StartRead(count);
			cliSerial->printf_P(PSTR("DF page, log file #, log page: %d,\t"), count);
			cliSerial->printf_P(PSTR("%d,\t"), DataFlash.GetFileNumber());
			cliSerial->printf_P(PSTR("%d\n"), DataFlash.GetFilePage());
		}
		return(-1);
	} else if (dump_log <= 0) {
		cliSerial->printf_P(PSTR("dumping all\n"));
		Log_Read(1, DataFlash.df_NumPages);
		return(-1);
	} else if ((argc != 2) || (dump_log <= (last_log_num - DataFlash.get_num_logs())) || (dump_log > last_log_num)) {
		cliSerial->printf_P(PSTR("bad log number\n"));
		return(-1);
	}

	DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
	cliSerial->printf_P(PSTR("Dumping Log %d,    start pg %d,   end pg %d\n"),
				  dump_log,
				  dump_log_start,
				  dump_log_end);

	Log_Read(dump_log_start, dump_log_end);
	cliSerial->printf_P(PSTR("Done\n"));
    return 0;
}


void erase_callback(unsigned long t) {
    mavlink_delay(t);
    if (DataFlash.GetWritePage() % 128 == 0) {
        cliSerial->printf_P(PSTR("+"));
    }
}

static void do_erase_logs(void)
{
	cliSerial->printf_P(PSTR("\nErasing log...\n"));
    DataFlash.EraseAll();
	cliSerial->printf_P(PSTR("\nLog erased.\n"));
}

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}

static int8_t
select_logs(uint8_t argc, const Menu::arg *argv)
{
	uint16_t	bits;

	if (argc != 2) {
		cliSerial->printf_P(PSTR("missing log type\n"));
		return(-1);
	}

	bits = 0;

	// Macro to make the following code a bit easier on the eye.
	// Pass it the capitalised name of the log option, as defined
	// in defines.h but without the LOG_ prefix.  It will check for
	// that name as the argument to the command, and set the bit in
	// bits accordingly.
	//
	if (!strcasecmp_P(argv[1].str, PSTR("all"))) {
		bits = ~0;
	} else {
		#define TARG(_s)	if (!strcasecmp_P(argv[1].str, PSTR(#_s))) bits |= MASK_LOG_ ## _s
		TARG(ATTITUDE_FAST);
		TARG(ATTITUDE_MED);
		TARG(GPS);
		TARG(PM);
		TARG(CTUN);
		TARG(NTUN);
		TARG(MODE);
		TARG(IMU);
		TARG(CMD);
		TARG(CURRENT);
		#undef TARG
	}

	if (!strcasecmp_P(argv[0].str, PSTR("enable"))) {
		g.log_bitmask.set_and_save(g.log_bitmask | bits);
	}else{
		g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
	}
	return(0);
}

static int8_t
process_logs(uint8_t argc, const Menu::arg *argv)
{
	log_menu.run();
	return 0;
}

// print_latlon - prints an latitude or longitude value held in an int32_t
// probably this should be moved to AP_Common
void print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon)
{
    int32_t dec_portion, frac_portion;
    int32_t abs_lat_or_lon = labs(lat_or_lon);

    // extract decimal portion (special handling of negative numbers to ensure we round towards zero)
    dec_portion = abs_lat_or_lon / T7;

    // extract fractional portion
    frac_portion = abs_lat_or_lon - dec_portion*T7;

    // print output including the minus sign
    if( lat_or_lon < 0 ) {
        s->printf_P(PSTR("-"));
    }
    s->printf_P(PSTR("%ld.%07ld"),(long)dec_portion,(long)frac_portion);
}


struct log_Attitute {
    LOG_PACKET_HEADER;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
};

// Write an attitude packet. Total length : 10 bytes
static void Log_Write_Attitude(int16_t log_roll, int16_t log_pitch, uint16_t log_yaw)
{
    struct log_Attitute pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        roll  : log_roll,
        pitch : log_pitch,
        yaw   : log_yaw
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read an attitude packet
static void Log_Read_Attitude()
{
    struct log_Attitute pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("ATT, %d, %d, %u\n"),
                        (int)pkt.roll,
                        (int)pkt.pitch,
                        (unsigned)pkt.yaw);
}

struct log_Performance {
    LOG_PACKET_HEADER;
    uint32_t loop_time;
    uint16_t main_loop_count;
    int16_t  g_dt_max;
    uint8_t  renorm_count;
    uint8_t  renorm_blowup;
    uint8_t  gps_fix_count;
    int16_t  gyro_drift_x;
    int16_t  gyro_drift_y;
    int16_t  gyro_drift_z;
    int16_t  pm_test;
};

// Write a performance monitoring packet. Total length : 19 bytes
#if HIL_MODE != HIL_MODE_ATTITUDE
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        loop_time       : millis()- perf_mon_timer,
        main_loop_count : mainLoop_count,
        g_dt_max        : G_Dt_max,
        renorm_count    : ahrs.renorm_range_count,
        renorm_blowup   : ahrs.renorm_blowup_count,
        gps_fix_count   : gps_fix_count,
        gyro_drift_x    : (int16_t)(ahrs.get_gyro_drift().x * 1000),
        gyro_drift_y    : (int16_t)(ahrs.get_gyro_drift().y * 1000),
        gyro_drift_z    : (int16_t)(ahrs.get_gyro_drift().z * 1000),
        pm_test         : pmTest1
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

// Read a performance packet
static void Log_Read_Performance()
{
    struct log_Performance pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    cliSerial->printf_P(PSTR("PM, %lu, %u, %d, %u, %u, %u, %d, %d, %d, %d\n"),
            pkt.loop_time,
            (unsigned)pkt.main_loop_count,
            (int)pkt.g_dt_max,
            (unsigned)pkt.renorm_count,
            (unsigned)pkt.renorm_blowup,
            (unsigned)pkt.gps_fix_count,
            (int)pkt.gyro_drift_x,
            (int)pkt.gyro_drift_y,
            (int)pkt.gyro_drift_z,
            (int)pkt.pm_test);
}

struct log_Cmd {
    LOG_PACKET_HEADER;
    uint8_t command_total;
    uint8_t command_number;
    uint8_t waypoint_id;
    uint8_t waypoint_options;
    uint8_t waypoint_param1;
    int32_t waypoint_altitude;
    int32_t waypoint_latitude;
    int32_t waypoint_longitude;
};

// Write a command processing packet. Total length : 19 bytes
static void Log_Write_Cmd(uint8_t num, struct Location *wp)
{
    struct log_Cmd pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CMD_MSG),
        command_total       : g.command_total,
        command_number      : num,
        waypoint_id         : wp->id,
        waypoint_options    : wp->options,
        waypoint_param1     : wp->p1,
        waypoint_altitude   : wp->alt,
        waypoint_latitude   : wp->lat,
        waypoint_longitude  : wp->lng
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a command processing packet
static void Log_Read_Cmd()
{
    struct log_Cmd pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    cliSerial->printf_P(PSTR("CMD, %u, %u, %u, %u, %u, %ld, %ld, %ld\n"),
        (unsigned)pkt.command_total,
        (unsigned)pkt.command_number,
        (unsigned)pkt.waypoint_id,
        (unsigned)pkt.waypoint_options,
        (unsigned)pkt.waypoint_param1,
        (long)pkt.waypoint_altitude,
        (long)pkt.waypoint_latitude,
        (long)pkt.waypoint_longitude);
}

struct log_Startup {
    LOG_PACKET_HEADER;
    uint8_t startup_type;
    uint8_t command_total;
};

static void Log_Write_Startup(uint8_t type)
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG),
        startup_type    : type,
        command_total   : g.command_total
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

    // write all commands to the dataflash as well
    struct Location cmd;
    for (uint8_t i = 0; i <= g.command_total; i++) {
        cmd = get_cmd_with_index(i);
        Log_Write_Cmd(i, &cmd);
    }
}

static void Log_Read_Startup()
{
    struct log_Startup pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    switch( pkt.startup_type ) {
        case TYPE_AIRSTART_MSG:
            cliSerial->printf_P(PSTR("AIR START"));
            break;
        case TYPE_GROUNDSTART_MSG:
            cliSerial->printf_P(PSTR("GROUND START"));
            break;
        default:
            cliSerial->printf_P(PSTR("UNKNOWN STARTUP"));
            break;
    }

    cliSerial->printf_P(PSTR(" - %u commands in memory\n"),(unsigned)pkt.command_total);
}

struct log_Control_Tuning {
    LOG_PACKET_HEADER;
    int16_t steer_out;
    int16_t roll;
    int16_t pitch;
    int16_t throttle_out;
    int16_t accel_y;
};

// Write a control tuning packet. Total length : 22 bytes
#if HIL_MODE != HIL_MODE_ATTITUDE
static void Log_Write_Control_Tuning()
{
    Vector3f accel = ins.get_accel();
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        steer_out       : (int16_t)g.channel_steer.servo_out,
        roll            : (int16_t)ahrs.roll_sensor,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)g.channel_throttle.servo_out,
        accel_y         : (int16_t)(accel.y * 10000)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

// Read an control tuning packet
static void Log_Read_Control_Tuning()
{
    struct log_Control_Tuning pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    cliSerial->printf_P(PSTR("CTUN, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f\n"),
        (float)pkt.steer_out / 100.f,
        (float)pkt.roll / 100.f,
        (float)pkt.pitch / 100.f,
        (float)pkt.throttle_out / 100.f,
        (float)pkt.accel_y / 10000.f
    );
}

struct log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint16_t yaw;
    uint32_t wp_distance;
    uint16_t target_bearing_cd;
    uint16_t nav_bearing_cd;
    int16_t altitude_error_cm;
    int16_t nav_gain_scheduler;
};

// Write a navigation tuning packet. Total length : 18 bytes
static void Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        yaw                 : (uint16_t)ahrs.yaw_sensor,
        wp_distance         : (uint32_t)wp_distance,
        target_bearing_cd   : (uint16_t)target_bearing,
        nav_bearing_cd      : (uint16_t)nav_bearing,
        altitude_error_cm   : (int16_t)altitude_error,
        nav_gain_scheduler  : (int16_t)(nav_gain_scaler*1000)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a nav tuning packet
static void Log_Read_Nav_Tuning()
{
    struct log_Nav_Tuning pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    cliSerial->printf_P(PSTR("NTUN, %4.4f, %lu, %4.4f, %4.4f, %4.4f, %4.4f\n"),
                    (float)pkt.yaw/100.0f,
                    (unsigned long)pkt.wp_distance,
                    (float)(pkt.target_bearing_cd/100.0f),
                    (float)(pkt.nav_bearing_cd/100.0f),
                    (float)(pkt.altitude_error_cm/100.0f),
                    (float)(pkt.nav_gain_scheduler/100.0f));
}

struct log_Mode {
    LOG_PACKET_HEADER;
    uint8_t mode;
};

// Write a mode packet. Total length : 5 bytes
static void Log_Write_Mode(uint8_t mode)
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        mode : mode
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a mode packet
static void Log_Read_Mode()
{
    struct log_Mode pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("MOD,"));
    print_mode(pkt.mode);
}

struct log_GPS {
    LOG_PACKET_HEADER;
    uint32_t gps_time;
    uint8_t  gps_fix;
    uint8_t  num_sats;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  rel_altitude;
    int32_t  altitude;
    uint32_t ground_speed;
    int32_t  ground_course;
};

// Write an GPS packet. Total length : 30 bytes
static void Log_Write_GPS(	uint32_t log_Time, int32_t log_Lattitude, int32_t log_Longitude, int32_t log_gps_alt, int32_t log_mix_alt,
                            uint32_t log_Ground_Speed, int32_t log_Ground_Course, uint8_t log_Fix, uint8_t log_NumSats)
{
    struct log_GPS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GPS_MSG),
    	gps_time      : log_Time,
        gps_fix       : log_Fix,
        num_sats      : log_NumSats,
        latitude      : log_Lattitude,
        longitude     : log_Longitude,
        rel_altitude  : log_mix_alt,
        altitude      : log_gps_alt,
        ground_speed  : log_Ground_Speed,
        ground_course : log_Ground_Course
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a GPS packet
static void Log_Read_GPS()
{
    struct log_GPS pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("GPS, %ld, %u, %u, "),
                        (long)pkt.gps_time,
                        (unsigned)pkt.gps_fix,
                        (unsigned)pkt.num_sats);
    print_latlon(cliSerial, pkt.latitude);
    cliSerial->print_P(PSTR(", "));
    print_latlon(cliSerial, pkt.longitude);
    cliSerial->printf_P(PSTR(", %4.4f, %4.4f, %lu, %ld\n"),
                        (float)pkt.rel_altitude*0.01,
                        (float)pkt.altitude*0.01,
                        (unsigned long)pkt.ground_speed,
                        (long)pkt.ground_course);
}

struct log_IMU {
    LOG_PACKET_HEADER;
    Vector3f gyro;
    Vector3f accel;
};

// Write an raw accel/gyro data packet. Total length : 28 bytes
#if HIL_MODE != HIL_MODE_ATTITUDE
static void Log_Write_IMU()
{
    struct log_IMU pkt = {
        LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
        gyro  : ins.get_gyro(),
        accel : ins.get_accel()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

// Read a raw accel/gyro packet
static void Log_Read_IMU()
{
    struct log_IMU pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("IMU, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f\n"),
                        pkt.gyro.x,
                        pkt.gyro.y,
                        pkt.gyro.z,
                        pkt.accel.x,
                        pkt.accel.y,
                        pkt.accel.z);
}

struct log_Current {
    LOG_PACKET_HEADER;
    int16_t throttle_in;
    int16_t battery_voltage;
    int16_t current_amps;
    int16_t current_total;
};

static void Log_Write_Current()
{
    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        throttle_in             : g.channel_throttle.control_in,
        battery_voltage         : (int16_t)(battery_voltage1 * 100.0),
        current_amps            : (int16_t)(current_amps1 * 100.0),
        current_total           : (int16_t)current_total1
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a Current packet
static void Log_Read_Current()
{
    struct log_Current pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("CURRENT, %d, %4.4f, %4.4f, %d\n"),
                    (int)pkt.throttle_in,
                    ((float)pkt.battery_voltage / 100.f),
                    ((float)pkt.current_amps / 100.f),
                    (int)pkt.current_total);
}

// Read the DataFlash log memory : Packet Parser
static void Log_Read(int16_t start_page, int16_t end_page)
{
	cliSerial->printf_P(PSTR("\n" THISFIRMWARE
                             "\nFree RAM: %u\n"),
                        memcheck_available_memory());

	DataFlash.log_read_process(start_page, end_page, log_callback);
}

// Read the DataFlash log memory : Packet Parser
static void log_callback(uint8_t msgid)
{
    switch (msgid) {
    case LOG_ATTITUDE_MSG: 
        Log_Read_Attitude();
        break;
        
    case LOG_MODE_MSG:
        Log_Read_Mode();
        break;

    case LOG_CONTROL_TUNING_MSG:
        Log_Read_Control_Tuning();
        break;

    case LOG_NAV_TUNING_MSG:
        Log_Read_Nav_Tuning();
        break;

    case LOG_PERFORMANCE_MSG:
        Log_Read_Performance();
        break;
    
    case LOG_IMU_MSG:
        Log_Read_IMU();
        break;

    case LOG_CMD_MSG:
        Log_Read_Cmd();
        break;

    case LOG_CURRENT_MSG:
        Log_Read_Current();
        break;

    case LOG_STARTUP_MSG:
        Log_Read_Startup();
        break;

    case LOG_GPS_MSG:
        Log_Read_GPS();
        break;
    }
}

#else // LOGGING_ENABLED

// dummy functions
static void Log_Write_Mode(uint8_t mode) {}
static void Log_Write_Startup(uint8_t type) {}
static void Log_Write_Cmd(uint8_t num, struct Location *wp) {}
static void Log_Write_Current() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_GPS(	uint32_t log_Time, int32_t log_Lattitude, int32_t log_Longitude, int32_t log_gps_alt, int32_t log_mix_alt,
                            uint32_t log_Ground_Speed, int32_t log_Ground_Course, uint8_t log_Fix, uint8_t log_NumSats) {}
static void Log_Write_Performance() {}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
static void Log_Write_Attitude(int16_t log_roll, int16_t log_pitch, uint16_t log_yaw) {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_IMU() {}


#endif // LOGGING_ENABLED
#endif
#line 1 "./Firmware/APMRover2/Parameters.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
  ArduPlane parameter definitions

  This firmware is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
*/

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value:def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info:class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info:class::var_info} }

const AP_Param::Info var_info[] PROGMEM = {
	GSCALAR(format_version,         "FORMAT_VERSION",   1),
	GSCALAR(software_type,          "SYSID_SW_TYPE",    Parameters::k_software_type),

	// misc
	GSCALAR(log_bitmask,            "LOG_BITMASK",      DEFAULT_LOG_BITMASK),
	GSCALAR(num_resets,             "SYS_NUM_RESETS",   0),
	GSCALAR(reset_switch_chan,      "RST_SWITCH_CH",    0),

    // @Param: RSSI_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: This selects an analog pin for the receiver RSSI voltage. It assumes the voltage is 5V for max rssi, 0V for minimum
    // @Values: -1:Disabled, 0:A0, 1:A1, 13:A13
    // @User: Standard
    GSCALAR(rssi_pin,            "RSSI_PIN",         -1),

    // @Param: BATT_VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 0:A0, 1:A1, 13:A13
    // @User: Standard
    GSCALAR(battery_volt_pin,    "BATT_VOLT_PIN",    1),

    // @Param: BATT_CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 1:A1, 2:A2, 12:A12
    // @User: Standard
    GSCALAR(battery_curr_pin,    "BATT_CURR_PIN",    2),



    // @Param: SYSID_THIS_MAV
    // @DisplayName: MAVLink system ID
    // @Description: ID used in MAVLink protocol to identify this vehicle
    // @User: Advanced
	GSCALAR(sysid_this_mav,         "SYSID_THISMAV",    MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: MAVLink ground station ID
    // @Description: ID used in MAVLink protocol to identify the controlling ground station
    // @User: Advanced
	GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",      255),

    // @Param: SERIAL0_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the first serial port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
	GSCALAR(serial0_baud,           "SERIAL0_BAUD",     SERIAL0_BAUD/1000),

    // @Param: SERIAL3_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the telemetry port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
	GSCALAR(serial3_baud,           "SERIAL3_BAUD",     SERIAL3_BAUD/1000),

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay 
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Standard
    // @Units: seconds
    // @Range: 0 10
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: MAG_ENABLED
    // @DisplayName: Magnetometer (compass) enabled
    // @Description: This should be set to 1 if a compass is installed
    // @User: Standard
    // @Values: 0:Disabled,1:Enabled
	GSCALAR(compass_enabled,        "MAG_ENABLE",       MAGNETOMETER),

    // @Param: BATT_MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Voltage Only,4:Voltage and Current
    // @User: Standard
	GSCALAR(battery_monitoring,     "BATT_MONITOR",     DISABLED),

    // @Param: VOLT_DIVIDER
    // @DisplayName: Voltage Divider
    // @Description: Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin voltage * INPUT_VOLTS/1024 * VOLT_DIVIDER)
    // @User: Advanced
	GSCALAR(volt_div_ratio,         "VOLT_DIVIDER",     VOLT_DIV_RATIO),

    // @Param: AMP_PER_VOLT
    // @DisplayName: Current Amps per volt
    // @Description: Used to convert the voltage on the current sensing pin (BATT_CURR_PIN) to the actual current being consumed in amps (curr pin voltage * INPUT_VOLTS/1024 * AMP_PER_VOLT )
    // @User: Advanced
	GSCALAR(curr_amp_per_volt,      "AMP_PER_VOLT",     CURR_AMP_PER_VOLT),

    // @Param: INPUT_VOLTS
    // @DisplayName: Max internal voltage of the battery voltage and current sensing pins
    // @Description: Used to convert the voltage read in on the voltage and current pins for battery monitoring.  Normally 5 meaning 5 volts.
    // @User: Advanced
	GSCALAR(input_voltage,          "INPUT_VOLTS",      INPUT_VOLTAGE),

    // @Param: BATT_CAPACITY
    // @DisplayName: Battery Capacity
    // @Description: Battery capacity in milliamp-hours (mAh)
    // @Units: mAh
	GSCALAR(pack_capacity,          "BATT_CAPACITY",    HIGH_DISCHARGE),

    // @Param: XTRK_GAIN_SC
    // @DisplayName: Crosstrack Gain
    // @Description: This controls how hard the Rover tries to follow the lines between waypoints, as opposed to driving directly to the next waypoint. The value is the scale between distance off the line and angle to meet the line (in Degrees * 100)
    // @Range: 0 2000
    // @Increment: 1
    // @User: Standard
	GSCALAR(crosstrack_gain,        "XTRK_GAIN_SC",     XTRACK_GAIN_SCALED),

    // @Param: XTRK_ANGLE_CD
    // @DisplayName: Crosstrack Entry Angle
    // @Description: Maximum angle used to correct for track following.
    // @Units: centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
	GSCALAR(crosstrack_entry_angle, "XTRK_ANGLE_CD",    XTRACK_ENTRY_ANGLE_CENTIDEGREE),

    // @Param: CRUISE_SPEED
    // @DisplayName: Target speed in auto modes
    // @Description: The target speed in auto missions.
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
	GSCALAR(speed_cruise,        "CRUISE_SPEED",    5),

    // @Param: CH7_OPTION
    // @DisplayName: Channel 7 option
    // @Description: What to do use channel 7 for
    // @Values: 0:Nothing,1:LearnWaypoint
    // @User: Standard
	GSCALAR(ch7_option,             "CH7_OPTION",          CH7_OPTION),

	GGROUP(channel_steer,           "RC1_", RC_Channel),
	GGROUP(rc_2,                    "RC2_", RC_Channel_aux),
	GGROUP(channel_throttle,        "RC3_", RC_Channel),
	GGROUP(rc_4,                    "RC4_", RC_Channel_aux),
	GGROUP(rc_5,                    "RC5_", RC_Channel_aux),
	GGROUP(rc_6,                    "RC6_", RC_Channel_aux),
	GGROUP(rc_7,                    "RC7_", RC_Channel_aux),
	GGROUP(rc_8,                    "RC8_", RC_Channel_aux),

    // @Param: THR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: The minimum throttle setting to which the autopilot will apply. This is mostly useful for rovers with internal combustion motors, to prevent the motor from cutting out in auto mode.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_min,           "THR_MIN",          THROTTLE_MIN),

    // @Param: THR_MAX
    // @DisplayName: Maximum Throttle
    // @Description: The maximum throttle setting to which the autopilot will apply.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_max,           "THR_MAX",          THROTTLE_MAX),

    // @Param: CRUISE_THROTTLE
    // @DisplayName: Base throttle percentage in auto
    // @Description: The base throttle percentage to use in auto mode
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_cruise,        "CRUISE_THROTTLE",    50),

    // @Param: THR_SLEWRATE
    // @DisplayName: Throttle slew rate
    // @Description: maximum percentage change in throttle per second. A setting of 10 means to not change the throttle by more than 10% of the full throttle range in one second. A value of zero means no limit.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_slewrate,      "THR_SLEWRATE",     0),

    // @Param: FS_ACTION
    // @DisplayName: Failsafe Action
    // @Description: What to do on a failsafe event
    // @Values: 0:Nothing,1:RTL,2:STOP
    // @User: Standard
	GSCALAR(fs_action,    "FS_ACTION",     0),

    // @Param: FS_TIMEOUT
    // @DisplayName: Failsafe timeout
    // @Description: How long a failsafe event need to happen for before we trigger the failsafe action
	// @Units: seconds
    // @User: Standard
	GSCALAR(fs_timeout,    "FS_TIMEOUT",     10),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel to a low value. This can be used to detect the RC transmitter going out of range.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
	GSCALAR(fs_throttle_enabled,    "FS_THR_ENABLE",     0),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers.
    // @User: Standard
	GSCALAR(fs_throttle_value,      "FS_THR_VALUE",     900),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: GCS failsafe enable
    // @Description: Enable ground control station telemetry failsafe
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
	GSCALAR(fs_gcs_enabled, "FS_GCS_ENABLE",   0),

#if CONFIG_SONAR == ENABLED     
	// @Param: SONAR_ENABLE
	// @DisplayName: Enable Sonar
	// @Description: Setting this to Enabled(1) will enable the sonar. Setting this to Disabled(0) will disable the sonar
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	GSCALAR(sonar_trigger,      "SONAR_TRIGGER",    SONAR_TRIGGER),
	GSCALAR(sonar_enabled,	    "SONAR_ENABLE",     SONAR_ENABLED),
	GSCALAR(sonar_type,	        "SONAR_TYPE",       AP_RANGEFINDER_MAXSONARXL),
#endif	


    // @Param: MODE_CH
    // @DisplayName: Mode channel
    // @Description: RC Channel to use for driving mode control
    // @User: Advanced
	GSCALAR(mode_channel,    "MODE_CH",       MODE_CHANNEL),

    // @Param: MODE1
    // @DisplayName: Mode1
    // @Values: 0:Manual,2:LEARNING,10:Auto,11:RTL,15:Guided
    // @User: Standard
    // @Description: Driving mode for switch position 1 (910 to 1230 and above 2049)
	GSCALAR(mode1,           "MODE1",         MODE_1),

    // @Param: MODE2
    // @DisplayName: Mode2
    // @Description: Driving mode for switch position 2 (1231 to 1360)
    // @Values: 0:Manual,2:LEARNING,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode2,           "MODE2",         MODE_2),

    // @Param: MODE3
    // @DisplayName: Mode3
    // @Description: Driving mode for switch position 3 (1361 to 1490)
    // @Values: 0:Manual,2:LEARNING,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode3,           "MODE3",         MODE_3),

    // @Param: MODE4
    // @DisplayName: Mode4
    // @Description: Driving mode for switch position 4 (1491 to 1620)
    // @Values: 0:Manual,2:LEARNING,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode4,           "MODE4",         MODE_4),

    // @Param: MODE5
    // @DisplayName: Mode5
    // @Description: Driving mode for switch position 5 (1621 to 1749)
    // @Values: 0:Manual,2:LEARNING,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode5,           "MODE5",         MODE_5),

    // @Param: MODE6
    // @DisplayName: Mode6
    // @Description: Driving mode for switch position 6 (1750 to 2049)
    // @Values: 0:Manual,2:LEARNING,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode6,           "MODE6",         MODE_6),

	GSCALAR(command_total,          "CMD_TOTAL",        0),
	GSCALAR(command_index,          "CMD_INDEX",        0),

    // @Param: WP_RADIUS
    // @DisplayName: Waypoint radius
    // @Description: The distance in meters from a waypoint when we consider the waypoint has been reached. This determines when the rover will turn along the next waypoint path.
    // @Units: meters
    // @Range: 0 1000
    // @Increment: 0.1
    // @User: Standard
	GSCALAR(waypoint_radius,        "WP_RADIUS",        2.0f),

	GGROUP(pidNavSteer,             "HDNG2STEER_",  PID),
	GGROUP(pidServoSteer,           "STEER2SRV_",   PID),
	GGROUP(pidSpeedThrottle,        "SPEED2THR_", PID),

	// variables not in the g class which contain EEPROM saved variables
	GOBJECT(compass,                "COMPASS_",	Compass),
	GOBJECT(gcs0,					"SR0_",     GCS_MAVLINK),
	GOBJECT(gcs3,					"SR3_",     GCS_MAVLINK),

#if HIL_MODE == HIL_MODE_DISABLED
    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                            "INS_", AP_InertialSensor),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL),
#endif

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

	AP_VAREND
};


static void load_parameters(void)
{
	if (!g.format_version.load() ||
	     g.format_version != Parameters::k_format_version) {

		// erase all parameters
		cliSerial->printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
		AP_Param::erase_all();

		// save the current format version
		g.format_version.set_and_save(Parameters::k_format_version);
		cliSerial->println_P(PSTR("done."));
    } else {
	    unsigned long before = micros();
	    // Load all auto-loaded EEPROM variables
	    AP_Param::load_all();

	    cliSerial->printf_P(PSTR("load_all took %luus\n"), micros() - before);
	}
}
#line 1 "./Firmware/APMRover2/Steering.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*****************************************
* Throttle slew limit
*****************************************/
static void throttle_slew_limit(int16_t last_throttle)
{
    // if slew limit rate is set to zero then do not slew limit
    if (g.throttle_slewrate) {                   
        // limit throttle change by the given percentage per second
        float temp = g.throttle_slewrate * G_Dt * 0.01f * fabsf(g.channel_throttle.radio_max - g.channel_throttle.radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        g.channel_throttle.radio_out = constrain_int16(g.channel_throttle.radio_out, last_throttle - temp, last_throttle + temp);
    }
}

static void calc_throttle()
{  
   int throttle_target = g.throttle_cruise + throttle_nudge;  
   
   groundspeed_error = g.speed_cruise - ground_speed; 
        
   throttle = throttle_target + (g.pidSpeedThrottle.get_pid(groundspeed_error * 100) / 100);
   g.channel_throttle.servo_out = constrain_int16(throttle, g.throttle_min.get(), g.throttle_max.get());
}

/*****************************************
 * Calculate desired turn angles (in medium freq loop)
 *****************************************/

static void calc_nav_steer()
{
	// Adjust gain based on ground speed
	nav_gain_scaler = (float)ground_speed / g.speed_cruise;
	nav_gain_scaler = constrain(nav_gain_scaler, 0.2, 1.4);

	// Calculate the required turn of the wheels rover
	// ----------------------------------------

    // negative error = left turn
	// positive error = right turn
	nav_steer = g.pidNavSteer.get_pid(bearing_error_cd, nav_gain_scaler);

    if (obstacle) {  // obstacle avoidance 
	    nav_steer += 9000;    // if obstacle in front turn 90° right	
    }
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void set_servos(void)
{
    int16_t last_throttle = g.channel_throttle.radio_out;

	if ((control_mode == MANUAL) || (control_mode == LEARNING)) {
		// do a direct pass through of radio values
		g.channel_steer.radio_out 		= g.channel_steer.radio_in;

        if (obstacle)    // obstacle in front, turn right in Stabilize mode
            g.channel_steer.radio_out -= 500;

		g.channel_throttle.radio_out 	= g.channel_throttle.radio_in;
	} else {       
        g.channel_steer.calc_pwm();
		g.channel_throttle.radio_out = g.channel_throttle.radio_in;
		g.channel_throttle.servo_out = constrain_int16(g.channel_throttle.servo_out, 
                                                       g.throttle_min.get(), 
                                                       g.throttle_max.get());
    }
                
    if (control_mode >= AUTO) {
        // convert 0 to 100% into PWM
        g.channel_throttle.calc_pwm();

        // limit throttle movement speed
        throttle_slew_limit(last_throttle);
    }


#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
	// send values to the PWM timers for output
	// ----------------------------------------
    hal.rcout->write(CH_1, g.channel_steer.radio_out);     // send to Servos
    hal.rcout->write(CH_3, g.channel_throttle.radio_out);     // send to Servos

	// Route configurable aux. functions to their respective servos
	g.rc_2.output_ch(CH_2);
	g.rc_4.output_ch(CH_4);
	g.rc_5.output_ch(CH_5);
	g.rc_6.output_ch(CH_6);
	g.rc_7.output_ch(CH_7);
	g.rc_8.output_ch(CH_8);

#endif
}

static bool demoing_servos;

static void demo_servos(uint8_t i) {

    while(i > 0) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Demo Servos!"));
        demoing_servos = true;
#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
        hal.rcout->write(1, 1400);
        mavlink_delay(400);
        hal.rcout->write(1, 1600);
        mavlink_delay(200);
        hal.rcout->write(1, 1500);
#endif
        demoing_servos = false;
        mavlink_delay(400);
        i--;
    }
}
#line 1 "./Firmware/APMRover2/commands.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* Functions in this file:
	void init_commands()
	struct Location get_cmd_with_index(int i)
	void set_cmd_with_index(struct Location temp, int i)
	void increment_cmd_index()
	void decrement_cmd_index()
	long read_alt_to_hold()
	void set_next_WP(struct Location *wp)
	void set_guided_WP(void)
	void init_home()
	void restart_nav()
************************************************************ 
*/

static void init_commands()
{
    g.command_index.set_and_save(0);
	nav_command_ID	= NO_COMMAND;
	non_nav_command_ID	= NO_COMMAND;
	next_nav_command.id 	= CMD_BLANK;
}

// Getters
// -------
static struct Location get_cmd_with_index(int i)
{
	struct Location temp;
	uint16_t mem;

	// Find out proper location in memory by using the start_byte position + the index
	// --------------------------------------------------------------------------------
	if (i > g.command_total) {
		memset(&temp, 0, sizeof(temp));
		temp.id = CMD_BLANK;
	}else{
		// read WP position
		mem = (WP_START_BYTE) + (i * WP_SIZE);
		temp.id = hal.storage->read_byte(mem);

		mem++;
		temp.options = hal.storage->read_byte(mem);

		mem++;
		temp.p1 = hal.storage->read_byte(mem);

		mem++;
		temp.alt = (long)hal.storage->read_dword(mem);

		mem += 4;
		temp.lat = (long)hal.storage->read_dword(mem);

		mem += 4;
		temp.lng = (long)hal.storage->read_dword(mem);
	}

	// Add on home altitude if we are a nav command (or other command with altitude) and stored alt is relative
	if((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) && temp.options & MASK_OPTIONS_RELATIVE_ALT){
		temp.alt += home.alt;
	}

	return temp;
}

// Setters
// -------
static void set_cmd_with_index(struct Location temp, int i)
{
	i = constrain_int16(i, 0, g.command_total.get());
	uint16_t mem = WP_START_BYTE + (i * WP_SIZE);

	// Set altitude options bitmask
	// XXX What is this trying to do?
	if (temp.options & MASK_OPTIONS_RELATIVE_ALT && i != 0){
		temp.options = MASK_OPTIONS_RELATIVE_ALT;
	} else {
		temp.options = 0;
	}

	hal.storage->write_byte(mem, temp.id);

    mem++;
	hal.storage->write_byte(mem, temp.options);

	mem++;
	hal.storage->write_byte(mem, temp.p1);

	mem++;
	hal.storage->write_dword(mem, temp.alt);

	mem += 4;
	hal.storage->write_dword(mem, temp.lat);

	mem += 4;
	hal.storage->write_dword(mem, temp.lng);
}

/*
This function stores waypoint commands
It looks to see what the next command type is and finds the last command.
*/
static void set_next_WP(struct Location *wp)
{
	// copy the current WP into the OldWP slot
	// ---------------------------------------
	prev_WP = next_WP;

	// Load the next_WP slot
	// ---------------------
	next_WP = *wp;

    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Resetting prev_WP"));
        prev_WP = current_loc;
    }

	// this is handy for the groundstation
	wp_totalDistance 	= get_distance(&current_loc, &next_WP);
	wp_distance 		= wp_totalDistance;
	target_bearing 		= get_bearing_cd(&current_loc, &next_WP);
	nav_bearing 		= target_bearing;

	// set a new crosstrack bearing
	// ----------------------------
	reset_crosstrack();
}

static void set_guided_WP(void)
{
	// copy the current location into the OldWP slot
	// ---------------------------------------
	prev_WP = current_loc;

	// Load the next_WP slot
	// ---------------------
	next_WP = guided_WP;

	// this is handy for the groundstation
	wp_totalDistance 	= get_distance(&current_loc, &next_WP);
	wp_distance 		= wp_totalDistance;
	target_bearing 		= get_bearing_cd(&current_loc, &next_WP);

	// set a new crosstrack bearing
	// ----------------------------
	reset_crosstrack();
}

// run this at setup on the ground
// -------------------------------
void init_home()
{
    if (!have_position) {
        // we need position information
        return;
    }

	gcs_send_text_P(SEVERITY_LOW, PSTR("init home"));

	home.id 	= MAV_CMD_NAV_WAYPOINT;

	home.lng 	= g_gps->longitude;				// Lon * 10**7
	home.lat 	= g_gps->latitude;				// Lat * 10**7
    gps_base_alt    = max(g_gps->altitude, 0);
    home.alt        = g_gps->altitude;
	home_is_set = true;

	// Save Home to EEPROM - Command 0
	// -------------------
	set_cmd_with_index(home, 0);

	// Save prev loc
	// -------------
	next_WP = prev_WP = home;

	// Load home for a default guided_WP
	// -------------
	guided_WP = home;
}

static void restart_nav()
{  
    g.pidNavSteer.reset_I();
    g.pidSpeedThrottle.reset_I();
    prev_WP = current_loc;
    nav_command_ID = NO_COMMAND;
    nav_command_index = 0;
    process_next_command();
}
#line 1 "./Firmware/APMRover2/commands_logic.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
static void
handle_process_nav_cmd()
{
	// reset navigation integrators
	// -------------------------
    g.pidNavSteer.reset_I();

    gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nav_command.id);

	switch(next_nav_command.id){
		case MAV_CMD_NAV_TAKEOFF:
			do_takeoff();
			break;

		case MAV_CMD_NAV_WAYPOINT:	// Navigate to Waypoint
			do_nav_wp();
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			do_RTL();
			break;

		default:
			break;
	}
}

static void
handle_process_condition_command()
{
	gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nonnav_command.id);
	switch(next_nonnav_command.id){

		case MAV_CMD_CONDITION_DELAY:
			do_wait_delay();
			break;

		case MAV_CMD_CONDITION_DISTANCE:
			do_within_distance();
			break;

		case MAV_CMD_CONDITION_CHANGE_ALT:
			do_change_alt();
			break;

		default:
			break;
	}
}

static void handle_process_do_command()
{
	gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nonnav_command.id);
	switch(next_nonnav_command.id){

		case MAV_CMD_DO_JUMP:
			do_jump();
			break;

		case MAV_CMD_DO_CHANGE_SPEED:
			do_change_speed();
			break;

		case MAV_CMD_DO_SET_HOME:
			do_set_home();
			break;

		case MAV_CMD_DO_SET_SERVO:
			do_set_servo();
			break;

		case MAV_CMD_DO_SET_RELAY:
			do_set_relay();
			break;

		case MAV_CMD_DO_REPEAT_SERVO:
			do_repeat_servo();
			break;

		case MAV_CMD_DO_REPEAT_RELAY:
			do_repeat_relay();
			break;

#if MOUNT == ENABLED
		// Sets the region of interest (ROI) for a sensor set or the
		// vehicle itself. This can then be used by the vehicles control
		// system to control the vehicle attitude and the attitude of various
		// devices such as cameras.
		//    |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
		case MAV_CMD_DO_SET_ROI:
			camera_mount.set_roi_cmd();
			break;

		case MAV_CMD_DO_MOUNT_CONFIGURE:	// Mission command to configure a camera mount |Mount operation mode (see MAV_CONFIGURE_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
			camera_mount.configure_cmd();
			break;

		case MAV_CMD_DO_MOUNT_CONTROL:		// Mission command to control a camera mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|
			camera_mount.control_cmd();
			break;
#endif
	}
}

static void handle_no_commands()
{      
	gcs_send_text_fmt(PSTR("No commands - setting MANUAL"));
    set_mode(MANUAL);
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

static bool verify_nav_command()	// Returns true if command complete
{
	switch(nav_command_ID) {

		case MAV_CMD_NAV_TAKEOFF:
			return verify_takeoff();

		case MAV_CMD_NAV_WAYPOINT:
			return verify_nav_wp();

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			return verify_RTL();

		default:
			gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_nav: Invalid or no current Nav cmd"));
			return false;
	}
}

static bool verify_condition_command()		// Returns true if command complete
{
	switch(non_nav_command_ID) {
    case NO_COMMAND:
        break;

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        return verify_change_alt();
        break;
        
    case WAIT_COMMAND:
        return 0;
        break;
        

    default:
        gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_conditon: Invalid or no current Condition cmd"));
        break;
	}
    return false;
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

static void do_RTL(void)
{
    prev_WP 		= current_loc;
	control_mode 	= RTL;
	next_WP 		= home;
}

static void do_takeoff()
{
	set_next_WP(&next_nav_command);
}

static void do_nav_wp()
{
	set_next_WP(&next_nav_command);
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
static bool verify_takeoff()
{  return true;
}

static bool verify_nav_wp()
{
    update_crosstrack();

    if ((wp_distance > 0) && (wp_distance <= g.waypoint_radius)) {
        gcs_send_text_fmt(PSTR("Reached Waypoint #%i dist %um"),
                          (unsigned)nav_command_index,
                          (unsigned)get_distance(&current_loc, &next_WP));
        return true;
    }

    // have we gone past the waypoint?
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_fmt(PSTR("Passed Waypoint #%i dist %um"),
                          (unsigned)nav_command_index,
                          (unsigned)get_distance(&current_loc, &next_WP));
        return true;
    }

    return false;
}

static bool verify_RTL()
{
	if (wp_distance <= g.waypoint_radius) {
		gcs_send_text_P(SEVERITY_LOW,PSTR("Reached home"));
                rtl_complete = true;
		return true;
	}else{
		return false;
	}
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

static void do_wait_delay()
{
	condition_start = millis();
	condition_value  = next_nonnav_command.lat * 1000;	// convert to milliseconds
}

static void do_change_alt()
{
	condition_rate		= abs((int)next_nonnav_command.lat);
	condition_value 	= next_nonnav_command.alt;
	if(condition_value < current_loc.alt) condition_rate = -condition_rate;
	next_WP.alt 		= condition_value;								// For future nav calculations
}

static void do_within_distance()
{
	condition_value  = next_nonnav_command.lat;
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_wait_delay()
{
	if ((uint32_t)(millis() - condition_start) > (uint32_t)condition_value){
		condition_value 	= 0;
		return true;
	}
	return false;
}

static bool verify_change_alt()
{
	if( (condition_rate>=0 && current_loc.alt >= condition_value) || (condition_rate<=0 && current_loc.alt <= condition_value)) {
		condition_value = 0;
		return true;
	}
	return false;
}

static bool verify_within_distance()
{
	if (wp_distance < condition_value){
		condition_value = 0;
		return true;
	}
	return false;
}

/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

static void do_jump()
{
	struct Location temp;
	gcs_send_text_fmt(PSTR("In jump.  Jumps left: %i"),next_nonnav_command.lat);
	if(next_nonnav_command.lat > 0) {

		nav_command_ID		= NO_COMMAND;
		next_nav_command.id = NO_COMMAND;
		non_nav_command_ID 	= NO_COMMAND;
		
		temp 				= get_cmd_with_index(g.command_index);
		temp.lat 			= next_nonnav_command.lat - 1;					// Decrement repeat counter

		set_cmd_with_index(temp, g.command_index);
	gcs_send_text_fmt(PSTR("setting command index: %i"),next_nonnav_command.p1 - 1);
		g.command_index.set_and_save(next_nonnav_command.p1 - 1);
		nav_command_index 	= next_nonnav_command.p1 - 1;
		next_WP = prev_WP;		// Need to back "next_WP" up as it was set to the next waypoint following the jump
		process_next_command();
	} else if (next_nonnav_command.lat == -1) {								// A repeat count of -1 = repeat forever
		nav_command_ID 	= NO_COMMAND;
		non_nav_command_ID 	= NO_COMMAND;
	gcs_send_text_fmt(PSTR("setting command index: %i"),next_nonnav_command.p1 - 1);
	    g.command_index.set_and_save(next_nonnav_command.p1 - 1);
		nav_command_index 	= next_nonnav_command.p1 - 1;
		next_WP = prev_WP;		// Need to back "next_WP" up as it was set to the next waypoint following the jump
		process_next_command();
	}
}

static void do_change_speed()
{
	switch (next_nonnav_command.p1)
	{
		case 0:
			if (next_nonnav_command.alt > 0)
				g.speed_cruise.set(next_nonnav_command.alt * 100);
			break;
	}

	if(next_nonnav_command.lat > 0)
		g.throttle_cruise.set(next_nonnav_command.lat);
}

static void do_set_home()
{
	if(next_nonnav_command.p1 == 1 && have_position) {
		init_home();
	} else {
		home.id 	= MAV_CMD_NAV_WAYPOINT;
		home.lng 	= next_nonnav_command.lng;				// Lon * 10**7
		home.lat 	= next_nonnav_command.lat;				// Lat * 10**7
		home.alt 	= max(next_nonnav_command.alt, 0);
		home_is_set = true;
	}
}

static void do_set_servo()
{
    hal.rcout->enable_ch(next_nonnav_command.p1 - 1);
    hal.rcout->write(next_nonnav_command.p1 - 1, next_nonnav_command.alt);
}

static void do_set_relay()
{
	if (next_nonnav_command.p1 == 1) {
		relay.on();
	} else if (next_nonnav_command.p1 == 0) {
		relay.off();
	}else{
		relay.toggle();
	}
}

static void do_repeat_servo()
{
	event_id = next_nonnav_command.p1 - 1;

	if(next_nonnav_command.p1 >= CH_5 + 1 && next_nonnav_command.p1 <= CH_8 + 1) {

		event_timer 	= 0;
		event_delay 	= next_nonnav_command.lng * 500.0;	// /2 (half cycle time) * 1000 (convert to milliseconds)
		event_repeat 	= next_nonnav_command.lat * 2;
		event_value 	= next_nonnav_command.alt;

		switch(next_nonnav_command.p1) {
			case CH_5:
				event_undo_value = g.rc_5.radio_trim;
				break;
			case CH_6:
				event_undo_value = g.rc_6.radio_trim;
				break;
			case CH_7:
				event_undo_value = g.rc_7.radio_trim;
				break;
			case CH_8:
				event_undo_value = g.rc_8.radio_trim;
				break;
		}
		update_events();
	}
}

static void do_repeat_relay()
{
	event_id 		= RELAY_TOGGLE;
	event_timer 	= 0;
	event_delay 	= next_nonnav_command.lat * 500.0;	// /2 (half cycle time) * 1000 (convert to milliseconds)
	event_repeat	= next_nonnav_command.alt * 2;
	update_events();
}

#line 1 "./Firmware/APMRover2/commands_process.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// For changing active command mid-mission
//----------------------------------------
static void change_command(uint8_t cmd_index)
{
	struct Location temp = get_cmd_with_index(cmd_index);

	if (temp.id > MAV_CMD_NAV_LAST ){
		gcs_send_text_P(SEVERITY_LOW,PSTR("Bad Request - cannot change to non-Nav cmd"));
	} else {
		gcs_send_text_fmt(PSTR("Received Request - jump to command #%i"),cmd_index);

		nav_command_ID		= NO_COMMAND;
		next_nav_command.id = NO_COMMAND;
		non_nav_command_ID 	= NO_COMMAND;

		nav_command_index 	= cmd_index - 1;
		g.command_index.set_and_save(cmd_index);
		update_commands();
	}
}

// called by 10 Hz loop
// --------------------
static void update_commands(void)
{
	if(control_mode == AUTO){
		if(home_is_set == true && g.command_total > 1){
		process_next_command();
		}
	}									// Other (eg GCS_Auto) modes may be implemented here
}

static void verify_commands(void)
{
	if(verify_nav_command()){
		nav_command_ID = NO_COMMAND;
	}

	if(verify_condition_command()){
		non_nav_command_ID = NO_COMMAND;
	}
}


static void process_next_command()
{
	// This function makes sure that we always have a current navigation command
	// and loads conditional or immediate commands if applicable

	struct Location temp;
	uint8_t old_index = 0;

	// these are Navigation/Must commands
	// ---------------------------------
	if (nav_command_ID == NO_COMMAND){ // no current navigation command loaded
		old_index = nav_command_index;
		temp.id = MAV_CMD_NAV_LAST;
		while(temp.id >= MAV_CMD_NAV_LAST && nav_command_index <= g.command_total) {
			nav_command_index++;
			temp = get_cmd_with_index(nav_command_index);
		}

		gcs_send_text_fmt(PSTR("Nav command index updated to #%i"),nav_command_index);

		if(nav_command_index > g.command_total){
            handle_no_commands();
		} else {
			next_nav_command = temp;
			nav_command_ID = next_nav_command.id;
			non_nav_command_index = NO_COMMAND;			// This will cause the next intervening non-nav command (if any) to be loaded
			non_nav_command_ID = NO_COMMAND;

			process_nav_cmd();
		}
	}

	// these are Condition/May and Do/Now commands
	// -------------------------------------------
	if (non_nav_command_index == NO_COMMAND) {		// If the index is NO_COMMAND then we have just loaded a nav command
		non_nav_command_index = old_index + 1;
		//gcs_send_text_fmt(PSTR("Non-Nav command index #%i"),non_nav_command_index);
	} else if (non_nav_command_ID == NO_COMMAND) {	// If the ID is NO_COMMAND then we have just completed a non-nav command
		non_nav_command_index++;
	}

		//gcs_send_text_fmt(PSTR("Nav command index #%i"),nav_command_index);
		//gcs_send_text_fmt(PSTR("Non-Nav command index #%i"),non_nav_command_index);
		//gcs_send_text_fmt(PSTR("Non-Nav command ID #%i"),non_nav_command_ID);
	if(nav_command_index <= (int)g.command_total && non_nav_command_ID == NO_COMMAND) {
		temp = get_cmd_with_index(non_nav_command_index);
		if(temp.id <= MAV_CMD_NAV_LAST) {		// The next command is a nav command.  No non-nav commands to do
			g.command_index.set_and_save(nav_command_index);
			non_nav_command_index = nav_command_index;
			non_nav_command_ID = WAIT_COMMAND;
			gcs_send_text_fmt(PSTR("Non-Nav command ID updated to #%i"),non_nav_command_ID);

		} else {								// The next command is a non-nav command.  Prepare to execute it.
			g.command_index.set_and_save(non_nav_command_index);
			next_nonnav_command = temp;
			non_nav_command_ID = next_nonnav_command.id;
			gcs_send_text_fmt(PSTR("Non-Nav command ID updated to #%i"),non_nav_command_ID);

			process_non_nav_command();
		}

	}
}

/**************************************************/
//  These functions implement the commands.
/**************************************************/
static void process_nav_cmd()
{
	//gcs_send_text_P(SEVERITY_LOW,PSTR("New nav command loaded"));

	// clear non-nav command ID and index
	non_nav_command_index	= NO_COMMAND;		// Redundant - remove?
	non_nav_command_ID		= NO_COMMAND;		// Redundant - remove?

	handle_process_nav_cmd();

}

static void process_non_nav_command()
{
	//gcs_send_text_P(SEVERITY_LOW,PSTR("new non-nav command loaded"));

	if(non_nav_command_ID < MAV_CMD_CONDITION_LAST) {
		handle_process_condition_command();
	} else {
		handle_process_do_command();
		// flag command ID so a new one is loaded
		// -----------------------------------------
		non_nav_command_ID = NO_COMMAND;
	}
}


#line 1 "./Firmware/APMRover2/compat.pde"


void delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

void mavlink_delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

uint32_t millis()
{
    return hal.scheduler->millis();
}

uint32_t micros()
{
    return hal.scheduler->micros();
}

void pinMode(uint8_t pin, uint8_t output)
{
    hal.gpio->pinMode(pin, output);
}

void digitalWrite(uint8_t pin, uint8_t out)
{
    hal.gpio->write(pin,out);
}

uint8_t digitalRead(uint8_t pin)
{
    return hal.gpio->read(pin);
}

#line 1 "./Firmware/APMRover2/control_modes.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void read_control_switch()
{
	
	uint8_t switchPosition = readSwitch();
	
	// If switchPosition = 255 this indicates that the mode control channel input was out of range
	// If we get this value we do not want to change modes.
	if(switchPosition == 255) return;

    // we look for changes in the switch position. If the
    // RST_SWITCH_CH parameter is set, then it is a switch that can be
    // used to force re-reading of the control switch. This is useful
    // when returning to the previous mode after a failsafe or fence
    // breach. This channel is best used on a momentary switch (such
    // as a spring loaded trainer switch).
	if (oldSwitchPosition != switchPosition ||
        (g.reset_switch_chan != 0 && 
         hal.rcin->read(g.reset_switch_chan-1) > RESET_SWITCH_CHAN_PWM)) {

		set_mode((enum mode)modes[switchPosition].get());

		oldSwitchPosition = switchPosition;
		prev_WP = current_loc;

		// reset navigation and speed integrators
		// -------------------------
        g.pidNavSteer.reset_I();
        g.pidSpeedThrottle.reset_I();
	}

}

static uint8_t readSwitch(void){
    uint16_t pulsewidth = hal.rcin->read(g.mode_channel - 1);
	if (pulsewidth <= 910 || pulsewidth >= 2090) 	return 255;	// This is an error condition
	if (pulsewidth > 1230 && pulsewidth <= 1360) 	return 1;
	if (pulsewidth > 1360 && pulsewidth <= 1490) 	return 2;
	if (pulsewidth > 1490 && pulsewidth <= 1620) 	return 3;
	if (pulsewidth > 1620 && pulsewidth <= 1749) 	return 4;	// Software Manual
	if (pulsewidth >= 1750) 						return 5;	// Hardware Manual
	return 0;
}

static void reset_control_switch()
{
	oldSwitchPosition = 0;
	read_control_switch();
}

#define CH_7_PWM_TRIGGER 1800

// read at 10 hz
// set this to your trainer switch
static void read_trim_switch()
{
    switch ((enum ch7_option)g.ch7_option.get()) {
    case CH7_DO_NOTHING:
        break;
    case CH7_SAVE_WP:
		if (g.rc_7.radio_in > CH_7_PWM_TRIGGER) {
            // switch is engaged
			ch7_flag = true;
		} else { // switch is disengaged
			if (ch7_flag) {
				ch7_flag = false;

				if (control_mode == MANUAL) {
                    hal.console->println_P(PSTR("Erasing waypoints"));
                    // if SW7 is ON in MANUAL = Erase the Flight Plan
					g.command_total.set_and_save(CH7_wp_index);
                    g.command_total = 0;
                    g.command_index =0;
                    nav_command_index = 0;
                    if (g.channel_steer.control_in > 3000) {
						// if roll is full right store the current location as home
                        init_home();
                    }
                    CH7_wp_index = 1;     
					return;
				} else if (control_mode == LEARNING) {    
                    // if SW7 is ON in LEARNING = record the Wp
                    // set the next_WP (home is stored at 0)

                    hal.console->printf_P(PSTR("Learning waypoint %u"), (unsigned)CH7_wp_index);        
                    current_loc.id = MAV_CMD_NAV_WAYPOINT;  
    
                    // store the index
                    g.command_total.set_and_save(CH7_wp_index);
                    g.command_total = CH7_wp_index;
                    g.command_index = CH7_wp_index;
                    nav_command_index = 0;
                                   
                    // save command
                    set_cmd_with_index(current_loc, CH7_wp_index);
                                  
                    // increment index
                    CH7_wp_index++; 
                    CH7_wp_index = constrain_int16(CH7_wp_index, 1, MAX_WAYPOINTS);

                } else if (control_mode == AUTO) {    
                    // if SW7 is ON in AUTO = set to RTL  
                    set_mode(RTL);
                }
            }
        }
        break;
    }
}

#line 1 "./Firmware/APMRover2/events.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


static void failsafe_long_on_event(int fstype)
{
	// This is how to handle a long loss of control signal failsafe.
	gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Long event on, "));
    hal.rcin->clear_overrides();
	failsafe = fstype;
	switch(control_mode)
	{
		case MANUAL: 
		case LEARNING:
			set_mode(RTL);
			break;

		case AUTO: 
		case GUIDED: 
            set_mode(RTL);
			break;
			
		case RTL: 
		default:
			break;
	}
    gcs_send_text_fmt(PSTR("flight mode = %u"), (unsigned)control_mode);
}

static void update_events(void)	// Used for MAV_CMD_DO_REPEAT_SERVO and MAV_CMD_DO_REPEAT_RELAY
{
	if(event_repeat == 0 || (millis() - event_timer) < event_delay)
		return;

	if (event_repeat > 0){
		event_repeat --;
	}

	if(event_repeat != 0) {		// event_repeat = -1 means repeat forever
		event_timer = millis();

		if (event_id >= CH_5 && event_id <= CH_8) {
			if(event_repeat%2) {
				hal.rcout->write(event_id, event_value); // send to Servos
			} else {
				hal.rcout->write(event_id, event_undo_value);
			}
		}

		if  (event_id == RELAY_TOGGLE) {
			relay.toggle();
		}
	}
}
#line 1 "./Firmware/APMRover2/failsafe.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  failsafe support
  Andrew Tridgell, December 2011
 */

/*
  our failsafe strategy is to detect main loop lockup and switch to
  passing inputs straight from the RC inputs to RC outputs.
 */

/*
  this failsafe_check function is called from the core timer interrupt
  at 1kHz.
 */
void failsafe_check(uint32_t tnow)
{
    static uint16_t last_mainLoop_count;
    static uint32_t last_timestamp;
    static bool in_failsafe;

    if (mainLoop_count != last_mainLoop_count) {
        // the main loop is running, all is OK
        last_mainLoop_count = mainLoop_count;
        last_timestamp = tnow;
        in_failsafe = false;
        return;
    }

    if (tnow - last_timestamp > 200000) {
        // we have gone at least 0.2 seconds since the main loop
        // ran. That means we're in trouble, or perhaps are in 
        // an initialisation routine or log erase. Start passing RC
        // inputs through to outputs
        in_failsafe = true;
    }

    if (in_failsafe && tnow - last_timestamp > 20000) {
        // pass RC inputs to outputs every 20ms        
        last_timestamp = tnow;
        hal.rcin->clear_overrides();
        uint8_t start_ch = 0;
        if (demoing_servos) {
            start_ch = 1;
        }
        for (uint8_t ch=start_ch; ch<4; ch++) {
            hal.rcout->write(ch, hal.rcin->read(ch));
        }
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_manual, true);
    }
}
#line 1 "./Firmware/APMRover2/navigation.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void navigate()
{
	// do not navigate with corrupt data
	// ---------------------------------
	if (!have_position) {
		return;
	}

	if ((next_WP.lat == 0)||(home_is_set==false)){
		return;
	}

	// waypoint distance from plane
	// ----------------------------
	wp_distance = get_distance(&current_loc, &next_WP);

	if (wp_distance < 0){
		gcs_send_text_P(SEVERITY_HIGH,PSTR("<navigate> WP error - distance < 0"));
		return;
	}

	// target_bearing is where we should be heading
	// --------------------------------------------
	target_bearing 	= get_bearing_cd(&current_loc, &next_WP);

	// nav_bearing will includes xtrac correction
	// ------------------------------------------
	nav_bearing = target_bearing;

	// control mode specific updates to nav_bearing
	// --------------------------------------------
	update_navigation();
}


static void calc_bearing_error()
{    
    static butter10hz1_6 butter;

	bearing_error_cd = wrap_180(nav_bearing - ahrs.yaw_sensor);
    bearing_error_cd = butter.filter(bearing_error_cd);
}

static long wrap_360(long error)
{
	if (error > 36000)	error -= 36000;
	if (error < 0)		error += 36000;
	return error;
}

static long wrap_180(long error)
{
	if (error > 18000)	error -= 36000;
	if (error < -18000)	error += 36000;
	return error;
}

static void update_crosstrack(void)
{
	// Crosstrack Error
	// ----------------
	if (abs(wrap_180(target_bearing - crosstrack_bearing)) < 4500) {	 // If we are too far off or too close we don't do track following
		crosstrack_error = sinf(radians((target_bearing - crosstrack_bearing) / (float)100)) * wp_distance;	 // Meters we are off track line
		nav_bearing += constrain(crosstrack_error * g.crosstrack_gain, -g.crosstrack_entry_angle.get(), g.crosstrack_entry_angle.get());
		nav_bearing = wrap_360(nav_bearing);
	}
}

static void reset_crosstrack()
{
	crosstrack_bearing 	= get_bearing_cd(&prev_WP, &next_WP);	// Used for track following
}

void reached_waypoint()
{       

}

#line 1 "./Firmware/APMRover2/radio.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------
static uint8_t failsafeCounter = 0;		// we wait a second to take over the throttle and send the plane circling


static void init_rc_in()
{
	// set rc channel ranges
	g.channel_steer.set_angle(SERVO_MAX);
	g.channel_throttle.set_angle(100);

	// set rc dead zones
	g.channel_steer.set_dead_zone(60);
	g.channel_throttle.set_dead_zone(6);

	//set auxiliary ranges
	update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8);
}

static void init_rc_out()
{
    hal.rcout->enable_ch(CH_1);
    hal.rcout->enable_ch(CH_2);
    hal.rcout->enable_ch(CH_3);
    hal.rcout->enable_ch(CH_4);
    hal.rcout->enable_ch(CH_5);
    hal.rcout->enable_ch(CH_6);
    hal.rcout->enable_ch(CH_7);
    hal.rcout->enable_ch(CH_8);

#if HIL_MODE != HIL_MODE_ATTITUDE
	hal.rcout->write(CH_1, 	g.channel_steer.radio_trim);					// Initialization of servo outputs
	hal.rcout->write(CH_3, 	g.channel_throttle.radio_trim);

	hal.rcout->write(CH_5, 	g.rc_5.radio_trim);
	hal.rcout->write(CH_6, 	g.rc_6.radio_trim);
	hal.rcout->write(CH_7,   g.rc_7.radio_trim);
    hal.rcout->write(CH_8,   g.rc_8.radio_trim);
#else
	hal.rcout->write(CH_1, 	1500);					// Initialization of servo outputs
	hal.rcout->write(CH_2, 	1500);
	hal.rcout->write(CH_3, 	1000);
	hal.rcout->write(CH_4, 	1500);

	hal.rcout->write(CH_5, 	1500);
	hal.rcout->write(CH_6, 	1500);
	hal.rcout->write(CH_7,   1500);
    hal.rcout->write(CH_8,   2000);
#endif

}

static void read_radio()
{
    g.channel_steer.set_pwm(hal.rcin->read(CH_ROLL));

	g.channel_throttle.set_pwm(hal.rcin->read(CH_3));
  	g.rc_5.set_pwm(hal.rcin->read(CH_5));
 	g.rc_6.set_pwm(hal.rcin->read(CH_6));        
	g.rc_7.set_pwm(hal.rcin->read(CH_7));
	g.rc_8.set_pwm(hal.rcin->read(CH_8));

	control_failsafe(g.channel_throttle.radio_in);

	g.channel_throttle.servo_out = g.channel_throttle.control_in;

	if (g.channel_throttle.servo_out > 50) {
        throttle_nudge = (g.throttle_max - g.throttle_cruise) * ((g.channel_throttle.norm_input()-0.5) / 0.5);
	} else {
		throttle_nudge = 0;
	}

	/*
	cliSerial->printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d \n"),
				g.rc_1.control_in,
				g.rc_2.control_in,
				g.rc_3.control_in,
				g.rc_4.control_in);
	*/
}

static void control_failsafe(uint16_t pwm)
{
	if (!g.fs_throttle_enabled) {
        // no throttle failsafe
		return;
    }

	// Check for failsafe condition based on loss of GCS control
	if (rc_override_active) {
		if(millis() - rc_override_fs_timer > FAILSAFE_SHORT_TIME) {
			ch3_failsafe = true;
		} else {
			ch3_failsafe = false;
		}

	//Check for failsafe and debounce funky reads
	} else if (g.fs_throttle_enabled) {
		if (pwm < (unsigned)g.fs_throttle_value){
			// we detect a failsafe from radio
			// throttle has dropped below the mark
			failsafeCounter++;
			if (failsafeCounter == 9){
				gcs_send_text_fmt(PSTR("MSG FS ON %u"), (unsigned)pwm);
			}else if(failsafeCounter == 10) {
				ch3_failsafe = true;
			}else if (failsafeCounter > 10){
				failsafeCounter = 11;
			}

		}else if(failsafeCounter > 0){
			// we are no longer in failsafe condition
			// but we need to recover quickly
			failsafeCounter--;
			if (failsafeCounter > 3){
				failsafeCounter = 3;
			}
			if (failsafeCounter == 1){
				gcs_send_text_fmt(PSTR("MSG FS OFF %u"), (unsigned)pwm);
			}else if(failsafeCounter == 0) {
				ch3_failsafe = false;
			}else if (failsafeCounter <0){
				failsafeCounter = -1;
			}
		}
	}
}

static void trim_control_surfaces()
{
	read_radio();
	// Store control surface trim values
	// ---------------------------------
    if (g.channel_steer.radio_in > 1400) {
		g.channel_steer.radio_trim = g.channel_steer.radio_in;
        // save to eeprom
        g.channel_steer.save_eeprom();
    }
}

static void trim_radio()
{
	for (int y = 0; y < 30; y++) {
		read_radio();
	}
    trim_control_surfaces();
}
#line 1 "./Firmware/APMRover2/sensors.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if CONFIG_SONAR == ENABLED
static void init_sonar(void)
{
  /*
    #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
	    sonar.calculate_scaler(g.sonar_type, 3.3);
	#else
        sonar.calculate_scaler(g.sonar_type, 5.0);
	#endif
*/
}
#endif

#if LITE == DISABLED
// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

void ReadSCP1000(void) {}

#endif // HIL_MODE != HIL_MODE_ATTITUDE
#endif

static void read_battery(void)
{
	if(g.battery_monitoring == 0) {
		battery_voltage1 = 0;
		return;
	}
	
    if(g.battery_monitoring == 3 || g.battery_monitoring == 4) {
        // this copes with changing the pin at runtime
        batt_volt_pin->set_pin(g.battery_volt_pin);
        battery_voltage1 = BATTERY_VOLTAGE(batt_volt_pin->read_average());
    }
    if(g.battery_monitoring == 4) {
        // this copes with changing the pin at runtime
        batt_curr_pin->set_pin(g.battery_curr_pin);
        current_amps1    = CURRENT_AMPS(batt_curr_pin->read_average());
        current_total1   += current_amps1 * (float)delta_ms_medium_loop * 0.0002778;                                    // .0002778 is 1/3600 (conversion to hours)
    }
}

#line 1 "./Firmware/APMRover2/setup.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// Functions called from the setup menu
static int8_t	setup_radio			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_show			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_factory		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_flightmodes	(uint8_t argc, const Menu::arg *argv);
#if !defined( __AVR_ATmega1280__ )
static int8_t   setup_accel_scale                       (uint8_t argc, const Menu::arg *argv);
#endif
static int8_t	setup_erase			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_compass			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_declination		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_batt_monitor			(uint8_t argc, const Menu::arg *argv);

// Command/function table for the setup menu
static const struct Menu::command setup_menu_commands[] PROGMEM = {
	// command			function called
	// =======        	===============
	{"reset", 			setup_factory},
	{"radio",			setup_radio},
	{"modes",			setup_flightmodes},
#if !defined( __AVR_ATmega1280__ )
    {"accel",           setup_accel_scale},
#endif
	{"compass",			setup_compass},
	{"declination",		setup_declination},
	{"battery",			setup_batt_monitor},
	{"show",			setup_show},
	{"erase",			setup_erase},
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
static int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
	// Give the user some guidance
	cliSerial->printf_P(PSTR("Setup Mode\n"
						 "\n"
						 "IMPORTANT: if you have not previously set this system up, use the\n"
						 "'reset' command to initialize the EEPROM to sensible default values\n"
						 "and then the 'radio' command to configure for your radio.\n"
						 "\n"));

	// Run the setup menu.  When the menu exits, we will return to the main menu.
	setup_menu.run();
    return 0;
}

// Print the current configuration.
// Called by the setup menu 'show' command.
static int8_t
setup_show(uint8_t argc, const Menu::arg *argv)
{
	// clear the area
	print_blanks(8);

	report_radio();
	report_batt_monitor();
	report_gains();
	report_xtrack();
	report_throttle();
	report_modes();
	report_compass();

	cliSerial->printf_P(PSTR("Raw Values\n"));
	print_divider();

    AP_Param::show_all();

	return(0);
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{
	int			c;

	cliSerial->printf_P(PSTR("\nType 'Y' and hit Enter to perform factory reset, any other key to abort: "));

	do {
		c = cliSerial->read();
	} while (-1 == c);

	if (('y' != c) && ('Y' != c))
		return(-1);
	AP_Param::erase_all();
	cliSerial->printf_P(PSTR("\nFACTORY RESET complete - please reset APM to continue"));

	for (;;) {
	}
	// note, cannot actually return here
	return(0);
}


// Perform radio setup.
// Called by the setup menu 'radio' command.
static int8_t
setup_radio(uint8_t argc, const Menu::arg *argv)
{
	cliSerial->printf_P(PSTR("\n\nRadio Setup:\n"));
	uint8_t i;

	for(i = 0; i < 100;i++){
		delay(20);
		read_radio();
	}


	if(g.channel_steer.radio_in < 500){
		while(1){
			cliSerial->printf_P(PSTR("\nNo radio; Check connectors."));
			delay(1000);
			// stop here
		}
	}

	g.channel_steer.radio_min 		= g.channel_steer.radio_in;
	g.channel_throttle.radio_min 	= g.channel_throttle.radio_in;
	g.rc_2.radio_min = g.rc_2.radio_in;
	g.rc_4.radio_min = g.rc_4.radio_in;
	g.rc_5.radio_min = g.rc_5.radio_in;
	g.rc_6.radio_min = g.rc_6.radio_in;
	g.rc_7.radio_min = g.rc_7.radio_in;
	g.rc_8.radio_min = g.rc_8.radio_in;

	g.channel_steer.radio_max 		= g.channel_steer.radio_in;
	g.channel_throttle.radio_max 	= g.channel_throttle.radio_in;
	g.rc_2.radio_max = g.rc_2.radio_in;
	g.rc_4.radio_max = g.rc_4.radio_in;
	g.rc_5.radio_max = g.rc_5.radio_in;
	g.rc_6.radio_max = g.rc_6.radio_in;
	g.rc_7.radio_max = g.rc_7.radio_in;
	g.rc_8.radio_max = g.rc_8.radio_in;

	g.channel_steer.radio_trim 		= g.channel_steer.radio_in;
	g.rc_2.radio_trim = 1500;
	g.rc_4.radio_trim = 1500;
	g.rc_5.radio_trim = 1500;
	g.rc_6.radio_trim = 1500;
	g.rc_7.radio_trim = 1500;
	g.rc_8.radio_trim = 1500;

	cliSerial->printf_P(PSTR("\nMove all controls to each extreme. Hit Enter to save: \n"));
	while(1){

		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();

		g.channel_steer.update_min_max();
		g.channel_throttle.update_min_max();
		g.rc_2.update_min_max();
		g.rc_4.update_min_max();
		g.rc_5.update_min_max();
		g.rc_6.update_min_max();
		g.rc_7.update_min_max();
		g.rc_8.update_min_max();

		if(cliSerial->available() > 0){
            while (cliSerial->available() > 0) {
                cliSerial->read();
            }
			g.channel_steer.save_eeprom();
			g.channel_throttle.save_eeprom();
			g.rc_2.save_eeprom();
			g.rc_4.save_eeprom();
			g.rc_5.save_eeprom();
			g.rc_6.save_eeprom();
			g.rc_7.save_eeprom();
			g.rc_8.save_eeprom();
			print_done();
			break;
		}
	}
	trim_radio();
	report_radio();
	return(0);
}


static int8_t
setup_flightmodes(uint8_t argc, const Menu::arg *argv)
{
	uint8_t switchPosition, mode = 0;

	cliSerial->printf_P(PSTR("\nMove RC toggle switch to each position to edit, move aileron stick to select modes."));
	print_hit_enter();
	trim_radio();

	while(1){
		delay(20);
		read_radio();
		switchPosition = readSwitch();


		// look for control switch change
		if (oldSwitchPosition != switchPosition){
			// force position 5 to MANUAL
			if (switchPosition > 4) {
				modes[switchPosition] = MANUAL;
			}
			// update our current mode
			mode = modes[switchPosition];

			// update the user
			print_switch(switchPosition, mode);

			// Remember switch position
			oldSwitchPosition = switchPosition;
		}

		// look for stick input
		int radioInputSwitch = radio_input_switch();

		if (radioInputSwitch != 0){

			mode += radioInputSwitch;

			while (
				mode != MANUAL &&
				mode != LEARNING &&
				mode != AUTO &&
				mode != RTL) 
			{
				if (mode < MANUAL)
					mode = RTL;
				else if (mode > RTL)
					mode = MANUAL;
				else
					mode += radioInputSwitch;
			}

			// Override position 5
			if(switchPosition > 4)
				mode = MANUAL;

			// save new mode
			modes[switchPosition] = mode;

			// print new mode
			print_switch(switchPosition, mode);
		}

		// escape hatch
		if(cliSerial->available() > 0){
		    // save changes
            for (mode=0; mode<6; mode++)
                modes[mode].save();
			report_modes();
			print_done();
			return (0);
		}
	}
}

static int8_t
setup_declination(uint8_t argc, const Menu::arg *argv)
{
	compass.set_declination(radians(argv[1].f));
	report_compass();
    return 0;
}


static int8_t
setup_erase(uint8_t argc, const Menu::arg *argv)
{
	int			c;

	cliSerial->printf_P(PSTR("\nType 'Y' and hit Enter to erase all waypoint and parameter data, any other key to abort: "));

	do {
		c = cliSerial->read();
	} while (-1 == c);

	if (('y' != c) && ('Y' != c))
		return(-1);
	zero_eeprom();
	return 0;
}

/*
  handle full accelerometer calibration via user dialog
 */
#if !defined( __AVR_ATmega1280__ )
static int8_t
setup_accel_scale(uint8_t argc, const Menu::arg *argv)
{
    float trim_roll, trim_pitch;
    cliSerial->println_P(PSTR("Initialising gyros"));
    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate,
             flash_leds);
    AP_InertialSensor_UserInteractStream interact(hal.console);
    if(ins.calibrate_accel(flash_leds, &interact, trim_roll, trim_pitch)) {
        // reset ahrs's trim to suggested values from calibration routine
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
    return(0);
}
#endif

static int8_t
setup_compass(uint8_t argc, const Menu::arg *argv)
{
	if (!strcmp_P(argv[1].str, PSTR("on"))) {
        compass.set_orientation(MAG_ORIENTATION);	// set compass's orientation on aircraft
		if (!compass.init()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            g.compass_enabled = true;
        }
	} else if (!strcmp_P(argv[1].str, PSTR("off"))) {
		g.compass_enabled = false;

	} else if (!strcmp_P(argv[1].str, PSTR("reset"))) {
		compass.set_offsets(0,0,0);

	} else {
		cliSerial->printf_P(PSTR("\nOptions:[on,off,reset]\n"));
		report_compass();
		return 0;
	}

	g.compass_enabled.save();
	report_compass();
	return 0;
}

static int8_t
setup_batt_monitor(uint8_t argc, const Menu::arg *argv)
{
	if(argv[1].i >= 0 && argv[1].i <= 4){
		g.battery_monitoring.set_and_save(argv[1].i);

	} else {
		cliSerial->printf_P(PSTR("\nOptions: 3-4"));
	}

	report_batt_monitor();
	return 0;
}

/***************************************************************************/
// CLI reports
/***************************************************************************/

static void report_batt_monitor()
{
	//print_blanks(2);
	cliSerial->printf_P(PSTR("Batt Mointor\n"));
	print_divider();
	if(g.battery_monitoring == 0)	cliSerial->printf_P(PSTR("Batt monitoring disabled"));
	if(g.battery_monitoring == 3)	cliSerial->printf_P(PSTR("Monitoring batt volts"));
	if(g.battery_monitoring == 4)	cliSerial->printf_P(PSTR("Monitoring volts and current"));
	print_blanks(2);
}
static void report_radio()
{
	//print_blanks(2);
	cliSerial->printf_P(PSTR("Radio\n"));
	print_divider();
	// radio
	print_radio_values();
	print_blanks(2);
}

static void report_gains()
{
	//print_blanks(2);
	cliSerial->printf_P(PSTR("Gains\n"));
	print_divider();

	cliSerial->printf_P(PSTR("servo steer:\n"));
	print_PID(&g.pidServoSteer);

	cliSerial->printf_P(PSTR("nav steer:\n"));
	print_PID(&g.pidNavSteer);

	cliSerial->printf_P(PSTR("speed throttle:\n"));
	print_PID(&g.pidSpeedThrottle);

	print_blanks(2);
}

static void report_xtrack()
{
	//print_blanks(2);
	cliSerial->printf_P(PSTR("Crosstrack\n"));
	print_divider();
	// radio
	cliSerial->printf_P(PSTR("XTRACK: %4.2f\n"
						 "XTRACK angle: %d\n"),
						 (float)g.crosstrack_gain,
						 (int)g.crosstrack_entry_angle);
	print_blanks(2);
}

static void report_throttle()
{
	//print_blanks(2);
	cliSerial->printf_P(PSTR("Throttle\n"));
	print_divider();

	cliSerial->printf_P(PSTR("min: %u\n"
                             "max: %u\n"
                             "cruise: %u\n"
                             "failsafe_enabled: %u\n"
                             "failsafe_value: %u\n"),
						 (unsigned)g.throttle_min,
						 (unsigned)g.throttle_max,
						 (unsigned)g.throttle_cruise,
						 (unsigned)g.fs_throttle_enabled,
						 (unsigned)g.fs_throttle_value);
	print_blanks(2);
}

static void report_compass()
{
	//print_blanks(2);
	cliSerial->printf_P(PSTR("Compass: "));

    switch (compass.product_id) {
    case AP_COMPASS_TYPE_HMC5883L:
        cliSerial->println_P(PSTR("HMC5883L"));
        break;
    case AP_COMPASS_TYPE_HMC5843:
        cliSerial->println_P(PSTR("HMC5843"));
        break;
    case AP_COMPASS_TYPE_HIL:
        cliSerial->println_P(PSTR("HIL"));
        break;
    default:
        cliSerial->println_P(PSTR("??"));
        break;
    }

	print_divider();

	print_enabled(g.compass_enabled);

	// mag declination
	cliSerial->printf_P(PSTR("Mag Declination: %4.4f\n"),
							degrees(compass.get_declination()));

	Vector3f offsets = compass.get_offsets();

	// mag offsets
	cliSerial->printf_P(PSTR("Mag offsets: %4.4f, %4.4f, %4.4f\n"),
							offsets.x,
							offsets.y,
							offsets.z);
	print_blanks(2);
}

static void report_modes()
{
	//print_blanks(2);
	cliSerial->printf_P(PSTR("Flight modes\n"));
	print_divider();

	for(int i = 0; i < 6; i++ ){
		print_switch(i, modes[i]);
	}
	print_blanks(2);
}

/***************************************************************************/
// CLI utilities
/***************************************************************************/

static void
print_PID(PID * pid)
{
	cliSerial->printf_P(PSTR("P: %4.3f, I:%4.3f, D:%4.3f, IMAX:%ld\n"),
					pid->kP(),
					pid->kI(),
					pid->kD(),
					(long)pid->imax());
}

static void
print_radio_values()
{
	cliSerial->printf_P(PSTR("CH1: %d | %d | %d\n"), (int)g.channel_steer.radio_min, (int)g.channel_steer.radio_trim, (int)g.channel_steer.radio_max);
	cliSerial->printf_P(PSTR("CH2: %d | %d | %d\n"), (int)g.rc_2.radio_min, (int)g.rc_2.radio_trim, (int)g.rc_2.radio_max);
	cliSerial->printf_P(PSTR("CH3: %d | %d | %d\n"), (int)g.channel_throttle.radio_min, (int)g.channel_throttle.radio_trim, (int)g.channel_throttle.radio_max);
	cliSerial->printf_P(PSTR("CH4: %d | %d | %d\n"), (int)g.rc_4.radio_min, (int)g.rc_4.radio_trim, (int)g.rc_4.radio_max);
	cliSerial->printf_P(PSTR("CH5: %d | %d | %d\n"), (int)g.rc_5.radio_min, (int)g.rc_5.radio_trim, (int)g.rc_5.radio_max);
	cliSerial->printf_P(PSTR("CH6: %d | %d | %d\n"), (int)g.rc_6.radio_min, (int)g.rc_6.radio_trim, (int)g.rc_6.radio_max);
	cliSerial->printf_P(PSTR("CH7: %d | %d | %d\n"), (int)g.rc_7.radio_min, (int)g.rc_7.radio_trim, (int)g.rc_7.radio_max);
	cliSerial->printf_P(PSTR("CH8: %d | %d | %d\n"), (int)g.rc_8.radio_min, (int)g.rc_8.radio_trim, (int)g.rc_8.radio_max);

}

static void
print_switch(uint8_t p, uint8_t m)
{
	cliSerial->printf_P(PSTR("Pos %d: "),p);
    print_mode(m);
}

static void
print_done()
{
	cliSerial->printf_P(PSTR("\nSaved Settings\n\n"));
}

static void
print_blanks(int num)
{
	while(num > 0){
		num--;
		cliSerial->println("");
	}
}

static void
print_divider(void)
{
	for (int i = 0; i < 40; i++) {
		cliSerial->printf_P(PSTR("-"));
	}
	cliSerial->println("");
}

static int8_t
radio_input_switch(void)
{
	static int8_t bouncer = 0;


	if (int16_t(g.channel_steer.radio_in - g.channel_steer.radio_trim) > 100) {
	    bouncer = 10;
	}
	if (int16_t(g.channel_steer.radio_in - g.channel_steer.radio_trim) < -100) {
	    bouncer = -10;
	}
	if (bouncer >0) {
	    bouncer --;
	}
	if (bouncer <0) {
	    bouncer ++;
	}

	if (bouncer == 1 || bouncer == -1) {
	    return bouncer;
	} else {
	    return 0;
	}
}


static void zero_eeprom(void)
{
	uint8_t b = 0;
	cliSerial->printf_P(PSTR("\nErasing EEPROM\n"));
	for (uint16_t i = 0; i < EEPROM_MAX_ADDR; i++) {
		hal.storage->write_byte(i, b);
	}
	cliSerial->printf_P(PSTR("done\n"));
}

static void print_enabled(bool b)
{
	if(b)
		cliSerial->printf_P(PSTR("en"));
	else
		cliSerial->printf_P(PSTR("dis"));
	cliSerial->printf_P(PSTR("abled\n"));
}

#endif // CLI_ENABLED
#line 1 "./Firmware/APMRover2/system.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
	We will determine later if we are actually on the ground and process a
	ground start in that case.

*****************************************************************************/

#if CLI_ENABLED == ENABLED

// Functions called from the top-level menu
#if LITE == DISABLED
static int8_t	process_logs(uint8_t argc, const Menu::arg *argv);	// in Log.pde
#endif
static int8_t	setup_mode(uint8_t argc, const Menu::arg *argv);	// in setup.pde
static int8_t	test_mode(uint8_t argc, const Menu::arg *argv);		// in test.cpp

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
static int8_t	main_menu_help(uint8_t argc, const Menu::arg *argv)
{
	cliSerial->printf_P(PSTR("Commands:\n"
						 "  logs        log readback/setup mode\n"
						 "  setup       setup mode\n"
						 "  test        test mode\n"
						 "\n"
						 "Move the slide switch and reset to FLY.\n"
						 "\n"));
	return(0);
}

// Command/function table for the top-level menu.
static const struct Menu::command main_menu_commands[] PROGMEM = {
//   command		function called
//   =======        ===============
#if LITE == DISABLED
	{"logs",		process_logs},
#endif
	{"setup",		setup_mode},
	{"test",		test_mode},
	{"help",		main_menu_help}
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

// the user wants the CLI. It never exits
static void run_cli(AP_HAL::UARTDriver *port)
{
    // disable the failsafe code in the CLI
    hal.scheduler->register_timer_failsafe(NULL,1);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);

    while (1) {
        main_menu.run();
    }
}

#endif // CLI_ENABLED

static void init_ardupilot()
{
#if USB_MUX_PIN > 0
    // on the APM2 board we have a mux thet switches UART0 between
    // USB and the board header. If the right ArduPPM firmware is
    // installed we can detect if USB is connected using the
    // USB_MUX_PIN
    pinMode(USB_MUX_PIN, INPUT);

    usb_connected = !digitalRead(USB_MUX_PIN);
    if (!usb_connected) {
        // USB is not connected, this means UART0 may be a Xbee, with
        // its darned bricking problem. We can't write to it for at
        // least one second after powering up. Simplest solution for
        // now is to delay for 1 second. Something more elegant may be
        // added later
        delay(1000);
    }
#endif

	// Console serial port
	//
	// The console port buffers are defined to be sufficiently large to support
	// the console's use as a logging device, optionally as the GPS port when
	// GPS_PROTOCOL_IMU is selected, and as the telemetry port.
	//
	// XXX This could be optimised to reduce the buffer sizes in the cases
	// where they are not otherwise required.
	//
    hal.uartA->begin(SERIAL0_BAUD, 128, 128);

	// GPS serial port.
	//
	// XXX currently the EM406 (SiRF receiver) is nominally configured
	// at 57600, however it's not been supported to date.  We should
	// probably standardise on 38400.
	//
	// XXX the 128 byte receive buffer may be too small for NMEA, depending
	// on the message set configured.
	//
    // standard gps running
    hal.uartB->begin(115200, 128, 16);

	cliSerial->printf_P(PSTR("\n\nInit " THISFIRMWARE
						 "\n\nFree RAM: %u\n"),
                    memcheck_available_memory());
                    
	//
	// Check the EEPROM format version before loading any parameters from EEPROM.
	//
	
    load_parameters();

    // after parameter load setup correct baud rate on uartA
    hal.uartA->begin(map_baudrate(g.serial0_baud, SERIAL0_BAUD));

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

	// init the GCS
	gcs0.init(hal.uartA);

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

#if USB_MUX_PIN > 0
    if (!usb_connected) {
        // we are not connected via USB, re-init UART0 with right
        // baud rate
        hal.uartA->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD));
    }
#else
    // we have a 2nd serial port for telemetry
    hal.uartC->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
	gcs3.init(hal.uartC);
#endif

	mavlink_system.sysid = g.sysid_this_mav;

#if LITE == DISABLED
#if LOGGING_ENABLED == ENABLED
	DataFlash.Init(); 	// DataFlash log initialization
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash card inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
		do_erase_logs();
    }
	if (g.log_bitmask != 0) {
		DataFlash.start_new_log();
	}
#endif
#endif

#if HIL_MODE != HIL_MODE_ATTITUDE

#if CONFIG_ADC == ENABLED
    adc.Init();      // APM ADC library initialization
#endif

#if LITE == DISABLED
	if (g.compass_enabled==true) {
        compass.set_orientation(MAG_ORIENTATION);							// set compass's orientation on aircraft
		if (!compass.init()|| !compass.read()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
            //compass.get_offsets();						// load offsets to account for airframe magnetic interference
        }
	}
#else
  I2c.begin();
  I2c.timeOut(20);

  // I2c.setSpeed(true);

  if (!compass.init()) {
	  cliSerial->println("compass initialisation failed!");
	  while (1) ;
  }

  compass.set_orientation(MAG_ORIENTATION);  // set compass's orientation on aircraft.
  compass.set_offsets(0,0,0);  // set offsets to account for surrounding interference
  compass.set_declination(ToRad(0.0));  // set local difference between magnetic north and true north

  cliSerial->print("Compass auto-detected as: ");
  switch( compass.product_id ) {
      case AP_COMPASS_TYPE_HIL:
	      cliSerial->println("HIL");
		  break;
      case AP_COMPASS_TYPE_HMC5843:
	      cliSerial->println("HMC5843");
		  break;
      case AP_COMPASS_TYPE_HMC5883L:
	      cliSerial->println("HMC5883L");
		  break;
      default:
	      cliSerial->println("unknown");
		  break;
  }
  
  delay(3000);

#endif
	// initialise sonar
	#if CONFIG_SONAR == ENABLED
	init_sonar();
	#endif

#endif
	// Do GPS init
	g_gps = &g_gps_driver;
    // GPS initialisation
	g_gps->init(hal.uartB, GPS::GPS_ENGINE_AUTOMOTIVE);

	//mavlink_system.sysid = MAV_SYSTEM_ID;				// Using g.sysid_this_mav
	mavlink_system.compid = 1;	//MAV_COMP_ID_IMU;   // We do not check for comp id
	mavlink_system.type = MAV_TYPE_GROUND_ROVER;

    rc_override_active = hal.rcin->set_overrides(rc_override, 8);

	init_rc_in();		// sets up rc channels from radio
	init_rc_out();		// sets up the timer libs

	pinMode(C_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(A_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(B_LED_PIN, OUTPUT);			// GPS status LED
#if SLIDE_SWITCH_PIN > 0
	pinMode(SLIDE_SWITCH_PIN, INPUT);	// To enter interactive mode
#endif
#if CONFIG_PUSHBUTTON == ENABLED
	pinMode(PUSHBUTTON_PIN, INPUT);		// unused
#endif
#if CONFIG_RELAY == ENABLED
    relay.init();
#endif

    /*
      setup the 'main loop is dead' check. Note that this relies on
      the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

	// If the switch is in 'menu' mode, run the main menu.
	//
	// Since we can't be sure that the setup or test mode won't leave
	// the system in an odd state, we don't let the user exit the top
	// menu; they must reset in order to fly.
	//
#if CLI_ENABLED == ENABLED && CLI_SLIDER_ENABLED == ENABLED
	if (digitalRead(SLIDE_SWITCH_PIN) == 0) {
		digitalWrite(A_LED_PIN,LED_ON);		// turn on setup-mode LED
		cliSerial->printf_P(PSTR("\n"
							 "Entering interactive setup mode...\n"
							 "\n"
							 "If using the Arduino Serial Monitor, ensure Line Ending is set to Carriage Return.\n"
							 "Type 'help' to list commands, 'exit' to leave a submenu.\n"
							 "Visit the 'setup' menu for first-time configuration.\n"));
        cliSerial->println_P(PSTR("\nMove the slide switch and reset to FLY.\n"));
        run_cli(&cliSerial);
	}
#else
    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
#if USB_MUX_PIN == 0
    hal.uartC->println_P(msg);
#endif
#endif // CLI_ENABLED

	startup_ground();

#if LITE == DISABLED
	if (g.log_bitmask & MASK_LOG_CMD)
			Log_Write_Startup(TYPE_GROUNDSTART_MSG);
#endif

    set_mode(MANUAL);

	// set the correct flight mode
	// ---------------------------
	reset_control_switch();
}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
static void startup_ground(void)
{
    set_mode(INITIALISING);

	gcs_send_text_P(SEVERITY_LOW,PSTR("<startup_ground> GROUND START"));

	#if(GROUND_START_DELAY > 0)
		gcs_send_text_P(SEVERITY_LOW,PSTR("<startup_ground> With Delay"));
		delay(GROUND_START_DELAY * 1000);
	#endif

	// Makes the servos wiggle
	// step 1 = 1 wiggle
	// -----------------------
	demo_servos(1);

#if LITE == DISABLED
	//IMU ground start
	//------------------------
    //

	startup_INS_ground(false);
#endif

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	// initialize commands
	// -------------------
	init_commands();

	// Makes the servos wiggle - 3 times signals ready to fly
	// -----------------------
	demo_servos(3);

	gcs_send_text_P(SEVERITY_LOW,PSTR("\n\n Ready to drive."));
}

static void set_mode(enum mode mode)
{       

	if(control_mode == mode){
		// don't switch modes if we are already in the correct mode.
		return;
	}
	control_mode = mode;
    throttle_last = 0;
    throttle = 500;
        
	switch(control_mode)
	{
		case MANUAL:
		case LEARNING:
			break;

		case AUTO:
            rtl_complete = false;
            restart_nav();
			break;

		case RTL:
			do_RTL();
			break;

		default:
			do_RTL();
			break;
	}

#if LITE == DISABLED
	if (g.log_bitmask & MASK_LOG_MODE)
		Log_Write_Mode(control_mode);
#endif

}

static void check_long_failsafe()
{
	// only act on changes
	// -------------------
	if(failsafe != FAILSAFE_LONG  && failsafe != FAILSAFE_GCS){
		if(rc_override_active && millis() - rc_override_fs_timer > FAILSAFE_LONG_TIME) {
			failsafe_long_on_event(FAILSAFE_LONG);
		}
		if(! rc_override_active && failsafe == FAILSAFE_SHORT && millis() - ch3_failsafe_timer > FAILSAFE_LONG_TIME) {
			failsafe_long_on_event(FAILSAFE_LONG);
		}
		if (g.fs_gcs_enabled && millis() - rc_override_fs_timer > FAILSAFE_LONG_TIME) {
			failsafe_long_on_event(FAILSAFE_GCS);
		}
	} else {
		// We do not change state but allow for user to change mode
		if(failsafe == FAILSAFE_GCS && millis() - rc_override_fs_timer < FAILSAFE_SHORT_TIME) failsafe = FAILSAFE_NONE;
		if(failsafe == FAILSAFE_LONG && rc_override_active && millis() - rc_override_fs_timer < FAILSAFE_SHORT_TIME) failsafe = FAILSAFE_NONE;
		if(failsafe == FAILSAFE_LONG && !rc_override_active && !ch3_failsafe) failsafe = FAILSAFE_NONE;
	}
}

#if LITE == DISABLED
static void startup_INS_ground(bool force_accel_level)
{
#if HIL_MODE != HIL_MODE_ATTITUDE
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Warming up ADC..."));
 	mavlink_delay(500);

	// Makes the servos wiggle twice - about to begin INS calibration - HOLD LEVEL AND STILL!!
	// -----------------------
	demo_servos(2);
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Beginning INS calibration; do not move plane"));
	mavlink_delay(1000);

    ahrs.init();
	ahrs.set_fly_forward(true);
	ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate, 
             flash_leds);
    if (force_accel_level) {
        // when MANUAL_LEVEL is set to 1 we don't do accelerometer
        // levelling on each boot, and instead rely on the user to do
        // it once via the ground station	
        ins.init_accel(flash_leds);
        ahrs.set_trim(Vector3f(0, 0, 0));
	}
    ahrs.reset();

#endif // HIL_MODE_ATTITUDE

	digitalWrite(B_LED_PIN, LED_ON);		// Set LED B high to indicate INS ready
	digitalWrite(A_LED_PIN, LED_OFF);
	digitalWrite(C_LED_PIN, LED_OFF);
}
#endif

static void update_GPS_light(void)
{
	// GPS LED on if we have a fix or Blink GPS LED if we are receiving data
	// ---------------------------------------------------------------------
	switch (g_gps->status()) {
		case(2):
			digitalWrite(C_LED_PIN, LED_ON);  //Turn LED C on when gps has valid fix.
			break;

		case(1):
			if (g_gps->valid_read == true){
				GPS_light = !GPS_light; // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
				if (GPS_light){
					digitalWrite(C_LED_PIN, LED_OFF);
				} else {
					digitalWrite(C_LED_PIN, LED_ON);
				}
				g_gps->valid_read = false;
			}
			break;

		default:
			digitalWrite(C_LED_PIN, LED_OFF);
			break;
	}
}


static void resetPerfData(void) {
	mainLoop_count 			= 0;
	G_Dt_max 				= 0;
	ahrs.renorm_range_count 	= 0;
	ahrs.renorm_blowup_count = 0;
	gps_fix_count 			= 0;
	pmTest1					= 0;
	perf_mon_timer 			= millis();
}


/*
  map from a 8 bit EEPROM baud rate to a real baud rate
 */
static uint32_t map_baudrate(int8_t rate, uint32_t default_baud)
{
    switch (rate) {
    case 1:    return 1200;
    case 2:    return 2400;
    case 4:    return 4800;
    case 9:    return 9600;
    case 19:   return 19200;
    case 38:   return 38400;
    case 57:   return 57600;
    case 111:  return 111100;
    case 115:  return 115200;
    }
    cliSerial->println_P(PSTR("Invalid SERIAL3_BAUD"));
    return default_baud;
}


#if USB_MUX_PIN > 0
static void check_usb_mux(void)
{
    bool usb_check = !digitalRead(USB_MUX_PIN);
    if (usb_check == usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    usb_connected = usb_check;
    if (usb_connected) {
        hal.uartA->begin(SERIAL0_BAUD, 128, 128);
    } else {
        hal.uartA->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
    }
}
#endif


/*
  called by gyro/accel init to flash LEDs so user
  has some mesmerising lights to watch while waiting
 */
void flash_leds(bool on)
{
    digitalWrite(A_LED_PIN, on?LED_OFF:LED_ON);
    digitalWrite(C_LED_PIN, on?LED_ON:LED_OFF);
}

/*
 * Read Vcc vs 1.1v internal reference
 */
uint16_t board_voltage(void)
{
    return vcc_pin->read_latest();
}

static void
print_mode(uint8_t mode)
{
    switch (mode) {
    case MANUAL:
        cliSerial->println_P(PSTR("Manual"));
        break;
    case LEARNING:
        cliSerial->println_P(PSTR("Learning"));
        break;
    case AUTO:
        cliSerial->println_P(PSTR("AUTO"));
        break;
    case RTL:
        cliSerial->println_P(PSTR("RTL"));
        break;
    default:
        cliSerial->println_P(PSTR("---"));
        break;
    }
}

/*
  force a software reset of the APM
 */
static void reboot_apm(void)
{
    hal.scheduler->reboot();
    while (1);
}
#line 1 "./Firmware/APMRover2/test.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t	test_radio_pwm(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_radio(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_passthru(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_failsafe(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_gps(uint8_t argc, 			const Menu::arg *argv);
#if CONFIG_ADC == ENABLED
static int8_t	test_adc(uint8_t argc, 			const Menu::arg *argv);
#endif
static int8_t	test_ins(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_battery(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_relay(uint8_t argc,	 	const Menu::arg *argv);
static int8_t	test_wp(uint8_t argc, 			const Menu::arg *argv);
#if CONFIG_SONAR == ENABLED
static int8_t	test_sonar(uint8_t argc, 	const Menu::arg *argv);
#endif
static int8_t	test_mag(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_modeswitch(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_logging(uint8_t argc, 		const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
static const struct Menu::command test_menu_commands[] PROGMEM = {
	{"pwm",				test_radio_pwm},
	{"radio",			test_radio},
	{"passthru",		test_passthru},
	{"failsafe",		test_failsafe},
	{"battery",	test_battery},
	{"relay",			test_relay},
	{"waypoints",		test_wp},
	{"modeswitch",		test_modeswitch},

	// Tests below here are for hardware sensors only present
	// when real sensors are attached or they are emulated
#if HIL_MODE == HIL_MODE_DISABLED
#if CONFIG_ADC == ENABLED
	{"adc", 		test_adc},
#endif
	{"gps",			test_gps},
	{"ins",			test_ins},
#if CONFIG_SONAR == ENABLED
	{"sonartest",	test_sonar},
#endif
	{"compass",		test_mag},
#elif HIL_MODE == HIL_MODE_SENSORS
	{"adc", 		test_adc},
	{"gps",			test_gps},
	{"ins",			test_ins},
	{"compass",		test_mag},
#elif HIL_MODE == HIL_MODE_ATTITUDE
#endif
	{"logging",		test_logging},

};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

static int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
	cliSerial->printf_P(PSTR("Test Mode\n\n"));
	test_menu.run();
    return 0;
}

static void print_hit_enter()
{
	cliSerial->printf_P(PSTR("Hit Enter to exit.\n\n"));
}

static int8_t
test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(20);

		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();

		cliSerial->printf_P(PSTR("IN:\t1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
							g.channel_steer.radio_in,
							g.rc_2.radio_in,
							g.channel_throttle.radio_in,
							g.rc_4.radio_in,
							g.rc_5.radio_in,
							g.rc_6.radio_in,
							g.rc_7.radio_in,
							g.rc_8.radio_in);

		if(cliSerial->available() > 0){
			return (0);
		}
	}
}


static int8_t
test_passthru(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(20);

        // New radio frame? (we could use also if((millis()- timer) > 20)
        if (hal.rcin->valid() > 0) {
            cliSerial->print("CH:");
            for(int i = 0; i < 8; i++){
                cliSerial->print(hal.rcin->read(i));	// Print channel values
                cliSerial->print(",");
                hal.rcout->write(i, hal.rcin->read(i)); // Copy input to Servos
            }
            cliSerial->println();
        }
        if (cliSerial->available() > 0){
            return (0);
        }
    }
    return 0;
}

static int8_t
test_radio(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	while(1){
		delay(20);
		read_radio();

		g.channel_steer.calc_pwm();
		g.channel_throttle.calc_pwm();

		// write out the servo PWM values
		// ------------------------------
		set_servos();

        tuning_value = constrain(((float)(g.rc_7.radio_in - g.rc_7.radio_min) / (float)(g.rc_7.radio_max - g.rc_7.radio_min)),0,1);
                
		cliSerial->printf_P(PSTR("IN 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d  Tuning = %2.3f\n"),
							g.channel_steer.control_in,
							g.rc_2.control_in,
							g.channel_throttle.control_in,
							g.rc_4.control_in,
							g.rc_5.control_in,
							g.rc_6.control_in,
							g.rc_7.control_in,
							g.rc_8.control_in,
                                                        tuning_value);

		if(cliSerial->available() > 0){
			return (0);
		}
	}
}

static int8_t
test_failsafe(uint8_t argc, const Menu::arg *argv)
{
	uint8_t fail_test;
	print_hit_enter();
	for(int i = 0; i < 50; i++){
		delay(20);
		read_radio();
	}

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	oldSwitchPosition = readSwitch();

	cliSerial->printf_P(PSTR("Unplug battery, throttle in neutral, turn off radio.\n"));
	while(g.channel_throttle.control_in > 0){
		delay(20);
		read_radio();
	}

	while(1){
		delay(20);
		read_radio();

		if(g.channel_throttle.control_in > 0){
			cliSerial->printf_P(PSTR("THROTTLE CHANGED %d \n"), g.channel_throttle.control_in);
			fail_test++;
		}

		if (oldSwitchPosition != readSwitch()){
			cliSerial->printf_P(PSTR("CONTROL MODE CHANGED: "));
            print_mode(readSwitch());
			fail_test++;
		}

		if (g.fs_throttle_enabled && g.channel_throttle.get_failsafe()){
			cliSerial->printf_P(PSTR("THROTTLE FAILSAFE ACTIVATED: %d, "), g.channel_throttle.radio_in);
            print_mode(readSwitch());
			fail_test++;
		}

		if(fail_test > 0){
			return (0);
		}
		if(cliSerial->available() > 0){
			cliSerial->printf_P(PSTR("LOS caused no change in APM.\n"));
			return (0);
		}
	}
}

static int8_t
test_battery(uint8_t argc, const Menu::arg *argv)
{
if (g.battery_monitoring == 3 || g.battery_monitoring == 4) {
	print_hit_enter();
	delta_ms_medium_loop = 100;

	while(1){
		delay(100);
		read_radio();
		read_battery();
		if (g.battery_monitoring == 3){
			cliSerial->printf_P(PSTR("V: %4.4f\n"),
						battery_voltage1,
						current_amps1,
						current_total1);
		} else {
			cliSerial->printf_P(PSTR("V: %4.4f, A: %4.4f, mAh: %4.4f\n"),
						battery_voltage1,
						current_amps1,
						current_total1);
		}

		// write out the servo PWM values
		// ------------------------------
		set_servos();

		if(cliSerial->available() > 0){
			return (0);
		}
	}
} else {
	cliSerial->printf_P(PSTR("Not enabled\n"));
	return (0);
}

}

static int8_t
test_relay(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		cliSerial->printf_P(PSTR("Relay on\n"));
		relay.on();
		delay(3000);
		if(cliSerial->available() > 0){
			return (0);
		}

		cliSerial->printf_P(PSTR("Relay off\n"));
		relay.off();
		delay(3000);
		if(cliSerial->available() > 0){
			return (0);
		}
	}
}

static int8_t
test_wp(uint8_t argc, const Menu::arg *argv)
{
	delay(1000);

	cliSerial->printf_P(PSTR("%u waypoints\n"), (unsigned)g.command_total);
	cliSerial->printf_P(PSTR("Hit radius: %f\n"), g.waypoint_radius);

	for(uint8_t i = 0; i <= g.command_total; i++){
		struct Location temp = get_cmd_with_index(i);
		test_wp_print(&temp, i);
	}

	return (0);
}

static void
test_wp_print(struct Location *cmd, uint8_t wp_index)
{
	cliSerial->printf_P(PSTR("command #: %d id:%d options:%d p1:%d p2:%ld p3:%ld p4:%ld \n"),
		(int)wp_index,
		(int)cmd->id,
		(int)cmd->options,
		(int)cmd->p1,
		cmd->alt,
		cmd->lat,
		cmd->lng);
}

static int8_t
test_modeswitch(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	cliSerial->printf_P(PSTR("Control CH "));

	cliSerial->println(MODE_CHANNEL, DEC);

	while(1){
		delay(20);
		uint8_t switchPosition = readSwitch();
		if (oldSwitchPosition != switchPosition){
			cliSerial->printf_P(PSTR("Position %d\n"),  switchPosition);
			oldSwitchPosition = switchPosition;
		}
		if(cliSerial->available() > 0){
			return (0);
		}
	}
}

/*
  test the dataflash is working
 */
static int8_t
test_logging(uint8_t argc, const Menu::arg *argv)
{
/*
	cliSerial->println_P(PSTR("Testing dataflash logging"));
    if (!DataFlash.CardInserted()) {
        cliSerial->println_P(PSTR("ERR: No dataflash inserted"));
        return 0;
    }
    DataFlash.ReadManufacturerID();
    cliSerial->printf_P(PSTR("Manufacturer: 0x%02x   Device: 0x%04x\n"),
                    (unsigned)DataFlash.df_manufacturer,
                    (unsigned)DataFlash.df_device);
    cliSerial->printf_P(PSTR("NumPages: %u  PageSize: %u\n"),
                    (unsigned)DataFlash.df_NumPages+1,
                    (unsigned)DataFlash.df_PageSize);
    DataFlash.StartRead(DataFlash.df_NumPages+1);
    cliSerial->printf_P(PSTR("Format version: %lx  Expected format version: %lx\n"),
                    (unsigned long)DataFlash.ReadLong(), (unsigned long)DF_LOGGING_FORMAT);
    return 0;
*/
}


//-------------------------------------------------------------------------------------------
// tests in this section are for real sensors or sensors that have been simulated

#if HIL_MODE == HIL_MODE_DISABLED || HIL_MODE == HIL_MODE_SENSORS

#if CONFIG_ADC == ENABLED
static int8_t
test_adc(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	adc.Init();
	delay(1000);
	cliSerial->printf_P(PSTR("ADC\n"));
	delay(1000);

	while(1){
		for (int i=0;i<9;i++) cliSerial->printf_P(PSTR("%.1f\t"),adc.Ch(i));
		cliSerial->println();
		delay(100);
		if(cliSerial->available() > 0){
			return (0);
		}
	}
}
#endif // CONFIG_ADC

static int8_t
test_gps(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(333);

		// Blink GPS LED if we don't have a fix
		// ------------------------------------
		update_GPS_light();

		g_gps->update();

		if (g_gps->new_data){
			cliSerial->printf_P(PSTR("Lat: %ld, Lon %ld, Alt: %ldm, #sats: %d\n"),
					g_gps->latitude,
					g_gps->longitude,
					g_gps->altitude/100,
					g_gps->num_sats);
		}else{
			cliSerial->printf_P(PSTR("."));
		}
		if(cliSerial->available() > 0){
			return (0);
		}
	}
}

static int8_t
test_ins(uint8_t argc, const Menu::arg *argv)
{
	//cliSerial->printf_P(PSTR("Calibrating."));
	ahrs.init();
    ahrs.set_fly_forward(true);
	ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate, 
             flash_leds);
    ahrs.reset();

	print_hit_enter();
	delay(1000);

	while(1){
		delay(20);
		if (millis() - fast_loopTimer > 19) {
			delta_ms_fast_loop 	= millis() - fast_loopTimer;
			G_Dt 				= (float)delta_ms_fast_loop / 1000.f;		// used by DCM integrator
			fast_loopTimer		= millis();

			// INS
			// ---
			ahrs.update();

			if(g.compass_enabled) {
				medium_loopCounter++;
				if(medium_loopCounter == 5){
					compass.read();
                    medium_loopCounter = 0;
				}
			}

			// We are using the IMU
			// ---------------------
            Vector3f gyros 	= ins.get_gyro();
            Vector3f accels = ins.get_accel();
			cliSerial->printf_P(PSTR("r:%4d  p:%4d  y:%3d  g=(%5.1f %5.1f %5.1f)  a=(%5.1f %5.1f %5.1f)\n"),
                            (int)ahrs.roll_sensor / 100,
                            (int)ahrs.pitch_sensor / 100,
                            (uint16_t)ahrs.yaw_sensor / 100,
                            gyros.x, gyros.y, gyros.z,
                            accels.x, accels.y, accels.z);
		}
		if(cliSerial->available() > 0){
			return (0);
		}
	}
}


static int8_t
test_mag(uint8_t argc, const Menu::arg *argv)
{
	if (!g.compass_enabled) {
        cliSerial->printf_P(PSTR("Compass: "));
		print_enabled(false);
		return (0);
    }

    compass.set_orientation(MAG_ORIENTATION);
    if (!compass.init()) {
        cliSerial->println_P(PSTR("Compass initialisation failed!"));
        return 0;
    }
    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_compass(&compass);
    report_compass();

    // we need the AHRS initialised for this test
	ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate, 
             flash_leds);
    ahrs.reset();

	int counter = 0;
    float heading = 0;

		//cliSerial->printf_P(PSTR("MAG_ORIENTATION: %d\n"), MAG_ORIENTATION);

    print_hit_enter();

    while(1) {
		delay(20);
		if (millis() - fast_loopTimer > 19) {
			delta_ms_fast_loop 	= millis() - fast_loopTimer;
			G_Dt 				= (float)delta_ms_fast_loop / 1000.f;		// used by DCM integrator
			fast_loopTimer		= millis();

			// IMU
			// ---
			ahrs.update();

            medium_loopCounter++;
            if(medium_loopCounter == 5){
                if (compass.read()) {
                    // Calculate heading
                    Matrix3f m = ahrs.get_dcm_matrix();
                    heading = compass.calculate_heading(m);
                    compass.null_offsets();
                }
                medium_loopCounter = 0;
            }

			counter++;
			if (counter>20) {
                if (compass.healthy) {
                    Vector3f maggy = compass.get_offsets();
                    cliSerial->printf_P(PSTR("Heading: %ld, XYZ: %d, %d, %d,\tXYZoff: %6.2f, %6.2f, %6.2f\n"),
                                    (wrap_360(ToDeg(heading) * 100)) /100,
                                    (int)compass.mag_x,
                                    (int)compass.mag_y,
                                    (int)compass.mag_z,
                                    maggy.x,
                                    maggy.y,
                                    maggy.z);
                } else {
                    cliSerial->println_P(PSTR("compass not healthy"));
                }
                counter=0;
            }
		}
        if (cliSerial->available() > 0) {
            break;
        }
    }

    // save offsets. This allows you to get sane offset values using
    // the CLI before you go flying.    
    cliSerial->println_P(PSTR("saving offsets"));
    compass.save_offsets();
    return (0);
}

#endif // HIL_MODE == HIL_MODE_DISABLED || HIL_MODE == HIL_MODE_SENSORS

//-------------------------------------------------------------------------------------------
// real sensors that have not been simulated yet go here

#if CONFIG_SONAR == ENABLED
static int8_t
test_sonar(uint8_t argc, const Menu::arg *argv)
{
  print_hit_enter();
	delay(1000);
	init_sonar();
	delay(1000);

	while(1){
	  delay(20);
	  if(g.sonar_enabled){
		sonar_dist = sonar->read();
	  }
    	  cliSerial->printf_P(PSTR("sonar_dist = %d\n"), (int)sonar_dist);

          if(cliSerial->available() > 0){
  		break;
	    }
  }
  return (0);
}
#endif // SONAR == ENABLED

#endif // CLI_ENABLED
