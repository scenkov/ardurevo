#line 1 "./APMRover2/APMrover2.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduRover v2.45"
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* 
   This is the APMrover2 firmware. It was originally derived from
   ArduPlane by Jean-Louis Naudin (JLN), and then rewritten after the
   AP_HAL merge by Andrew Tridgell

   Maintainer: Andrew Tridgell

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Jean-Louis Naudin

   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier 

   APMrover alpha version tester: Franco Borasio, Daniel Chapelat... 

   Please contribute your ideas! See http://dev.ardupilot.com for details
*/

// Radio setup:
// APM INPUT (Rec = receiver)
// Rec ch1: Steering
// Rec ch2: not used
// Rec ch3: Throttle
// Rec ch4: not used
// Rec ch5: not used
// Rec ch6: not used
// Rec ch7: Option channel to 2 position switch
// Rec ch8: Mode channel to 6 position switch
// APM OUTPUT
// Ch1: Wheel servo (direction)
// Ch2: not used
// Ch3: to the motor ESC
// Ch4: not used

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
#include <AP_ServoRelayEvents.h>
#include <AP_Mount.h>		// Camera/Antenna mount
#include <AP_Camera.h>		// Camera triggering
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Airspeed.h>    // needed for AHRS build
#include <AP_Vehicle.h>     // needed for AHRS build
#include <DataFlash.h>
#include <AP_RCMapper.h>        // RC input mapping library
#include <SITL.h>
#include <AP_Scheduler.h>       // main loop scheduler
#include <stdarg.h>
#include <AP_Navigation.h>
#include <APM_Control.h>
#include <AP_L1_Control.h>
#include <AP_BoardConfig.h>

//#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_REVOMINI.h>
#include "compat.h"

#include <AP_Notify.h>      // Notify library
#include <AP_BattMonitor.h> // Battery monitor library

// Configuration
#include "config.h"

// Local modules
#include "defines.h"
#include "Parameters.h"
#include "GCS.h"

#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library

 void setup() ;
 void loop() ;
 static void ahrs_update() ;
 static void mount_update(void) ;
 static void gcs_failsafe_check(void) ;
 static void compass_accumulate(void) ;
 static void update_compass(void) ;
 static void update_logging1(void) ;
 static void update_logging2(void) ;
 static void update_aux(void) ;
 static void one_second_loop(void) ;
  static void update_GPS_50Hz(void) ;
   static void update_GPS_10Hz(void) ;
  static void update_current_mode(void) ;
  static void update_navigation() ;
  static NOINLINE void send_heartbeat(mavlink_channel_t chan) ;
  static NOINLINE void send_attitude(mavlink_channel_t chan) ;
  static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops) ;
  static void NOINLINE send_location(mavlink_channel_t chan) ;
  static void NOINLINE send_nav_controller_output(mavlink_channel_t chan) ;
  static void NOINLINE send_gps_raw(mavlink_channel_t chan) ;
  static void NOINLINE send_system_time(mavlink_channel_t chan) ;
 static void NOINLINE send_servo_out(mavlink_channel_t chan) ;
  static void NOINLINE send_radio_in(mavlink_channel_t chan) ;
  static void NOINLINE send_radio_out(mavlink_channel_t chan) ;
  static void NOINLINE send_vfr_hud(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu1(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu3(mavlink_channel_t chan) ;
  static void NOINLINE send_ahrs(mavlink_channel_t chan) ;
 static void NOINLINE send_simstate(mavlink_channel_t chan) ;
  static void NOINLINE send_hwstatus(mavlink_channel_t chan) ;
  static void NOINLINE send_rangefinder(mavlink_channel_t chan) ;
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
 static void gcs_retry_deferred(void) ;
  static bool print_log_menu(void) ;
   static void do_erase_logs(void) ;
 static void Log_Write_Performance() ;
 static void Log_Write_Cmd(uint8_t num, const struct Location *wp) ;
 static void Log_Write_Camera() ;
 static void Log_Write_Steering() ;
  static void Log_Write_Startup(uint8_t type) ;
 static void Log_Write_Control_Tuning() ;
 static void Log_Write_Nav_Tuning() ;
 static void Log_Write_Attitude() ;
 static void Log_Write_Mode() ;
 static void Log_Write_Sonar() ;
  static void Log_Write_Current() ;
 static void Log_Write_Compass() ;
   static void Log_Write_RC(void) ;
 static void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page) ;
 static void start_logging()  ;
 static void Log_Write_Startup(uint8_t type) ;
 static void Log_Write_Cmd(uint8_t num, const struct Location *wp) ;
 static void Log_Write_Current() ;
 static void Log_Write_Nav_Tuning() ;
 static void Log_Write_Performance() ;
 static void Log_Write_Control_Tuning() ;
 static void Log_Write_Sonar() ;
 static void Log_Write_Mode() ;
 static void Log_Write_Attitude() ;
 static void Log_Write_Compass() ;
 static void start_logging() ;
 static void Log_Write_RC(void) ;
  static void load_parameters(void) ;
 static void throttle_slew_limit(int16_t last_throttle) ;
 static bool auto_check_trigger(void) ;
 static void calc_throttle(float target_speed) ;
  static void calc_lateral_acceleration() ;
 static void calc_nav_steer() ;
 static void set_servos(void) ;
  static void init_commands() ;
 static struct Location get_cmd_with_index(int i) ;
 static void set_cmd_with_index(struct Location temp, int i) ;
 static void set_next_WP(const struct Location *wp) ;
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
 static void do_take_picture() ;
 static void change_command(uint8_t cmd_index) ;
 static void update_commands(void) ;
  static void verify_commands(void) ;
   static void process_next_command() ;
 static void process_nav_cmd() ;
  static void process_non_nav_command() ;
  static void delay(uint32_t ms) ;
  static void mavlink_delay(uint32_t ms) ;
  static uint32_t millis() ;
  static uint32_t micros() ;
  static void read_control_switch() ;
  static uint8_t readSwitch(void);
  static void reset_control_switch() ;
 static void read_trim_switch() ;
   static void update_events(void) ;
 static void failsafe_check() ;
 static void navigate() ;
   void reached_waypoint() ;
 static void set_control_channels(void) ;
  static void init_rc_in() ;
  static void init_rc_out() ;
  static void read_radio() ;
  static void control_failsafe(uint16_t pwm) ;
  static void trim_control_surfaces() ;
  static void trim_radio() ;
  static void init_sonar(void) ;
 static void read_battery(void) ;
 void read_receiver_rssi(void) ;
 static void read_sonars(void) ;
  static void report_batt_monitor() ;
 static void report_radio() ;
  static void report_gains() ;
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
 static void set_reverse(bool reverse) ;
  static void set_mode(enum mode mode) ;
 static void failsafe_trigger(uint8_t failsafe_type, bool on) ;
  static void startup_INS_ground(bool force_accel_level) ;
 static void update_notify() ;
  static void resetPerfData(void) ;
 static uint32_t map_baudrate(int8_t rate, uint32_t default_baud) ;
   static void check_usb_mux(void) ;
 uint16_t board_voltage(void) ;
 static uint8_t check_digital_pin(uint8_t pin) ;
 static void servo_write(uint8_t ch, uint16_t pwm) ;
 static bool should_log(uint32_t mask) ;
  static void print_hit_enter() ;
  static void test_wp_print(const struct Location *cmd, uint8_t wp_index) ;
#line 115 "./APMRover2/APMrover2.pde"
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

// main loop scheduler
static AP_Scheduler scheduler;

// mapping between input channels
static RCMapper rcmap;

// board specific config
static AP_BoardConfig BoardConfig;

// primary control channels
static RC_Channel *channel_steer;
static RC_Channel *channel_throttle;
static RC_Channel *channel_learn;

////////////////////////////////////////////////////////////////////////////////
// prototypes
static void update_events(void);
void gcs_send_text_fmt(const prog_char_t *fmt, ...);
static void print_mode(AP_HAL::BetterStream *port, uint8_t mode);

////////////////////////////////////////////////////////////////////////////////
// DataFlash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
static DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
static DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
static DataFlash_File DataFlash("logs");
//static DataFlash_SITL DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
static DataFlash_File DataFlash("/fs/microsd/APM/LOGS");
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
static DataFlash_File DataFlash("logs");
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
static DataFlash_VRBRAIN DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_REVOMINI
static DataFlash_REVOMINI DataFlash;
#else
DataFlash_Empty DataFlash;
#endif

static bool in_log_download;

////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal driving mode.  Real sensors are used.
// - HIL Attitude mode.  Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode.  Synthetic sensors are configured that
//   supply data from the simulation.
//

// All GPS access should be through this pointer.
static GPS         *g_gps;

// flight modes convenience array
static AP_Int8		*modes = &g.mode1;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
static AP_ADC_ADS7844 adc;
#endif

#if CONFIG_COMPASS == AP_COMPASS_PX4
static AP_Compass_PX4 compass;
#elif CONFIG_COMPASS == AP_COMPASS_HMC5843
static AP_Compass_HMC5843 compass;
#elif CONFIG_COMPASS == AP_COMPASS_HIL
static AP_Compass_HIL compass;
#else
 #error Unrecognized CONFIG_COMPASS setting
#endif

// GPS selection
#if   GPS_PROTOCOL == GPS_PROTOCOL_AUTO
AP_GPS_Auto     g_gps_driver(&g_gps);

#elif GPS_PROTOCOL == GPS_PROTOCOL_NMEA
AP_GPS_NMEA     g_gps_driver;

#elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
AP_GPS_SIRF     g_gps_driver;

#elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
AP_GPS_UBLOX    g_gps_driver;

#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
AP_GPS_MTK      g_gps_driver;

#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK19
AP_GPS_MTK19    g_gps_driver;

#elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_None     g_gps_driver;

#elif GPS_PROTOCOL == GPS_PROTOCOL_HIL
AP_GPS_HIL      g_gps_driver;

#else
  #error Unrecognised GPS_PROTOCOL setting.
#endif // GPS PROTOCOL

#if CONFIG_INS_TYPE == CONFIG_INS_MPU6000
AP_InertialSensor_MPU6000 ins;
#elif CONFIG_INS_TYPE == CONFIG_INS_PX4
AP_InertialSensor_PX4 ins;
#elif CONFIG_INS_TYPE == CONFIG_INS_HIL
AP_InertialSensor_HIL ins;
#elif CONFIG_INS_TYPE == CONFIG_INS_FLYMAPLE
AP_InertialSensor_Flymaple ins;
#elif CONFIG_INS_TYPE == CONFIG_INS_L3G4200D
AP_InertialSensor_L3G4200D ins;
#elif CONFIG_INS_TYPE == CONFIG_INS_OILPAN
AP_InertialSensor_Oilpan ins( &adc );
#else
  #error Unrecognised CONFIG_INS_TYPE setting.
#endif // CONFIG_INS_TYPE

AP_AHRS_DCM ahrs(ins, g_gps);

static AP_L1_Control L1_controller(ahrs);

// selected navigation controller
static AP_Navigation *nav_controller = &L1_controller;

// steering controller
static AP_SteerController steerController(ahrs);

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
SITL sitl;
#endif

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
//
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

// a pin for reading the receiver RSSI voltage. The scaling by 0.25 
// is to take the 0 to 1024 range down to an 8 bit range for MAVLink
AP_HAL::AnalogSource *rssi_analog_source;

AP_HAL::AnalogSource *vcc_pin;

////////////////////////////////////////////////////////////////////////////////
// SONAR selection
////////////////////////////////////////////////////////////////////////////////
//
static AP_RangeFinder_analog sonar;
static AP_RangeFinder_analog sonar2;

// relay support
AP_Relay relay;

AP_ServoRelayEvents ServoRelayEvents(relay);

// Camera
#if CAMERA == ENABLED
static AP_Camera camera(&relay);
#endif

// The rover's current location
static struct 	Location current_loc;


// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
// mabe one could use current_loc for lat/lon too and eliminate g_gps alltogether?
AP_Mount camera_mount(&current_loc, g_gps, ahrs, 0);
#endif


////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

// if USB is connected
static bool usb_connected;

/* Radio values
		Channel assignments
			1   Steering
			2   ---
			3   Throttle
			4   ---
			5   Aux5
			6   Aux6
			7   Aux7/learn
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
// Used for failsafe based on loss of RC signal or GCS signal. See 
// FAILSAFE_EVENT_*
static struct {
    uint8_t bits;
    uint32_t rc_override_timer;
    uint32_t start_time;
    uint8_t triggered;
    uint32_t last_valid_rc_ms;
} failsafe;

// notification object for LEDs, buzzers etc (parameter set to false disables external leds)
static AP_Notify notify;

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

// obstacle detection information
static struct {
    // have we detected an obstacle?
    uint8_t detected_count;
    float turn_angle;
    uint16_t sonar1_distance_cm;
    uint16_t sonar2_distance_cm;

    // time when we last detected an obstacle, in milliseconds
    uint32_t detected_time_ms;
} obstacle;

// this is set to true when auto has been triggered to start
static bool auto_triggered;

////////////////////////////////////////////////////////////////////////////////
// Ground speed
////////////////////////////////////////////////////////////////////////////////
// The amount current ground speed is below min ground speed.  meters per second
static float 	ground_speed = 0;
static int16_t throttle_last = 0, throttle = 500;

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

////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
static AP_BattMonitor battery;

////////////////////////////////////////////////////////////////////////////////
// Navigation control variables
////////////////////////////////////////////////////////////////////////////////
// The instantaneous desired lateral acceleration in m/s/s
static float lateral_acceleration;

////////////////////////////////////////////////////////////////////////////////
// Waypoint distances
////////////////////////////////////////////////////////////////////////////////
// Distance between rover and next waypoint.  Meters
static float wp_distance;
// Distance between previous and next waypoint.  Meters
static int32_t wp_totalDistance;

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
static uint32_t 	G_Dt_max;
// The number of gps fixes recorded in the current performance monitoring interval
static uint8_t 	gps_fix_count = 0;

////////////////////////////////////////////////////////////////////////////////
// System Timers
////////////////////////////////////////////////////////////////////////////////
// Time in microseconds of start of main control loop. 
static uint32_t 	fast_loopTimer_us;
// Number of milliseconds used in last main loop cycle
static uint32_t		delta_us_fast_loop;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t			mainLoop_count;

// set if we are driving backwards
static bool in_reverse;

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

/*
  scheduler table - all regular tasks should be listed here, along
  with how often they should be called (in 20ms units) and the maximum
  time they are expected to take (in microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { read_radio,             1,   1000 },
    { ahrs_update,            1,   6400 },
    { read_sonars,            1,   2000 },
    { update_current_mode,    1,   1500 },
    { set_servos,             1,   1500 },
    { update_GPS_50Hz,        1,   2500 },
    { update_GPS_10Hz,        5,   2500 },
    { navigate,               5,   1600 },
    { update_compass,         5,   2000 },
    { update_commands,        5,   1000 },
    { update_logging1,        5,   1000 },
    { update_logging2,        5,   1000 },
    { gcs_retry_deferred,     1,   1000 },
    { gcs_update,             1,   1700 },
    { gcs_data_stream_send,   1,   3000 },
    { read_control_switch,   15,   1000 },
    { read_trim_switch,       5,   1000 },
    { read_battery,           5,   1000 },
    { read_receiver_rssi,     5,   1000 },
    { update_events,          1,   1000 },
    { check_usb_mux,         15,   1000 },
    { mount_update,           1,    600 },
    { gcs_failsafe_check,     5,    600 },
    { compass_accumulate,     1,    900 },
    { update_notify,          1,    300 },
    { one_second_loop,       50,   3000 }
};


/*
  setup is called when the sketch starts
 */
void setup() {
    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    // rover does not use arming nor pre-arm checks
    AP_Notify::flags.armed = true;
    AP_Notify::flags.pre_arm_check = true;
    AP_Notify::flags.failsafe_battery = false;

    notify.init(false);

    battery.init();

    rssi_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);
    vcc_pin = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);

	init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

/*
  loop() is called rapidly while the sketch is running
 */
void loop()
{
    // wait for an INS sample
    if (!ins.wait_for_sample(1000)) {
        return;
    }
    uint32_t timer = hal.scheduler->micros();

    delta_us_fast_loop	= timer - fast_loopTimer_us;
    G_Dt                = delta_us_fast_loop * 1.0e-6f;
    fast_loopTimer_us   = timer;

	if (delta_us_fast_loop > G_Dt_max)
		G_Dt_max = delta_us_fast_loop;

    mainLoop_count++;

    // tell the scheduler one tick has passed
    scheduler.tick();

    scheduler.run(19500U);
}

// update AHRS system
static void ahrs_update()
{
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before AHRS update
    gcs_update();
#endif

    // when in reverse we need to tell AHRS not to assume we are a
    // 'fly forward' vehicle, otherwise it will see a large
    // discrepancy between the mag and the GPS heading and will try to
    // correct for it, leading to a large yaw error
    ahrs.set_fly_forward(!in_reverse);

    ahrs.update();

    if (should_log(MASK_LOG_ATTITUDE_FAST))
        Log_Write_Attitude();

    if (should_log(MASK_LOG_IMU))
        DataFlash.Log_Write_IMU(ins);
}

/*
  update camera mount - 50Hz
 */
static void mount_update(void)
{
#if MOUNT == ENABLED
	camera_mount.update_mount_position();
#endif
#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
#endif
}

/*
  check for GCS failsafe - 10Hz
 */
static void gcs_failsafe_check(void)
{
	if (g.fs_gcs_enabled) {
        failsafe_trigger(FAILSAFE_EVENT_GCS, last_heartbeat_ms != 0 && (millis() - last_heartbeat_ms) > 2000);
    }
}

/*
  if the compass is enabled then try to accumulate a reading
 */
static void compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }    
}

/*
  check for new compass data - 10Hz
 */
static void update_compass(void)
{
    if (g.compass_enabled && compass.read()) {
        ahrs.set_compass(&compass);
        // update offsets
        compass.null_offsets();
        if (should_log(MASK_LOG_COMPASS)) {
            Log_Write_Compass();
        }
    } else {
        ahrs.set_compass(NULL);
    }
}

/*
  log some key data - 10Hz
 */
static void update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST))
        Log_Write_Attitude();
    
    if (should_log(MASK_LOG_CTUN))
        Log_Write_Control_Tuning();

    if (should_log(MASK_LOG_NTUN))
        Log_Write_Nav_Tuning();
}

/*
  log some key data - 10Hz
 */
static void update_logging2(void)
{
    if (should_log(MASK_LOG_STEERING)) {
        if (control_mode == STEERING || control_mode == AUTO || control_mode == RTL || control_mode == GUIDED) {
            Log_Write_Steering();
        }
    }

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();
}


/*
  update aux servo mappings
 */
static void update_aux(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_9, &g.rc_10, &g.rc_11, &g.rc_12);
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_10, &g.rc_11);
#else
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8);
#endif
    enable_aux_servos();
        
#if MOUNT == ENABLED
    camera_mount.update_mount_type();
#endif
}

/*
  once a second events
 */
static void one_second_loop(void)
{
	if (should_log(MASK_LOG_CURRENT))
		Log_Write_Current();
	// send a heartbeat
	gcs_send_message(MSG_HEARTBEAT);

    // allow orientation change at runtime to aid config
    ahrs.set_orientation();

    set_control_channels();

    // cope with changes to aux functions
    update_aux();

#if MOUNT == ENABLED
    camera_mount.update_mount_type();
#endif

    // cope with changes to mavlink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    static uint8_t counter;

    counter++;

    // write perf data every 20s
    if (counter % 10 == 0) {
        if (scheduler.debug() != 0) {
            hal.console->printf_P(PSTR("G_Dt_max=%lu\n"), (unsigned long)G_Dt_max);
        }
        if (should_log(MASK_LOG_PM))
            Log_Write_Performance();
        G_Dt_max = 0;
        resetPerfData();
    }

    // save compass offsets once a minute
    if (counter >= 60) {				
        if (g.compass_enabled) {
            compass.save_offsets();
        }
        counter = 0;
    }
}

static void update_GPS_50Hz(void)
{        
    static uint32_t last_gps_reading;
	g_gps->update();

    if (g_gps->last_message_time_ms() != last_gps_reading) {
        last_gps_reading = g_gps->last_message_time_ms();
        if (should_log(MASK_LOG_GPS)) {
            DataFlash.Log_Write_GPS(g_gps, current_loc.alt);
        }
    }
}


static void update_GPS_10Hz(void)
{        
    have_position = ahrs.get_projected_position(current_loc);

	if (g_gps->new_data && g_gps->status() >= GPS::GPS_OK_FIX_3D) {
		gps_fix_count++;

		if(ground_start_count > 1){
			ground_start_count--;
			ground_start_avg += g_gps->ground_speed_cm;

		} else if (ground_start_count == 1) {
			// We countdown N number of good GPS fixes
			// so that the altitude is more accurate
			// -------------------------------------
			if (current_loc.lat == 0) {
				ground_start_count = 5;

			} else {
                init_home();

                // set system clock for log timestamps
                hal.util->set_system_clock(g_gps->time_epoch_usec());

				if (g.compass_enabled) {
					// Set compass declination automatically
					compass.set_initial_location(g_gps->latitude, g_gps->longitude);
				}
				ground_start_count = 0;
			}
		}
        ground_speed   = g_gps->ground_speed_cm * 0.01;

#if CAMERA == ENABLED
        if (camera.update_location(current_loc) == true) {
            do_take_picture();
        }
#endif        
	}
}

static void update_current_mode(void)
{ 
    switch (control_mode){
    case AUTO:
    case RTL:
    case GUIDED:
        set_reverse(false);
        calc_lateral_acceleration();
        calc_nav_steer();
        calc_throttle(g.speed_cruise);
        break;

    case STEERING: {
        /*
          in steering mode we control lateral acceleration
          directly. We first calculate the maximum lateral
          acceleration at full steering lock for this speed. That is
          V^2/R where R is the radius of turn. We get the radius of
          turn from half the STEER2SRV_P.
         */
        float max_g_force = ground_speed * ground_speed / steerController.get_turn_radius();

        // constrain to user set TURN_MAX_G
        max_g_force = constrain_float(max_g_force, 0.1f, g.turn_max_g * GRAVITY_MSS);

        lateral_acceleration = max_g_force * (channel_steer->pwm_to_angle()/4500.0f);
        calc_nav_steer();

        // and throttle gives speed in proportion to cruise speed, up
        // to 50% throttle, then uses nudging above that.
        float target_speed = channel_throttle->pwm_to_angle() * 0.01 * 2 * g.speed_cruise;
        set_reverse(target_speed < 0);
        if (in_reverse) {
            target_speed = constrain_float(target_speed, -g.speed_cruise, 0);
        } else {
            target_speed = constrain_float(target_speed, 0, g.speed_cruise);
        }
        calc_throttle(target_speed);
        break;
    }

    case LEARNING:
    case MANUAL:
        /*
          in both MANUAL and LEARNING we pass through the
          controls. Setting servo_out here actually doesn't matter, as
          we set the exact value in set_servos(), but it helps for
          logging
         */
        channel_throttle->servo_out = channel_throttle->control_in;
        channel_steer->servo_out = channel_steer->pwm_to_angle();

        // mark us as in_reverse when using a negative throttle to
        // stop AHRS getting off
        set_reverse(channel_throttle->servo_out < 0);
        break;

    case HOLD:
        // hold position - stop motors and center steering
        channel_throttle->servo_out = 0;
        channel_steer->servo_out = 0;
        set_reverse(false);
        break;

    case INITIALISING:
        break;
	}
}

static void update_navigation()
{
    switch (control_mode) {
    case MANUAL:
    case HOLD:
    case LEARNING:
    case STEERING:
    case INITIALISING:
        break;

    case AUTO:
		verify_commands();
        break;

    case RTL:
    case GUIDED:
        // no loitering around the wp with the rover, goes direct to the wp position
        calc_lateral_acceleration();
        calc_nav_steer();
        if (verify_RTL()) {  
            channel_throttle->servo_out = g.throttle_min.get();
            set_mode(HOLD);
        }
        break;
	}
}

AP_HAL_MAIN();
#line 1 "./APMRover2/GCS_Mavlink.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// default sensors are present and healthy: gyro, accelerometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS)

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;

// true if we are out of time in our event timeslice
static bool	gcs_out_of_time;

// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_## id ##_LEN) return false

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
    
    if (failsafe.triggered != 0) {
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
    case LEARNING:
    case STEERING:
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
    case HOLD:
        system_status = 0;
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
        MAV_TYPE_GROUND_ROVER,
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
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }
    if (g_gps != NULL && g_gps->status() > GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }

    // all present sensors enabled by default except rate control, attitude stabilization, yaw, altitude, position control and motor output which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL & ~MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION & ~MAV_SYS_STATUS_SENSOR_YAW_POSITION & ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL & ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);

    switch (control_mode) {
    case MANUAL:
    case HOLD:
        break;

    case LEARNING:
    case STEERING:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        break;

    case AUTO:
    case RTL:
    case GUIDED:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; // yaw position
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL; // X/Y position control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS; // motor control
        break;

    case INITIALISING:
        break;
    }

    // default to all healthy except compass and gps which we set individually
    control_sensors_health = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_3D_MAG & ~MAV_SYS_STATUS_SENSOR_GPS);
    if (g.compass_enabled && compass.healthy() && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (g_gps != NULL && g_gps->status() >= GPS::GPS_OK_FIX_3D) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (!ins.healthy()) {
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }

    int16_t battery_current = -1;
    int8_t battery_remaining = -1;

    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        battery_remaining = battery.capacity_remaining_pct();
        battery_current = battery.current_amps() * 100;
    }

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(scheduler.load_average(20000) * 1000),
        battery.voltage() * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);

}

static void NOINLINE send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.    
    if (g_gps->status() >= GPS::GPS_OK_FIX_2D) {
        fix_time = g_gps->last_fix_time;
    } else {
        fix_time = millis();
    }
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        g_gps->altitude_cm * 10,             // millimeters above sea level
        (current_loc.alt - home.alt) * 10,           // millimeters above ground
        g_gps->velocity_north() * 100,  // X speed cm/s (+ve North)
        g_gps->velocity_east()  * 100,  // Y speed cm/s (+ve East)
        g_gps->velocity_down()  * -100, // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);
}

static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)
{
    mavlink_msg_nav_controller_output_send(
        chan,
        lateral_acceleration, // use nav_roll to hold demanded Y accel
        g_gps->ground_speed_cm * 0.01f * ins.get_gyro().z, // use nav_pitch to hold actual Y accel
        nav_controller->nav_bearing_cd() * 0.01f,
        nav_controller->target_bearing_cd() * 0.01f,
        wp_distance,
        0,
        groundspeed_error,
        nav_controller->crosstrack_error());
}

static void NOINLINE send_gps_raw(mavlink_channel_t chan)
{
    mavlink_msg_gps_raw_int_send(
        chan,
        g_gps->last_fix_time*(uint64_t)1000,
        g_gps->status(),
        g_gps->latitude,      // in 1E7 degrees
        g_gps->longitude,     // in 1E7 degrees
        g_gps->altitude_cm * 10, // in mm
        g_gps->hdop,
        65535,
        g_gps->ground_speed_cm,  // cm/s
        g_gps->ground_course_cd, // 1/100 degrees,
        g_gps->num_sats);
}

static void NOINLINE send_system_time(mavlink_channel_t chan)
{
    mavlink_msg_system_time_send(
        chan,
        g_gps->time_epoch_usec(),
        hal.scheduler->millis());
}


#if HIL_MODE != HIL_MODE_DISABLED
static void NOINLINE send_servo_out(mavlink_channel_t chan)
{
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with
    // HIL maintainers
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        10000 * channel_steer->norm_output(),
        0,
        10000 * channel_throttle->norm_output(),
        0,
        0,
        0,
        0,
        0,
        receiver_rssi);
}
#endif

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
    mavlink_msg_servo_output_raw_send(
        chan,
        micros(),
        0,     // port
        RC_Channel::rc_channel(0)->radio_out,
        RC_Channel::rc_channel(1)->radio_out,
        RC_Channel::rc_channel(2)->radio_out,
        RC_Channel::rc_channel(3)->radio_out,
        RC_Channel::rc_channel(4)->radio_out,
        RC_Channel::rc_channel(5)->radio_out,
        RC_Channel::rc_channel(6)->radio_out,
        RC_Channel::rc_channel(7)->radio_out);
#endif
}

static void NOINLINE send_vfr_hud(mavlink_channel_t chan)
{
    mavlink_msg_vfr_hud_send(
        chan,
        (float)g_gps->ground_speed_cm / 100.0,
        (float)g_gps->ground_speed_cm / 100.0,
        (ahrs.yaw_sensor / 100) % 360,
        (uint16_t)(100 * fabsf(channel_throttle->norm_output())),
        current_loc.alt / 100.0,
        0);
}

static void NOINLINE send_raw_imu1(mavlink_channel_t chan)
{
    const Vector3f &accel = ins.get_accel();
    const Vector3f &gyro = ins.get_gyro();
    const Vector3f &mag = compass.get_field();

    mavlink_msg_raw_imu_send(
        chan,
        micros(),
        accel.x * 1000.0 / GRAVITY_MSS,
        accel.y * 1000.0 / GRAVITY_MSS,
        accel.z * 1000.0 / GRAVITY_MSS,
        gyro.x * 1000.0,
        gyro.y * 1000.0,
        gyro.z * 1000.0,
        mag.x,
        mag.y,
        mag.z);

    if (ins.get_gyro_count() <= 1 &&
        ins.get_accel_count() <= 1 &&
        compass.get_count() <= 1) {
        return;
    }
    const Vector3f &accel2 = ins.get_accel(1);
    const Vector3f &gyro2 = ins.get_gyro(1);
    const Vector3f &mag2 = compass.get_field(1);
    mavlink_msg_scaled_imu2_send(
        chan,
        millis(),
        accel2.x * 1000.0f / GRAVITY_MSS,
        accel2.y * 1000.0f / GRAVITY_MSS,
        accel2.z * 1000.0f / GRAVITY_MSS,
        gyro2.x * 1000.0f,
        gyro2.y * 1000.0f,
        gyro2.z * 1000.0f,
        mag2.x,
        mag2.y,
        mag2.z);        
}

static void NOINLINE send_raw_imu3(mavlink_channel_t chan)
{
    const Vector3f &mag_offsets = compass.get_offsets();
    const Vector3f &accel_offsets = ins.get_accel_offsets();
    const Vector3f &gyro_offsets = ins.get_gyro_offsets();

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
    const Vector3f &omega_I = ahrs.get_gyro_drift();
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

// report simulator state
static void NOINLINE send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    sitl.simstate_send(chan);
#endif
}

static void NOINLINE send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        board_voltage(),
        hal.i2c->lockup_count());
}

static void NOINLINE send_rangefinder(mavlink_channel_t chan)
{
    if (!sonar.enabled()) {
        // no sonar to report
        return;
    }

    /*
      report smaller distance of two sonars if more than one enabled
     */
    float distance_cm, voltage;
    if (!sonar2.enabled()) {
        distance_cm = sonar.distance_cm();
        voltage = sonar.voltage();
    } else {
        float dist1 = sonar.distance_cm();
        float dist2 = sonar2.distance_cm();
        if (dist1 <= dist2) {
            distance_cm = dist1;
            voltage = sonar.voltage();
        } else {
            distance_cm = dist2;
            voltage = sonar2.voltage();
        }
    }
    mavlink_msg_rangefinder_send(
        chan,
        distance_cm * 0.01f,
        voltage);
}

static void NOINLINE send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_mission_current_send(
        chan,
        g.command_index);
}

static void NOINLINE send_statustext(mavlink_channel_t chan)
{
    mavlink_statustext_t *s = &gcs[chan-MAVLINK_COMM_0].pending_status;
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
    if (chan == MAVLINK_COMM_0 && hal.gpio->usb_connected()) {
        // this is USB telemetry, so won't be an Xbee
        return false;
    }
    // we're either on the 2nd UART, or no USB cable is connected
    // we need to delay telemetry by the TELEM_DELAY time
    return true;
}


// try to send a message, return false if it won't fit in the serial tx buffer
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    int16_t payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;

    if (telemetry_delayed(chan)) {
        return false;
    }

    // if we don't have at least 1ms remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (!in_mavlink_delay && scheduler.time_available_usec() < 1200) {
        gcs_out_of_time = true;
        return false;
    }

    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        gcs[chan-MAVLINK_COMM_0].last_heartbeat_time = hal.scheduler->millis();        
        send_heartbeat(chan);
        return true;

    case MSG_EXTENDED_STATUS1:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        send_extended_status1(chan, packet_drops);
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        gcs[chan-MAVLINK_COMM_0].send_meminfo();
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

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
        send_system_time(chan);
        break;

    case MSG_SERVO_OUT:
#if HIL_MODE != HIL_MODE_DISABLED
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        send_servo_out(chan);
#endif
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
        gcs[chan-MAVLINK_COMM_0].queued_param_send();
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        gcs[chan-MAVLINK_COMM_0].queued_waypoint_send();
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
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        send_simstate(chan);
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus(chan);
        break;

    case MSG_RANGEFINDER:
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        send_rangefinder(chan);
        break;

    case MSG_RAW_IMU2:
    case MSG_LIMITS_STATUS:
    case MSG_FENCE_STATUS:
    case MSG_WIND:
        // unused
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
} mavlink_queue[MAVLINK_COMM_NUM_BUFFERS];

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
        mavlink_statustext_t *s = &gcs[chan-MAVLINK_COMM_0].pending_status;
        s->severity = (uint8_t)severity;
        strncpy((char *)s->text, str, sizeof(s->text));
        mavlink_send_message(chan, MSG_STATUSTEXT, 0);
    } else {
        // send immediately
        mavlink_msg_statustext_send(chan, severity, str);
    }
}

/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Raw sensor stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  1),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Extended status stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  1),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: RC Channel stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[2],  1),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Raw Control stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRates[3],  1),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Position stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  1),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Extra data type 1 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[5],  1),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Extra data type 2 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[6],  1),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Extra data type 3 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[7],  1),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Parameter stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[8],  10),
    AP_GROUPEND
};


void
GCS_MAVLINK::update(void)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
	status.packet_rx_drop_count = 0;

    // process received bytes
    uint16_t nbytes = comm_get_available(chan);
    for (uint16_t i=0; i<nbytes; i++)
    {
        uint8_t c = comm_receive_ch(chan);

#if CLI_ENABLED == ENABLED
        /* allow CLI to be started by hitting enter 3 times, if no
         *  heartbeat packets have been received */
        if (mavlink_active == 0 && (millis() - _cli_timeout) < 20000 && 
            comm_is_idle(chan)) {
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
            if (msg.msgid != MAVLINK_MSG_ID_RADIO && msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
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
    if (stream_num >= NUM_STREAMS) {
        return false;
    }
    float rate = (uint8_t)streamRates[stream_num].get();

    // send at a much lower rate while handling waypoints and
    // parameter sends
    if ((stream_num != STREAM_PARAMS) && 
        (waypoint_receiving || _queued_parameter != NULL)) {
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
    gcs_out_of_time = false;

    if (!in_mavlink_delay) {
        handle_log_send(DataFlash);
    }

    if (_queued_parameter != NULL) {
        if (streamRates[STREAM_PARAMS].get() <= 0) {
            streamRates[STREAM_PARAMS].set(10);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
    }

    if (gcs_out_of_time) return;

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

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU3);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);            // TODO - remove this message after location message is working
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_POSITION)) {
        // sent with GPS read
        send_message(MSG_LOCATION);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_RADIO_OUT);
        send_message(MSG_RADIO_IN);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
        send_message(MSG_RANGEFINDER);
        send_message(MSG_SYSTEM_TIME);
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
        if (m.text[i] == '\0') {
            break;
        }
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
            // note that we don't set STREAM_PARAMS - that is internal only
            for (uint8_t i=0; i<STREAM_PARAMS; i++) {
                streamRates[i].set_and_save_ifchanged(freq);
            }
                    break;

                case MAV_DATA_STREAM_RAW_SENSORS:
            streamRates[STREAM_RAW_SENSORS].set_and_save_ifchanged(freq);
                    break;
                case MAV_DATA_STREAM_EXTENDED_STATUS:
            streamRates[STREAM_EXTENDED_STATUS].set_and_save_ifchanged(freq);
                    break;
                case MAV_DATA_STREAM_RC_CHANNELS:
            streamRates[STREAM_RC_CHANNELS].set_and_save_ifchanged(freq);
                    break;
                case MAV_DATA_STREAM_RAW_CONTROLLER:
            streamRates[STREAM_RAW_CONTROLLER].set_and_save_ifchanged(freq);
                    break;
                case MAV_DATA_STREAM_POSITION:
            streamRates[STREAM_POSITION].set_and_save_ifchanged(freq);
                    break;
                case MAV_DATA_STREAM_EXTRA1:
            streamRates[STREAM_EXTRA1].set_and_save_ifchanged(freq);
                    break;
                case MAV_DATA_STREAM_EXTRA2:
            streamRates[STREAM_EXTRA2].set_and_save_ifchanged(freq);
                    break;
                case MAV_DATA_STREAM_EXTRA3:
            streamRates[STREAM_EXTRA3].set_and_save_ifchanged(freq);
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
                    startup_INS_ground(true);
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
            if (ServoRelayEvents.do_set_servo(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            if (ServoRelayEvents.do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_RELAY:
            if (ServoRelayEvents.do_set_relay(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            if (ServoRelayEvents.do_repeat_relay(packet.param1, packet.param2, packet.param3*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (packet.param1 == 1 || packet.param1 == 3) {
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(packet.param1 == 3);
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
            case HOLD:
            case LEARNING:
            case STEERING:
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
                    param4 = tell_command.lng*0.001f; // time
                    param3 = tell_command.lat;        // repeat
                    param2 = tell_command.alt;        // pwm
                    param1 = tell_command.p1;         // channel
                    break;

            	case MAV_CMD_DO_REPEAT_RELAY:
                    param3 = tell_command.lat*0.001f; // time
                    param2 = tell_command.alt;        // count
                    param1 = tell_command.p1;         // relay number
                    break;

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

				case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
					param1 = tell_command.alt;
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

            // mark the firmware version in the tlog
            send_text_P(SEVERITY_LOW, PSTR(FIRMWARE_STRING));

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
            send_text_P(SEVERITY_LOW, PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif

            // send system ID if we can
            char sysid[40];
            if (hal.util->get_system_id(sysid)) {
                mavlink_send_text(chan, SEVERITY_LOW, sysid);
            }

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
            vp->copy_name_token(token, param_name, AP_MAX_NAME_SIZE, true);
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
                tell_command.lng = packet.param4*1000; // time
                tell_command.lat = packet.param3;      // count
                tell_command.alt = packet.param2;      // PWM
                tell_command.p1  = packet.param1;      // channel
                break;

            case MAV_CMD_DO_REPEAT_RELAY:
                tell_command.lat = packet.param3*1000; // time
                tell_command.alt = packet.param2;      // count
                tell_command.p1  = packet.param1;      // relay number
                break;

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

            case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                tell_command.alt = packet.param1;
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
                v = constrain_float(v, -2147483648.0, 2147483647.0);
					((AP_Int32 *)vp)->set_and_save(v);
                } else if (var_type == AP_PARAM_INT16) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    float v = packet.param_value+rounding_addition;
                    v = constrain_float(v, -32768, 32767);
					((AP_Int16 *)vp)->set_and_save(v);
                } else if (var_type == AP_PARAM_INT8) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    float v = packet.param_value+rounding_addition;
                    v = constrain_float(v, -128, 127);
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
                DataFlash.Log_Write_Parameter(key, vp->cast_to_float(var_type));
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

        failsafe.rc_override_timer = millis();
        failsafe_trigger(FAILSAFE_EVENT_RC, false);
        break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT:
        {
            // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
			if(msg->sysid != g.sysid_my_gcs) break;
            last_heartbeat_ms = failsafe.rc_override_timer = millis();
            failsafe_trigger(FAILSAFE_EVENT_GCS, false);
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
            compass.setHIL(packet.roll, packet.pitch, packet.yaw);
            break;
		}
#endif // HIL_MODE

#if CAMERA == ENABLED
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
    {
        camera.configure_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
    {
        camera.control_msg(msg);
        break;
    }
#endif // CAMERA == ENABLED

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
    case MAVLINK_MSG_ID_RADIO_STATUS:
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

    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
        in_log_download = true;
        // fallthru
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        if (!in_mavlink_delay) {
            handle_log_message(msg, DataFlash);
        }
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        in_log_download = false;
        if (!in_mavlink_delay) {
            handle_log_message(msg, DataFlash);
        }
        break;

    default:
        // forward unknown messages to the other link if there is one
        for (uint8_t i=0; i<num_gcs; i++) {
            if (gcs[i].initialised && i != (uint8_t)chan) {
                mavlink_channel_t out_chan = (mavlink_channel_t)i;
                // only forward if it would fit in the transmit buffer
            if (comm_get_txspace(out_chan) > ((uint16_t)msg->len) + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
                _mavlink_resend_uart(out_chan, msg);
            }
        }
        }
        break;

    } // end switch
} // end handle mavlink

/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
static void mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs[0].initialised || in_mavlink_delay) return;

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
        notify.update();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs_send_text_P(SEVERITY_LOW, PSTR("Initialising APM..."));
    }
    check_usb_mux();

    in_mavlink_delay = false;
}

/*
 *  send a message on both GCS links
 */
static void gcs_send_message(enum ap_message id)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_message(id);
        }
    }
}

/*
 *  send data streams in the given rate range on both links
 */
static void gcs_data_stream_send(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].data_stream_send();
        }
    }
}

/*
 *  look for incoming commands on the GCS links
 */
static void gcs_update(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].update();
        }
    }
}

static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_text_P(severity, str);
    }
    }
#if LOGGING_ENABLED == ENABLED
    DataFlash.Log_Write_Message_P(str);
#endif
}

/*
 *  send a low priority formatted message to the GCS
 *  only one fits in the queue, so if you send more than one before the
 *  last one gets into the serial buffer then the old one will be lost
 */
void gcs_send_text_fmt(const prog_char_t *fmt, ...)
{
    va_list arg_list;
    gcs[0].pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)gcs[0].pending_status.text,
            sizeof(gcs[0].pending_status.text), fmt, arg_list);
    va_end(arg_list);
#if LOGGING_ENABLED == ENABLED
    DataFlash.Log_Write_Message(gcs[0].pending_status.text);
#endif
    mavlink_send_message(MAVLINK_COMM_0, MSG_STATUSTEXT, 0);
    for (uint8_t i=1; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].pending_status = gcs[0].pending_status;
            mavlink_send_message((mavlink_channel_t)i, MSG_STATUSTEXT, 0);
        }
    }
}


/**
   retry any deferred messages
 */
static void gcs_retry_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
}
#line 1 "./APMRover2/Log.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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
		PLOG(SONAR);
		PLOG(COMPASS);
		PLOG(CAMERA);
		PLOG(STEERING);
		#undef PLOG
	}

	cliSerial->println();

    DataFlash.ListAvailableLogs(cliSerial);
	return(true);
}

static int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log;
    uint16_t dump_log_start;
    uint16_t dump_log_end;
    uint16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = DataFlash.find_last_log();

    if (dump_log == -2) {
        DataFlash.DumpPageInfo(cliSerial);
        return(-1);
    } else if (dump_log <= 0) {
        cliSerial->printf_P(PSTR("dumping all\n"));
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2)
               || ((uint16_t)dump_log > last_log_num))
    {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log, dump_log_start, dump_log_end);
    return 0;
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
		TARG(SONAR);
		TARG(COMPASS);
		TARG(CAMERA);
		TARG(STEERING);
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

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint32_t loop_time;
    uint16_t main_loop_count;
    uint32_t g_dt_max;
    uint8_t  renorm_count;
    uint8_t  renorm_blowup;
    int16_t  gyro_drift_x;
    int16_t  gyro_drift_y;
    int16_t  gyro_drift_z;
    uint8_t  i2c_lockup_count;
    uint16_t ins_error_count;
};

// Write a performance monitoring packet. Total length : 19 bytes
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        time_ms         : millis(),
        loop_time       : millis()- perf_mon_timer,
        main_loop_count : mainLoop_count,
        g_dt_max        : G_Dt_max,
        renorm_count    : ahrs.renorm_range_count,
        renorm_blowup   : ahrs.renorm_blowup_count,
        gyro_drift_x    : (int16_t)(ahrs.get_gyro_drift().x * 1000),
        gyro_drift_y    : (int16_t)(ahrs.get_gyro_drift().y * 1000),
        gyro_drift_z    : (int16_t)(ahrs.get_gyro_drift().z * 1000),
        i2c_lockup_count: hal.i2c->lockup_count(),
        ins_error_count  : ins.error_count()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Cmd {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
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
static void Log_Write_Cmd(uint8_t num, const struct Location *wp)
{
    struct log_Cmd pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CMD_MSG),
        time_ms             : millis(),
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

struct PACKED log_Camera {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint32_t gps_time;
    uint16_t gps_week;
    int32_t  latitude;
    int32_t  longitude;
    int16_t  roll;
    int16_t  pitch;
    uint16_t yaw;
};

// Write a Camera packet. Total length : 26 bytes
static void Log_Write_Camera()
{
#if CAMERA == ENABLED
    struct log_Camera pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CAMERA_MSG),
        time_ms     : millis(),
        gps_time    : g_gps->time_week_ms,
        gps_week    : g_gps->time_week,
        latitude    : current_loc.lat,
        longitude   : current_loc.lng,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#endif
}

struct PACKED log_Steering {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    float demanded_accel;
    float achieved_accel;
};

// Write a steering packet
static void Log_Write_Steering()
{
    struct log_Steering pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STEERING_MSG),
        time_ms        : hal.scheduler->millis(),
        demanded_accel : lateral_acceleration,
        achieved_accel : g_gps->ground_speed_cm * 0.01f * ins.get_gyro().z,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t startup_type;
    uint8_t command_total;
};

static void Log_Write_Startup(uint8_t type)
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG),
        time_ms         : millis(),
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

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t steer_out;
    int16_t roll;
    int16_t pitch;
    int16_t throttle_out;
    float accel_y;
};

// Write a control tuning packet. Total length : 22 bytes
static void Log_Write_Control_Tuning()
{
    Vector3f accel = ins.get_accel();
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CTUN_MSG),
        time_ms         : millis(),
        steer_out       : (int16_t)channel_steer->servo_out,
        roll            : (int16_t)ahrs.roll_sensor,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)channel_throttle->servo_out,
        accel_y         : accel.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint16_t yaw;
    float    wp_distance;
    uint16_t target_bearing_cd;
    uint16_t nav_bearing_cd;
    int8_t   throttle;
};

// Write a navigation tuning packet. Total length : 18 bytes
static void Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_ms             : millis(),
        yaw                 : (uint16_t)ahrs.yaw_sensor,
        wp_distance         : wp_distance,
        target_bearing_cd   : (uint16_t)nav_controller->target_bearing_cd(),
        nav_bearing_cd      : (uint16_t)nav_controller->nav_bearing_cd(),
        throttle            : (int8_t)(100 * channel_throttle->norm_output())
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
};


// Write an attitude packet
static void Log_Write_Attitude()
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_ms : millis(),
        roll    : (int16_t)ahrs.roll_sensor,
        pitch   : (int16_t)ahrs.pitch_sensor,
        yaw     : (uint16_t)ahrs.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Mode {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t mode;
    uint8_t mode_num;
};

// Write a mode packet
static void Log_Write_Mode()
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        time_ms         : millis(),
        mode            : (uint8_t)control_mode,
        mode_num        : (uint8_t)control_mode
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}


struct PACKED log_Sonar {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    float    lateral_accel;
    uint16_t sonar1_distance;
    uint16_t sonar2_distance;
    uint16_t detected_count;
    int8_t   turn_angle;
    uint16_t turn_time;
    uint16_t ground_speed;
    int8_t   throttle;
};

// Write a sonar packet
static void Log_Write_Sonar()
{
    uint16_t turn_time = 0;
    if (obstacle.turn_angle != 0) {
        turn_time = hal.scheduler->millis() - obstacle.detected_time_ms;
    }
    struct log_Sonar pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SONAR_MSG),
        time_ms         : millis(),
        lateral_accel   : lateral_acceleration,
        sonar1_distance : (uint16_t)sonar.distance_cm(),
        sonar2_distance : (uint16_t)sonar2.distance_cm(),
        detected_count  : obstacle.detected_count,
        turn_angle      : (int8_t)obstacle.turn_angle,
        turn_time       : turn_time,
        ground_speed    : (uint16_t)(ground_speed*100),
        throttle        : (int8_t)(100 * channel_throttle->norm_output())
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Current {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t throttle_in;
    int16_t battery_voltage;
    int16_t current_amps;
    uint16_t board_voltage;
    float   current_total;
};

static void Log_Write_Current()
{
    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        time_ms                 : millis(),
        throttle_in             : channel_throttle->control_in,
        battery_voltage         : (int16_t)(battery.voltage() * 100.0),
        current_amps            : (int16_t)(battery.current_amps() * 100.0),
        board_voltage           : board_voltage(),
        current_total           : battery.current_total_mah()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;
    int16_t motor_offset_x;
    int16_t motor_offset_y;
    int16_t motor_offset_z;
};

// Write a Compass packet. Total length : 15 bytes
static void Log_Write_Compass()
{
    const Vector3f &mag_offsets = compass.get_offsets();
    const Vector3f &mag_motor_offsets = compass.get_motor_offsets();
    const Vector3f &mag = compass.get_field();
    struct log_Compass pkt = {
        LOG_PACKET_HEADER_INIT(LOG_COMPASS_MSG),
        time_ms         : millis(),
        mag_x           : (int16_t)mag.x,
        mag_y           : (int16_t)mag.y,
        mag_z           : (int16_t)mag.z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z,
        motor_offset_x  : (int16_t)mag_motor_offsets.x,
        motor_offset_y  : (int16_t)mag_motor_offsets.y,
        motor_offset_z  : (int16_t)mag_motor_offsets.z
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#if COMPASS_MAX_INSTANCES > 1
    if (compass.get_count() > 1) {
        const Vector3f &mag2_offsets = compass.get_offsets(1);
        const Vector3f &mag2_motor_offsets = compass.get_motor_offsets(1);
        const Vector3f &mag2 = compass.get_field(1);
        struct log_Compass pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_COMPASS2_MSG),
            time_ms         : millis(),
            mag_x           : (int16_t)mag2.x,
            mag_y           : (int16_t)mag2.y,
            mag_z           : (int16_t)mag2.z,
            offset_x        : (int16_t)mag2_offsets.x,
            offset_y        : (int16_t)mag2_offsets.y,
            offset_z        : (int16_t)mag2_offsets.z,
            motor_offset_x  : (int16_t)mag2_motor_offsets.x,
            motor_offset_y  : (int16_t)mag2_motor_offsets.y,
            motor_offset_z  : (int16_t)mag2_motor_offsets.z
        };
        DataFlash.WriteBlock(&pkt2, sizeof(pkt2));
    }
#endif
}


static void Log_Write_RC(void)
{
    DataFlash.Log_Write_RCIN();
    DataFlash.Log_Write_RCOUT();
}

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
    { LOG_ATTITUDE_MSG, sizeof(log_Attitude),       
      "ATT", "IccC",        "TimeMS,Roll,Pitch,Yaw" },
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "IIHIBBhhhBH", "TimeMS,LTime,MLC,gDt,RNCnt,RNBl,GDx,GDy,GDz,I2CErr,INSErr" },
    { LOG_CMD_MSG, sizeof(log_Cmd),                 
      "CMD", "IBBBBBeLL",   "TimeMS,CTot,CNum,CId,COpt,Prm1,Alt,Lat,Lng" },
    { LOG_CAMERA_MSG, sizeof(log_Camera),                 
      "CAM", "IIHLLeccC",   "TimeMS,GPSTime,GPSWeek,Lat,Lng,Alt,Roll,Pitch,Yaw" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "IBB",        "TimeMS,SType,CTot" },
    { LOG_CTUN_MSG, sizeof(log_Control_Tuning),     
      "CTUN", "Ihcchf",     "TimeMS,Steer,Roll,Pitch,ThrOut,AccY" },
    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),         
      "NTUN", "IHfHHb",     "TimeMS,Yaw,WpDist,TargBrg,NavBrg,Thr" },
    { LOG_SONAR_MSG, sizeof(log_Sonar),             
      "SONR", "IfHHHbHCb",  "TimeMS,LatAcc,S1Dist,S2Dist,DCnt,TAng,TTim,Spd,Thr" },
    { LOG_CURRENT_MSG, sizeof(log_Current),             
      "CURR", "IhhhHf",     "TimeMS,Thr,Volt,Curr,Vcc,CurrTot" },
    { LOG_MODE_MSG, sizeof(log_Mode),             
      "MODE", "IMB",        "TimeMS,Mode,ModeNum" },
    { LOG_COMPASS_MSG, sizeof(log_Compass),             
      "MAG", "Ihhhhhhhhh",  "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
    { LOG_COMPASS2_MSG, sizeof(log_Compass),             
      "MAG2", "Ihhhhhhhhh",   "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
    { LOG_STEERING_MSG, sizeof(log_Steering),             
      "STER", "Iff",   "TimeMS,Demanded,Achieved" },
};


// Read the DataFlash log memory : Packet Parser
static void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page)
{
    cliSerial->printf_P(PSTR("\n" FIRMWARE_STRING
                             "\nFree RAM: %u\n"),
                        (unsigned)hal.util->available_memory());

    cliSerial->println_P(PSTR(HAL_BOARD_NAME));

	DataFlash.LogReadProcess(log_num, start_page, end_page, 
                             print_mode,
                             cliSerial);
}

// start a new log
static void start_logging() 
{
    in_mavlink_delay = true;
    DataFlash.StartNewLog();
    in_mavlink_delay = false;
    DataFlash.Log_Write_Message_P(PSTR(FIRMWARE_STRING));

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
    DataFlash.Log_Write_Message_P(PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif

    // write system identifier as well if available
    char sysid[40];
    if (hal.util->get_system_id(sysid)) {
        DataFlash.Log_Write_Message(sysid);
    }
}

#else // LOGGING_ENABLED

// dummy functions
static void Log_Write_Startup(uint8_t type) {}
static void Log_Write_Cmd(uint8_t num, const struct Location *wp) {}
static void Log_Write_Current() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_Performance() {}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
static void Log_Write_Control_Tuning() {}
static void Log_Write_Sonar() {}
static void Log_Write_Mode() {}
static void Log_Write_Attitude() {}
static void Log_Write_Compass() {}
static void start_logging() {}
static void Log_Write_RC(void) {}

#endif // LOGGING_ENABLED

#line 1 "./APMRover2/Parameters.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  ArduPlane parameter definitions
*/

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value:def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info:class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info:class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, &v, {group_info : class::var_info} }

const AP_Param::Info var_info[] PROGMEM = {
	GSCALAR(format_version,         "FORMAT_VERSION",   1),
	GSCALAR(software_type,          "SYSID_SW_TYPE",    Parameters::k_software_type),

	// misc
    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: Two byte bitmap of log types to enable in dataflash
    // @Values: 0:Disabled,3950:Default,4078:Default+IMU
    // @User: Advanced
	GSCALAR(log_bitmask,            "LOG_BITMASK",      DEFAULT_LOG_BITMASK),
	GSCALAR(num_resets,             "SYS_NUM_RESETS",   0),

    // @Param: RST_SWITCH_CH
    // @DisplayName: Reset Switch Channel
    // @Description: RC channel to use to reset to last flight mode	after geofence takeover.
    // @User: Advanced
	GSCALAR(reset_switch_chan,      "RST_SWITCH_CH",    0),

    // @Param: INITIAL_MODE
    // @DisplayName: Initial driving mode
    // @Description: This selects the mode to start in on boot. This is useful for when you want to start in AUTO mode on boot without a receiver. Usuallly used in combination with when AUTO_TRIGGER_PIN or AUTO_KICKSTART.
    // @Values: 0:MANUAL,2:LEARNING,3:STEERING,4:HOLD,10:AUTO,11:RTL,15:GUIDED
    // @User: Advanced
	GSCALAR(initial_mode,        "INITIAL_MODE",     MANUAL),

    // @Param: RSSI_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: This selects an analog pin for the receiver RSSI voltage. It assumes the voltage is 5V for max rssi, 0V for minimum
    // @Values: -1:Disabled, 0:A0, 1:A1, 13:A13
    // @User: Standard
    GSCALAR(rssi_pin,            "RSSI_PIN",         -1),

    // @Param: SYSID_THIS_MAV
    // @DisplayName: MAVLink system ID
    // @Description: ID used in MAVLink protocol to identify this vehicle
    // @Range: 1 255
    // @User: Advanced
	GSCALAR(sysid_this_mav,         "SYSID_THISMAV",    MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: MAVLink ground station ID
    // @Description: ID used in MAVLink protocol to identify the controlling ground station
    // @Range: 1 255
    // @User: Advanced
	GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",      255),

    // @Param: SERIAL0_BAUD
    // @DisplayName: USB Console Baud Rate
    // @Description: The baud rate used on the USB console
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
	GSCALAR(serial0_baud,           "SERIAL0_BAUD",     115),

    // @Param: SERIAL1_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the first telemetry port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
	GSCALAR(serial1_baud,           "SERIAL1_BAUD",     57),

#if MAVLINK_COMM_NUM_BUFFERS > 2
    // @Param: SERIAL2_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the second telemetry port (where available)
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
	GSCALAR(serial2_baud,           "SERIAL2_BAUD",     57),
#endif

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay 
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Standard
    // @Units: seconds
    // @Range: 0 10
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: SKIP_GYRO_CAL
    // @DisplayName: Skip gyro calibration
    // @Description: When enabled this tells the APM to skip the normal gyroscope calibration at startup, and instead use the saved gyro calibration from the last flight. You should only enable this if you are careful to check that your aircraft has good attitude control before flying, as some boards may have significantly different gyro calibration between boots, especially if the temperature changes a lot. If gyro calibration is skipped then APM relies on using the gyro drift detection code to get the right gyro calibration in the few minutes after it boots. This option is mostly useful where the requirement to hold the vehicle still while it is booting is a significant problem.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(skip_gyro_cal,           "SKIP_GYRO_CAL",   0),

    // @Param: MAG_ENABLED
    // @DisplayName: Magnetometer (compass) enabled
    // @Description: This should be set to 1 if a compass is installed
    // @User: Standard
    // @Values: 0:Disabled,1:Enabled
	GSCALAR(compass_enabled,        "MAG_ENABLE",       MAGNETOMETER),

	// @Param: AUTO_TRIGGER_PIN
	// @DisplayName: Auto mode trigger pin
	// @Description: pin number to use to enable the throttle in auto mode. If set to -1 then don't use a trigger, otherwise this is a pin number which if held low in auto mode will enable the motor to run. If the switch is released while in AUTO then the motor will stop again. This can be used in combination with INITIAL_MODE to give a 'press button to start' rover with no receiver.
	// @Values: -1:Disabled,0-8:TiggerPin
	// @User: standard
	GSCALAR(auto_trigger_pin,        "AUTO_TRIGGER_PIN", -1),

	// @Param: AUTO_KICKSTART
	// @DisplayName: Auto mode trigger kickstart acceleration
	// @Description: X acceleration in meters/second/second to use to trigger the motor start in auto mode. If set to zero then auto throttle starts immediately when the mode switch happens, otherwise the rover waits for the X acceleration to go above this value before it will start the motor
	// @Units: m/s/s
	// @Range: 0 20
	// @Increment: 0.1
	// @User: standard
	GSCALAR(auto_kickstart,          "AUTO_KICKSTART", 0.0f),

    // @Param: CRUISE_SPEED
    // @DisplayName: Target cruise speed in auto modes
    // @Description: The target speed in auto missions.
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
	GSCALAR(speed_cruise,        "CRUISE_SPEED",    5),

    // @Param: SPEED_TURN_GAIN
    // @DisplayName: Target speed reduction while turning
    // @Description: The percentage to reduce the throttle while turning. If this is 100% then the target speed is not reduced while turning. If this is 50% then the target speed is reduced in proportion to the turn rate, with a reduction of 50% when the steering is maximally deflected.
    // @Units: percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(speed_turn_gain,    "SPEED_TURN_GAIN",  50),

    // @Param: SPEED_TURN_DIST
    // @DisplayName: Distance to turn to start reducing speed
    // @Description: The distance to the next turn at which the rover reduces its target speed by the SPEED_TURN_GAIN
    // @Units: meters
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
	GSCALAR(speed_turn_dist,    "SPEED_TURN_DIST",  2.0f),

    // @Param: CH7_OPTION
    // @DisplayName: Channel 7 option
    // @Description: What to do use channel 7 for
    // @Values: 0:Nothing,1:LearnWaypoint
    // @User: Standard
	GSCALAR(ch7_option,             "CH7_OPTION",          CH7_OPTION),

    // @Group: RC1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
	GGROUP(rc_1,                    "RC1_", RC_Channel),

    // @Group: RC2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
	GGROUP(rc_2,                    "RC2_", RC_Channel_aux),

    // @Group: RC3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
	GGROUP(rc_3,                    "RC3_", RC_Channel),

    // @Group: RC4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
	GGROUP(rc_4,                    "RC4_", RC_Channel_aux),

    // @Group: RC5_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
	GGROUP(rc_5,                    "RC5_", RC_Channel_aux),

    // @Group: RC6_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
	GGROUP(rc_6,                    "RC6_", RC_Channel_aux),

    // @Group: RC7_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
	GGROUP(rc_7,                    "RC7_", RC_Channel_aux),

    // @Group: RC8_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
	GGROUP(rc_8,                    "RC8_", RC_Channel_aux),

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Group: RC9_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_9,                    "RC9_", RC_Channel_aux),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Group: RC10_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_10,                    "RC10_", RC_Channel_aux),

    // @Group: RC11_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_11,                    "RC11_", RC_Channel_aux),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Group: RC12_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_12,                    "RC12_", RC_Channel_aux),
#endif

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
    // @Description: The maximum throttle setting to which the autopilot will apply. This can be used to prevent overheating a ESC or motor on an electric rover.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_max,           "THR_MAX",          THROTTLE_MAX),

    // @Param: CRUISE_THROTTLE
    // @DisplayName: Base throttle percentage in auto
    // @Description: The base throttle percentage to use in auto mode. The CRUISE_SPEED parameter controls the target speed, but the rover starts with the CRUISE_THROTTLE setting as the initial estimate for how much throttle is needed to achieve that speed. It then adjusts the throttle based on how fast the rover is actually going.
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

    // @Param: SKID_STEER_OUT
    // @DisplayName: Skid steering output
    // @Description: Set this to 1 for skid steering controlled rovers (tank track style). When enabled, servo1 is used for the left track control, servo3 is used for right track control
    // @Values: 0:Disabled, 1:SkidSteeringOutput
    // @User: Standard
	GSCALAR(skid_steer_out,          "SKID_STEER_OUT",     0),

    // @Param: SKID_STEER_IN
    // @DisplayName: Skid steering input
    // @Description: Set this to 1 for skid steering input rovers (tank track style in RC controller). When enabled, servo1 is used for the left track control, servo3 is used for right track control
    // @Values: 0:Disabled, 1:SkidSteeringInput
    // @User: Standard
	GSCALAR(skid_steer_in,           "SKID_STEER_IN",     0),

    // @Param: FS_ACTION
    // @DisplayName: Failsafe Action
    // @Description: What to do on a failsafe event
    // @Values: 0:Nothing,1:RTL,2:HOLD
    // @User: Standard
	GSCALAR(fs_action,    "FS_ACTION",     2),

    // @Param: FS_TIMEOUT
    // @DisplayName: Failsafe timeout
    // @Description: How long a failsafe event need to happen for before we trigger the failsafe action
	// @Units: seconds
    // @User: Standard
	GSCALAR(fs_timeout,    "FS_TIMEOUT",     5),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel to a low value. This can be used to detect the RC transmitter going out of range. Failsafe will be triggered when the throttle channel goes below the FS_THR_VALUE for FS_TIMEOUT seconds.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
	GSCALAR(fs_throttle_enabled,    "FS_THR_ENABLE",     1),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers.
    // @Range: 925 1100
    // @Increment: 1
    // @User: Standard
	GSCALAR(fs_throttle_value,      "FS_THR_VALUE",     910),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: GCS failsafe enable
    // @Description: Enable ground control station telemetry failsafe. When enabled the Rover will execute the FS_ACTION when it fails to receive MAVLink heartbeat packets for FS_TIMEOUT seconds.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
	GSCALAR(fs_gcs_enabled, "FS_GCS_ENABLE",   0),

	// @Param: SONAR_TRIGGER_CM
	// @DisplayName: Sonar trigger distance
	// @Description: The distance from an obstacle in centimeters at which the sonar triggers a turn to avoid the obstacle
	// @Units: centimeters
    // @Range: 0 1000
    // @Increment: 1
	// @User: Standard
	GSCALAR(sonar_trigger_cm,   "SONAR_TRIGGER_CM",    100),

	// @Param: SONAR_TURN_ANGLE
	// @DisplayName: Sonar trigger angle
	// @Description: The course deviation in degrees to apply while avoiding an obstacle detected with the sonar. A positive number means to turn right, and a negative angle means to turn left.
	// @Units: centimeters
    // @Range: -45 45
    // @Increment: 1
	// @User: Standard
	GSCALAR(sonar_turn_angle,   "SONAR_TURN_ANGLE",    45),

	// @Param: SONAR_TURN_TIME
	// @DisplayName: Sonar turn time
	// @Description: The amount of time in seconds to apply the SONAR_TURN_ANGLE after detecting an obstacle.
	// @Units: seconds
    // @Range: 0 100
    // @Increment: 0.1
	// @User: Standard
	GSCALAR(sonar_turn_time,    "SONAR_TURN_TIME",     1.0f),

	// @Param: SONAR_DEBOUNCE
	// @DisplayName: Sonar debounce count
	// @Description: The number of 50Hz sonar hits needed to trigger an obstacle avoidance event. If you get a lot of false sonar events then raise this number, but if you make it too large then it will cause lag in detecting obstacles, which could cause you go hit the obstacle.
    // @Range: 1 100
    // @Increment: 1
	// @User: Standard
	GSCALAR(sonar_debounce,   "SONAR_DEBOUNCE",    2),

    // @Param: LEARN_CH
    // @DisplayName: Learning channel
    // @Description: RC Channel to use for learning waypoints
    // @User: Advanced
	GSCALAR(learn_channel,    "LEARN_CH",       7),

    // @Param: MODE_CH
    // @DisplayName: Mode channel
    // @Description: RC Channel to use for driving mode control
    // @User: Advanced
	GSCALAR(mode_channel,    "MODE_CH",       MODE_CHANNEL),

    // @Param: MODE1
    // @DisplayName: Mode1
    // @Values: 0:Manual,2:LEARNING,3:STEERING,4:HOLD,10:Auto,11:RTL,15:Guided
    // @User: Standard
    // @Description: Driving mode for switch position 1 (910 to 1230 and above 2049)
	GSCALAR(mode1,           "MODE1",         MODE_1),

    // @Param: MODE2
    // @DisplayName: Mode2
    // @Description: Driving mode for switch position 2 (1231 to 1360)
    // @Values: 0:Manual,2:LEARNING,3:STEERING,4:HOLD,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode2,           "MODE2",         MODE_2),

    // @Param: MODE3
    // @DisplayName: Mode3
    // @Description: Driving mode for switch position 3 (1361 to 1490)
    // @Values: 0:Manual,2:LEARNING,3:STEERING,4:HOLD,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode3,           "MODE3",         MODE_3),

    // @Param: MODE4
    // @DisplayName: Mode4
    // @Description: Driving mode for switch position 4 (1491 to 1620)
    // @Values: 0:Manual,2:LEARNING,3:STEERING,4:HOLD,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode4,           "MODE4",         MODE_4),

    // @Param: MODE5
    // @DisplayName: Mode5
    // @Description: Driving mode for switch position 5 (1621 to 1749)
    // @Values: 0:Manual,2:LEARNING,3:STEERING,4:HOLD,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode5,           "MODE5",         MODE_5),

    // @Param: MODE6
    // @DisplayName: Mode6
    // @Description: Driving mode for switch position 6 (1750 to 2049)
    // @Values: 0:Manual,2:LEARNING,3:STEERING,4:HOLD,10:Auto,11:RTL,15:Guided
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

    // @Param: TURN_MAX_G
    // @DisplayName: Turning maximum G force
    // @Description: The maximum turning acceleration (in units of gravities) that the rover can handle while remaining stable. The navigation code will keep the lateral acceleration below this level to avoid rolling over or slipping the wheels in turns
    // @Units: gravities
    // @Range: 0.2 10
    // @Increment: 0.1
    // @User: Standard
	GSCALAR(turn_max_g,             "TURN_MAX_G",      2.0f),

    // @Group: STEER2SRV_
    // @Path: ../libraries/APM_Control/AP_SteerController.cpp
	GOBJECT(steerController,        "STEER2SRV_",   AP_SteerController),

	GGROUP(pidSpeedThrottle,        "SPEED2THR_", PID),

	// variables not in the g class which contain EEPROM saved variables

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/Compass.cpp
	GOBJECT(compass,                "COMPASS_",	Compass),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: RELAY_
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY_", AP_Relay),

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap,                 "RCMAP_",         RCMapper),

    // @Group: SR0_
    // @Path: GCS_Mavlink.pde
    GOBJECTN(gcs[0], gcs0,        "SR0_",     GCS_MAVLINK),

    // @Group: SR1_
    // @Path: GCS_Mavlink.pde
    GOBJECTN(gcs[1],  gcs1,       "SR1_",     GCS_MAVLINK),

#if MAVLINK_COMM_NUM_BUFFERS > 2
    // @Group: SR2_
    // @Path: GCS_Mavlink.pde
    GOBJECTN(gcs[2],  gcs2,       "SR2_",     GCS_MAVLINK),
#endif

    // @Group: NAVL1_
    // @Path: ../libraries/AP_L1_Control/AP_L1_Control.cpp
    GOBJECT(L1_controller,         "NAVL1_",   AP_L1_Control),

    // @Group: SONAR_
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder_analog.cpp
    GOBJECT(sonar,                  "SONAR_", AP_RangeFinder_analog),

    // @Group: SONAR2_
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder_analog.cpp
    GOBJECT(sonar2,                 "SONAR2_", AP_RangeFinder_analog),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                            "INS_", AP_InertialSensor),

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL),
#endif

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,                  "CAM_", AP_Camera),
#endif

#if MOUNT == ENABLED
    // @Group: MNT_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT_", AP_Mount),
#endif

    // @Group: BATT_
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT_",       AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

	AP_VAREND
};

/*
  This is a conversion table from old parameter values to new
  parameter names. The startup code looks for saved values of the old
  parameters and will copy them across to the new parameters if the
  new parameter does not yet have a saved value. It then saves the new
  value.

  Note that this works even if the old parameter has been removed. It
  relies on the old k_param index not being removed

  The second column below is the index in the var_info[] table for the
  old object. This should be zero for top level parameters.
 */
const AP_Param::ConversionInfo conversion_table[] PROGMEM = {
    { Parameters::k_param_battery_monitoring, 0,      AP_PARAM_INT8,  "BATT_MONITOR" },
    { Parameters::k_param_battery_volt_pin,   0,      AP_PARAM_INT8,  "BATT_VOLT_PIN" },
    { Parameters::k_param_battery_curr_pin,   0,      AP_PARAM_INT8,  "BATT_CURR_PIN" },
    { Parameters::k_param_volt_div_ratio,     0,      AP_PARAM_FLOAT, "BATT_VOLT_MULT" },
    { Parameters::k_param_curr_amp_per_volt,  0,      AP_PARAM_FLOAT, "BATT_AMP_PERVOLT" },
    { Parameters::k_param_pack_capacity,      0,      AP_PARAM_INT32, "BATT_CAPACITY" },
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

    // set a lower default filter frequency for rovers, due to very
    // high vibration levels on rough surfaces
    ins.set_default_filter(5);

    // set a more reasonable default NAVL1_PERIOD for rovers
    L1_controller.set_default_period(8);
}
#line 1 "./APMRover2/Steering.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*****************************************
* Throttle slew limit
*****************************************/
static void throttle_slew_limit(int16_t last_throttle)
{
    // if slew limit rate is set to zero then do not slew limit
    if (g.throttle_slewrate) {                   
        // limit throttle change by the given percentage per second
        float temp = g.throttle_slewrate * G_Dt * 0.01f * fabsf(channel_throttle->radio_max - channel_throttle->radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        channel_throttle->radio_out = constrain_int16(channel_throttle->radio_out, last_throttle - temp, last_throttle + temp);
    }
}

/*
  check for triggering of start of auto mode
 */
static bool auto_check_trigger(void)
{
    // only applies to AUTO mode
    if (control_mode != AUTO) {
        return true;
    }

    // check for user pressing the auto trigger to off
    if (auto_triggered && g.auto_trigger_pin != -1 && check_digital_pin(g.auto_trigger_pin) == 1) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("AUTO triggered off"));
        auto_triggered = false;
        return false; 
    }

    // if already triggered, then return true, so you don't
    // need to hold the switch down
    if (auto_triggered) {
        return true;
    }

    if (g.auto_trigger_pin == -1 && g.auto_kickstart == 0.0f) {
        // no trigger configured - let's go!
        auto_triggered = true;
        return true;
    }
 
    if (g.auto_trigger_pin != -1 && check_digital_pin(g.auto_trigger_pin) == 0) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Triggered AUTO with pin"));
        auto_triggered = true;
        return true;            
    }

    if (g.auto_kickstart != 0.0f) {
        float xaccel = ins.get_accel().x;
        if (xaccel >= g.auto_kickstart) {
            gcs_send_text_fmt(PSTR("Triggered AUTO xaccel=%.1f"), xaccel);
            auto_triggered = true;
            return true;            
        }
    }

    return false;   
}


/*
  calculate the throtte for auto-throttle modes
 */
static void calc_throttle(float target_speed)
{  
    if (!auto_check_trigger()) {
        channel_throttle->servo_out = g.throttle_min.get();
        return;
    }

    float throttle_base = (fabsf(target_speed) / g.speed_cruise) * g.throttle_cruise;
    int throttle_target = throttle_base + throttle_nudge;  

    /*
      reduce target speed in proportion to turning rate, up to the
      SPEED_TURN_GAIN percentage.
    */
    float steer_rate = fabsf(lateral_acceleration / (g.turn_max_g*GRAVITY_MSS));
    steer_rate = constrain_float(steer_rate, 0.0, 1.0);
    float reduction = 1.0 - steer_rate*(100 - g.speed_turn_gain)*0.01;
    
    if (control_mode >= AUTO && wp_distance <= g.speed_turn_dist) {
        // in auto-modes we reduce speed when approaching waypoints
        float reduction2 = 1.0 - (100-g.speed_turn_gain)*0.01*((g.speed_turn_dist - wp_distance)/g.speed_turn_dist);
        if (reduction2 < reduction) {
            reduction = reduction2;
        }
    }
    
    // reduce the target speed by the reduction factor
    target_speed *= reduction;

    groundspeed_error = fabsf(target_speed) - ground_speed; 
    
    throttle = throttle_target + (g.pidSpeedThrottle.get_pid(groundspeed_error * 100) / 100);

    // also reduce the throttle by the reduction factor. This gives a
    // much faster response in turns
    throttle *= reduction;

    if (in_reverse) {
        channel_throttle->servo_out = constrain_int16(-throttle, -g.throttle_max, -g.throttle_min);
    } else {
        channel_throttle->servo_out = constrain_int16(throttle, g.throttle_min, g.throttle_max);
    }
}

/*****************************************
 * Calculate desired turn angles (in medium freq loop)
 *****************************************/

static void calc_lateral_acceleration()
{
    switch (control_mode) {
    case AUTO:
        nav_controller->update_waypoint(prev_WP, next_WP);
        break;

    case RTL:
    case GUIDED:
    case STEERING:
        nav_controller->update_waypoint(current_loc, next_WP);
        break;
    default:
        return;
    }

	// Calculate the required turn of the wheels

    // negative error = left turn
	// positive error = right turn
    lateral_acceleration = nav_controller->lateral_acceleration();
}

/*
  calculate steering angle given lateral_acceleration
 */
static void calc_nav_steer()
{
    float speed = g_gps->ground_speed_cm * 0.01f;

    if (speed < 1.0f) {
        // gps speed isn't very accurate at low speed
        speed = 1.0f;
    }

    // add in obstacle avoidance
    lateral_acceleration += (obstacle.turn_angle/45.0f) * g.turn_max_g;

    // constrain to max G force
    lateral_acceleration = constrain_float(lateral_acceleration, -g.turn_max_g*GRAVITY_MSS, g.turn_max_g*GRAVITY_MSS);

    channel_steer->servo_out = steerController.get_steering_out_lat_accel(lateral_acceleration);
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void set_servos(void)
{
    int16_t last_throttle = channel_throttle->radio_out;

	if ((control_mode == MANUAL || control_mode == LEARNING) &&
        (g.skid_steer_out == g.skid_steer_in)) {
        // do a direct pass through of radio values
        channel_steer->radio_out       = channel_steer->read();
        channel_throttle->radio_out    = channel_throttle->read();
        if (failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
            // suppress throttle if in failsafe and manual
            channel_throttle->radio_out = channel_throttle->radio_trim;
        }
	} else {       
        channel_steer->calc_pwm();
        if (in_reverse) {
            channel_throttle->servo_out = constrain_int16(channel_throttle->servo_out, 
                                                          -g.throttle_max,
                                                          -g.throttle_min);
        } else {
            channel_throttle->servo_out = constrain_int16(channel_throttle->servo_out, 
                                                          g.throttle_min.get(), 
                                                          g.throttle_max.get());
        }

        if ((failsafe.bits & FAILSAFE_EVENT_THROTTLE) && control_mode < AUTO) {
            // suppress throttle if in failsafe
            channel_throttle->servo_out = 0;
        }

        // convert 0 to 100% into PWM
        channel_throttle->calc_pwm();

        // limit throttle movement speed
        throttle_slew_limit(last_throttle);

        if (g.skid_steer_out) {
            // convert the two radio_out values to skid steering values
            /*
              mixing rule:
              steering = motor1 - motor2
              throttle = 0.5*(motor1 + motor2)
              motor1 = throttle + 0.5*steering
              motor2 = throttle - 0.5*steering
            */          
            float steering_scaled = channel_steer->norm_output();
            float throttle_scaled = channel_throttle->norm_output();
            float motor1 = throttle_scaled + 0.5*steering_scaled;
            float motor2 = throttle_scaled - 0.5*steering_scaled;
            channel_steer->servo_out = 4500*motor1;
            channel_throttle->servo_out = 100*motor2;
            channel_steer->calc_pwm();
            channel_throttle->calc_pwm();
        }
    }


#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
	// send values to the PWM timers for output
	// ----------------------------------------
    channel_steer->output(); 
    channel_throttle->output();

	// Route configurable aux. functions to their respective servos
	g.rc_2.output_ch(CH_2);
	g.rc_4.output_ch(CH_4);
	g.rc_5.output_ch(CH_5);
	g.rc_6.output_ch(CH_6);
	g.rc_7.output_ch(CH_7);
	g.rc_8.output_ch(CH_8);
 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    g.rc_9.output_ch(CH_9);
 #endif
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4
    g.rc_10.output_ch(CH_10);
    g.rc_11.output_ch(CH_11);
 #endif
 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    g.rc_12.output_ch(CH_12);
 #endif

#endif
}


#line 1 "./APMRover2/commands.pde"
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
	if ((temp.options & MASK_OPTIONS_RELATIVE_ALT) && i != 0){
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
static void set_next_WP(const struct Location *wp)
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
	wp_totalDistance 	= get_distance(current_loc, next_WP);
	wp_distance 		= wp_totalDistance;
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
	wp_totalDistance 	= get_distance(current_loc, next_WP);
	wp_distance 		= wp_totalDistance;
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
    gps_base_alt    = max(g_gps->altitude_cm, 0);
    home.alt        = g_gps->altitude_cm;
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
    g.pidSpeedThrottle.reset_I();
    prev_WP = current_loc;
    nav_command_ID = NO_COMMAND;
    nav_command_index = 0;
    process_next_command();
}
#line 1 "./APMRover2/commands_logic.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
static void
handle_process_nav_cmd()
{
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
            ServoRelayEvents.do_set_servo(next_nonnav_command.p1, next_nonnav_command.alt);
            break;

    	case MAV_CMD_DO_SET_RELAY:
            ServoRelayEvents.do_set_relay(next_nonnav_command.p1, next_nonnav_command.alt);
            break;

    	case MAV_CMD_DO_REPEAT_SERVO:
            ServoRelayEvents.do_repeat_servo(next_nonnav_command.p1, next_nonnav_command.alt,
                                             next_nonnav_command.lat, next_nonnav_command.lng);
            break;

    	case MAV_CMD_DO_REPEAT_RELAY:
            ServoRelayEvents.do_repeat_relay(next_nonnav_command.p1, next_nonnav_command.alt,
                                             next_nonnav_command.lat);
            break;

#if CAMERA == ENABLED
    case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
        do_take_picture();
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        camera.set_trigger_distance(next_nonnav_command.alt);
        break;
#endif

#if MOUNT == ENABLED
		// Sets the region of interest (ROI) for a sensor set or the
		// vehicle itself. This can then be used by the vehicles control
		// system to control the vehicle attitude and the attitude of various
		// devices such as cameras.
		//    |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
		case MAV_CMD_DO_SET_ROI:
#if 0
            // not supported yet
			camera_mount.set_roi_cmd();
#endif
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
	gcs_send_text_fmt(PSTR("No commands - setting HOLD"));
    set_mode(HOLD);
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
    if ((wp_distance > 0) && (wp_distance <= g.waypoint_radius)) {
        gcs_send_text_fmt(PSTR("Reached Waypoint #%i dist %um"),
                          (unsigned)nav_command_index,
                          (unsigned)get_distance(current_loc, next_WP));
        return true;
    }

    // have we gone past the waypoint?
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_fmt(PSTR("Passed Waypoint #%i dist %um"),
                          (unsigned)nav_command_index,
                          (unsigned)get_distance(current_loc, next_WP));
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
	}

    // have we gone past the waypoint?
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_fmt(PSTR("Reached Home dist %um"),
                          (unsigned)get_distance(current_loc, next_WP));
        return true;
    }

    return false;
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
			if (next_nonnav_command.alt > 0) {
				g.speed_cruise.set(next_nonnav_command.alt);
                gcs_send_text_fmt(PSTR("Cruise speed: %.1f"), g.speed_cruise.get());
            }
			break;
	}

	if (next_nonnav_command.lat > 0) {
		g.throttle_cruise.set(next_nonnav_command.lat);
        gcs_send_text_fmt(PSTR("Cruise throttle: %.1f"), g.throttle_cruise.get());
    }
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

// do_take_picture - take a picture with the camera library
static void do_take_picture()
{
#if CAMERA == ENABLED
    camera.trigger_pic();
    if (should_log(MASK_LOG_CAMERA)) {
        Log_Write_Camera();
    }
#endif
}
#line 1 "./APMRover2/commands_process.pde"
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


#line 1 "./APMRover2/compat.pde"


static void delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

static void mavlink_delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

static uint32_t millis()
{
    return hal.scheduler->millis();
}

static uint32_t micros()
{
    return hal.scheduler->micros();
}

#line 1 "./APMRover2/control_modes.pde"
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

		// reset speed integrator
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
		if (channel_learn->radio_in > CH_7_PWM_TRIGGER) {
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
                    if (channel_steer->control_in > 3000) {
						// if roll is full right store the current location as home
                        init_home();
                    }
                    CH7_wp_index = 1;     
					return;
				} else if (control_mode == LEARNING || control_mode == STEERING) {    
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

#line 1 "./APMRover2/events.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


static void update_events(void)
{
    ServoRelayEvents.update_events();
}
#line 1 "./APMRover2/failsafe.pde"
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
static void failsafe_check()
{
    static uint16_t last_mainLoop_count;
    static uint32_t last_timestamp;
    static bool in_failsafe;
    uint32_t tnow = hal.scheduler->micros();

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

    if (in_failsafe && tnow - last_timestamp > 20000 && 
        channel_throttle->read() >= (uint16_t)g.fs_throttle_value) {
        // pass RC inputs to outputs every 20ms        
        last_timestamp = tnow;
        hal.rcin->clear_overrides();
        uint8_t start_ch = 0;
        for (uint8_t ch=start_ch; ch<4; ch++) {
            hal.rcout->write(ch, hal.rcin->read(ch));
        }
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_manual, true);
    }
}
#line 1 "./APMRover2/navigation.pde"
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

	// waypoint distance from rover
	// ----------------------------
	wp_distance = get_distance(current_loc, next_WP);

	if (wp_distance < 0){
		gcs_send_text_P(SEVERITY_HIGH,PSTR("<navigate> WP error - distance < 0"));
		return;
	}

	// control mode specific updates to nav_bearing
	// --------------------------------------------
	update_navigation();
}


void reached_waypoint()
{       

}

#line 1 "./APMRover2/radio.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  allow for runtime change of control channel ordering
 */
static void set_control_channels(void)
{
    channel_steer    = RC_Channel::rc_channel(rcmap.roll()-1);
    channel_throttle = RC_Channel::rc_channel(rcmap.throttle()-1);
    channel_learn    = RC_Channel::rc_channel(g.learn_channel-1);

	// set rc channel ranges
	channel_steer->set_angle(SERVO_MAX);
	channel_throttle->set_angle(100);
}

static void init_rc_in()
{
	// set rc dead zones
	channel_steer->set_default_dead_zone(30);
	channel_throttle->set_default_dead_zone(30);

	//set auxiliary ranges
    update_aux();
}

static void init_rc_out()
{
    for (uint8_t i=0; i<8; i++) {
        RC_Channel::rc_channel(i)->enable_out();
        RC_Channel::rc_channel(i)->output_trim();
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    servo_write(CH_9,   g.rc_9.radio_trim);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4
    servo_write(CH_10,  g.rc_10.radio_trim);
    servo_write(CH_11,  g.rc_11.radio_trim);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    servo_write(CH_12,  g.rc_12.radio_trim);
#endif
}

static void read_radio()
{
    if (!hal.rcin->valid_channels()) {
        control_failsafe(channel_throttle->radio_in);
        return;
    }

    failsafe.last_valid_rc_ms = hal.scheduler->millis();

    for (uint8_t i=0; i<8; i++) {
        RC_Channel::rc_channel(i)->set_pwm(RC_Channel::rc_channel(i)->read());
    }

	control_failsafe(channel_throttle->radio_in);

	channel_throttle->servo_out = channel_throttle->control_in;

	if (abs(channel_throttle->servo_out) > 50) {
        throttle_nudge = (g.throttle_max - g.throttle_cruise) * ((fabsf(channel_throttle->norm_input())-0.5) / 0.5);
	} else {
		throttle_nudge = 0;
	}

    if (g.skid_steer_in) {
        // convert the two radio_in values from skid steering values
        /*
          mixing rule:
          steering = motor1 - motor2
          throttle = 0.5*(motor1 + motor2)
          motor1 = throttle + 0.5*steering
          motor2 = throttle - 0.5*steering
        */          

        float motor1 = channel_steer->norm_input();
        float motor2 = channel_throttle->norm_input();
        float steering_scaled = motor1 - motor2;
        float throttle_scaled = 0.5f*(motor1 + motor2);
        int16_t steer = channel_steer->radio_trim;
        int16_t thr   = channel_throttle->radio_trim;
        if (steering_scaled > 0.0f) {
            steer += steering_scaled*(channel_steer->radio_max-channel_steer->radio_trim);
        } else {
            steer += steering_scaled*(channel_steer->radio_trim-channel_steer->radio_min);
        }
        if (throttle_scaled > 0.0f) {
            thr += throttle_scaled*(channel_throttle->radio_max-channel_throttle->radio_trim);
        } else {
            thr += throttle_scaled*(channel_throttle->radio_trim-channel_throttle->radio_min);
        }
        channel_steer->set_pwm(steer);
        channel_throttle->set_pwm(thr);
    }
}

static void control_failsafe(uint16_t pwm)
{
	if (!g.fs_throttle_enabled) {
        // no throttle failsafe
		return;
    }

	// Check for failsafe condition based on loss of GCS control
	if (rc_override_active) {
        failsafe_trigger(FAILSAFE_EVENT_RC, (millis() - failsafe.rc_override_timer) > 1500);
	} else if (g.fs_throttle_enabled) {
        bool failed = pwm < (uint16_t)g.fs_throttle_value;
        if (hal.scheduler->millis() - failsafe.last_valid_rc_ms > 2000) {
            failed = true;
        }
        failsafe_trigger(FAILSAFE_EVENT_THROTTLE, failed);
	}
}

static void trim_control_surfaces()
{
	read_radio();
	// Store control surface trim values
	// ---------------------------------
    if (channel_steer->radio_in > 1400) {
		channel_steer->radio_trim = channel_steer->radio_in;
        // save to eeprom
        channel_steer->save_eeprom();
    }
}

static void trim_radio()
{
	for (int y = 0; y < 30; y++) {
		read_radio();
	}
    trim_control_surfaces();
}
#line 1 "./APMRover2/sensors.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_sonar(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    sonar.Init(&adc);
    sonar2.Init(&adc);
#else
    sonar.Init(NULL);
    sonar2.Init(NULL);
#endif
}

// read_battery - reads battery voltage and current and invokes failsafe
// should be called at 10hz
static void read_battery(void)
{
    battery.read();
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    rssi_analog_source->set_pin(g.rssi_pin);
    float ret = rssi_analog_source->voltage_average() * 50;
    receiver_rssi = constrain_int16(ret, 0, 255);
}

// read the sonars
static void read_sonars(void)
{
    if (!sonar.enabled()) {
        // this makes it possible to disable sonar at runtime
        return;
    }

    if (sonar2.enabled()) {
        // we have two sonars
        obstacle.sonar1_distance_cm = sonar.distance_cm();
        obstacle.sonar2_distance_cm = sonar2.distance_cm();
        if (obstacle.sonar1_distance_cm <= (uint16_t)g.sonar_trigger_cm &&
            obstacle.sonar2_distance_cm <= (uint16_t)obstacle.sonar2_distance_cm)  {
            // we have an object on the left
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(PSTR("Sonar1 obstacle %u cm"),
                                  (unsigned)obstacle.sonar1_distance_cm);
            }
            obstacle.detected_time_ms = hal.scheduler->millis();
            obstacle.turn_angle = g.sonar_turn_angle;
        } else if (obstacle.sonar2_distance_cm <= (uint16_t)g.sonar_trigger_cm) {
            // we have an object on the right
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(PSTR("Sonar2 obstacle %u cm"),
                                  (unsigned)obstacle.sonar2_distance_cm);
            }
            obstacle.detected_time_ms = hal.scheduler->millis();
            obstacle.turn_angle = -g.sonar_turn_angle;
        }
    } else {
        // we have a single sonar
        obstacle.sonar1_distance_cm = sonar.distance_cm();
        obstacle.sonar2_distance_cm = 0;
        if (obstacle.sonar1_distance_cm <= (uint16_t)g.sonar_trigger_cm)  {
            // obstacle detected in front 
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(PSTR("Sonar obstacle %u cm"),
                                  (unsigned)obstacle.sonar1_distance_cm);
            }
            obstacle.detected_time_ms = hal.scheduler->millis();
            obstacle.turn_angle = g.sonar_turn_angle;
        }
    }

    Log_Write_Sonar();

    // no object detected - reset after the turn time
    if (obstacle.detected_count >= g.sonar_debounce &&
        hal.scheduler->millis() > obstacle.detected_time_ms + g.sonar_turn_time*1000) { 
        gcs_send_text_fmt(PSTR("Obstacle passed"));
        obstacle.detected_count = 0;
        obstacle.turn_angle = 0;
    }
}
#line 1 "./APMRover2/setup.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// Functions called from the setup menu
static int8_t	setup_radio			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_show			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_factory		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_flightmodes	(uint8_t argc, const Menu::arg *argv);
#if !defined( __AVR_ATmega1280__ )
static int8_t   setup_accel_scale   (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_set           (uint8_t argc, const Menu::arg *argv);
#endif
static int8_t   setup_level         (uint8_t argc, const Menu::arg *argv);
static int8_t	setup_erase			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_compass		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_declination	(uint8_t argc, const Menu::arg *argv);

// Command/function table for the setup menu
static const struct Menu::command setup_menu_commands[] PROGMEM = {
	// command			function called
	// =======        	===============
	{"reset", 			setup_factory},
	{"radio",			setup_radio},
	{"modes",			setup_flightmodes},
	{"level",			setup_level},
#if !defined( __AVR_ATmega1280__ )
    {"accel",           setup_accel_scale},
#endif
	{"compass",			setup_compass},
	{"declination",		setup_declination},
	{"show",			setup_show},
#if !defined( __AVR_ATmega1280__ )
	{"set",				setup_set},
#endif
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

#if !defined( __AVR_ATmega1280__ )
	AP_Param *param;
	ap_var_type type;

	//If a parameter name is given as an argument to show, print only that parameter
	if(argc>=2)
	{

		param=AP_Param::find(argv[1].str, &type);

		if(!param)
		{
			cliSerial->printf_P(PSTR("Parameter not found: '%s'\n"), argv[1]);
		return 0;
		}

		//Print differently for different types, and include parameter type in output.
		switch (type) {
			case AP_PARAM_INT8:
				cliSerial->printf_P(PSTR("INT8  %s: %d\n"), argv[1].str, (int)((AP_Int8 *)param)->get());
				break;
			case AP_PARAM_INT16:
				cliSerial->printf_P(PSTR("INT16 %s: %d\n"), argv[1].str, (int)((AP_Int16 *)param)->get());
				break;
			case AP_PARAM_INT32:
				cliSerial->printf_P(PSTR("INT32 %s: %ld\n"), argv[1].str, (long)((AP_Int32 *)param)->get());
				break;
			case AP_PARAM_FLOAT:
				cliSerial->printf_P(PSTR("FLOAT %s: %f\n"), argv[1].str, ((AP_Float *)param)->get());
				break;
			default:
				cliSerial->printf_P(PSTR("Unhandled parameter type for %s: %d.\n"), argv[1].str, type);
				break;
		}

		return 0;
	}
#endif

	// clear the area
	print_blanks(8);

	report_radio();
	report_batt_monitor();
	report_gains();
	report_throttle();
	report_modes();
	report_compass();

	cliSerial->printf_P(PSTR("Raw Values\n"));
	print_divider();

    AP_Param::show_all(cliSerial);

	return(0);
}


#if !defined( __AVR_ATmega1280__ )

//Set a parameter to a specified value. It will cast the value to the current type of the
//parameter and make sure it fits in case of INT8 and INT16
static int8_t setup_set(uint8_t argc, const Menu::arg *argv)
{
	int8_t value_int8;
	int16_t value_int16;

	AP_Param *param;
	enum ap_var_type p_type;

	if(argc!=3)
	{
		cliSerial->printf_P(PSTR("Invalid command. Usage: set <name> <value>\n"));
		return 0;
	}

	param = AP_Param::find(argv[1].str, &p_type);
	if(!param)
	{
		cliSerial->printf_P(PSTR("Param not found: %s\n"), argv[1].str);
		return 0;
	}

	switch(p_type)
	{
		case AP_PARAM_INT8:
			value_int8 = (int8_t)(argv[2].i);
			if(argv[2].i!=value_int8)
			{
				cliSerial->printf_P(PSTR("Value out of range for type INT8\n"));
				return 0;
			}
			((AP_Int8*)param)->set_and_save(value_int8);
			break;
		case AP_PARAM_INT16:
			value_int16 = (int16_t)(argv[2].i);
			if(argv[2].i!=value_int16)
			{
				cliSerial->printf_P(PSTR("Value out of range for type INT16\n"));
				return 0;
			}
			((AP_Int16*)param)->set_and_save(value_int16);
			break;

		//int32 and float don't need bounds checking, just use the value provoded by Menu::arg
		case AP_PARAM_INT32:
			((AP_Int32*)param)->set_and_save(argv[2].i);
			break;
		case AP_PARAM_FLOAT:
			((AP_Float*)param)->set_and_save(argv[2].f);
			break;
		default:
			cliSerial->printf_P(PSTR("Cannot set parameter of type %d.\n"), p_type);
			break;
	}

	return 0;
}
#endif


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
	cliSerial->printf_P(PSTR("\nFACTORY RESET complete - please reset board to continue"));

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


	if(channel_steer->radio_in < 500){
		while(1){
			cliSerial->printf_P(PSTR("\nNo radio; Check connectors."));
			delay(1000);
			// stop here
		}
	}

	channel_steer->radio_min 		= channel_steer->radio_in;
	channel_throttle->radio_min 	= channel_throttle->radio_in;
	g.rc_2.radio_min = g.rc_2.radio_in;
	g.rc_4.radio_min = g.rc_4.radio_in;
	g.rc_5.radio_min = g.rc_5.radio_in;
	g.rc_6.radio_min = g.rc_6.radio_in;
	g.rc_7.radio_min = g.rc_7.radio_in;
	g.rc_8.radio_min = g.rc_8.radio_in;

	channel_steer->radio_max 		= channel_steer->radio_in;
	channel_throttle->radio_max 	= channel_throttle->radio_in;
	g.rc_2.radio_max = g.rc_2.radio_in;
	g.rc_4.radio_max = g.rc_4.radio_in;
	g.rc_5.radio_max = g.rc_5.radio_in;
	g.rc_6.radio_max = g.rc_6.radio_in;
	g.rc_7.radio_max = g.rc_7.radio_in;
	g.rc_8.radio_max = g.rc_8.radio_in;

	channel_steer->radio_trim 		= channel_steer->radio_in;
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

		channel_steer->update_min_max();
		channel_throttle->update_min_max();
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
			channel_steer->save_eeprom();
			channel_throttle->save_eeprom();
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
				mode != HOLD &&
				mode != LEARNING &&
				mode != STEERING &&
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
             ins_sample_rate);
    AP_InertialSensor_UserInteractStream interact(hal.console);
    if(ins.calibrate_accel(&interact, trim_roll, trim_pitch)) {
        // reset ahrs's trim to suggested values from calibration routine
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
    return(0);
}
#endif

static int8_t
setup_level(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->println_P(PSTR("Initialising gyros"));
    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    ins.init_accel();
    ahrs.set_trim(Vector3f(0, 0, 0));
    return(0);
}

static int8_t
setup_compass(uint8_t argc, const Menu::arg *argv)
{
	if (!strcmp_P(argv[1].str, PSTR("on"))) {
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

/***************************************************************************/
// CLI reports
/***************************************************************************/

static void report_batt_monitor()
{
    //print_blanks(2);
    cliSerial->printf_P(PSTR("Batt Mointor\n"));
    print_divider();
    if(battery.monitoring() == AP_BATT_MONITOR_DISABLED) cliSerial->printf_P(PSTR("Batt monitoring disabled"));
    if(battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_ONLY) cliSerial->printf_P(PSTR("Monitoring batt volts"));
    if(battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) cliSerial->printf_P(PSTR("Monitoring volts and current"));
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

	cliSerial->printf_P(PSTR("speed throttle:\n"));
	print_PID(&g.pidSpeedThrottle);

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
	cliSerial->printf_P(PSTR("CH1: %d | %d | %d\n"), (int)channel_steer->radio_min, (int)channel_steer->radio_trim, (int)channel_steer->radio_max);
	cliSerial->printf_P(PSTR("CH2: %d | %d | %d\n"), (int)g.rc_2.radio_min, (int)g.rc_2.radio_trim, (int)g.rc_2.radio_max);
	cliSerial->printf_P(PSTR("CH3: %d | %d | %d\n"), (int)channel_throttle->radio_min, (int)channel_throttle->radio_trim, (int)channel_throttle->radio_max);
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
    print_mode(cliSerial, m);
    cliSerial->println();
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


	if (int16_t(channel_steer->radio_in - channel_steer->radio_trim) > 100) {
	    bouncer = 10;
	}
	if (int16_t(channel_steer->radio_in - channel_steer->radio_trim) < -100) {
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
//	uint8_t b = 0;
	cliSerial->printf_P(PSTR("\nErasing EEPROM\n"));
#if CONFIG_HAL_BOARD == HAL_BOARD_REVOMINI
	hal.storage->format_eeprom(); // Format Internal 16kb Flash EEprom
#else
	for (uint16_t i = 0; i < EEPROM_MAX_ADDR; i++) {
		hal.storage->write_byte(i, 0);
	}
#endif
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
#line 1 "./APMRover2/system.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
	We will determine later if we are actually on the ground and process a
	ground start in that case.

*****************************************************************************/

#if CLI_ENABLED == ENABLED

// Functions called from the top-level menu
static int8_t	process_logs(uint8_t argc, const Menu::arg *argv);	// in Log.pde
static int8_t	setup_mode(uint8_t argc, const Menu::arg *argv);	// in setup.pde
static int8_t	test_mode(uint8_t argc, const Menu::arg *argv);		// in test.cpp
static int8_t   reboot_board(uint8_t argc, const Menu::arg *argv);

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
	{"logs",		process_logs},
	{"setup",		setup_mode},
	{"test",		test_mode},
    {"reboot",      reboot_board},
	{"help",		main_menu_help}
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

static int8_t reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

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
    hal.uartB->begin(115200, 256, 16);

	cliSerial->printf_P(PSTR("\n\nInit " FIRMWARE_STRING
						 "\n\nFree RAM: %u\n"),
                        hal.util->available_memory());
                    
	//
	// Check the EEPROM format version before loading any parameters from EEPROM.
	//
	
    load_parameters();

    BoardConfig.init();

    ServoRelayEvents.set_channel_mask(0xFFF0);

    set_control_channels();

    // after parameter load setup correct baud rate on uartA
    hal.uartA->begin(map_baudrate(g.serial0_baud, SERIAL0_BAUD));

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

    // init the GCS
    gcs[0].init(hal.uartA);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.    
    usb_connected = true;
    check_usb_mux();

    // we have a 2nd serial port for telemetry
#if CONFIG_HAL_BOARD != HAL_BOARD_REVOMINI
    hal.uartC->begin(map_baudrate(g.serial1_baud, SERIAL1_BAUD), 128, 128);
	gcs[1].init(hal.uartC);
#endif

#if MAVLINK_COMM_NUM_BUFFERS > 2
    // we may have a 3rd serial port for telemetry
    if (hal.uartD != NULL) {
        hal.uartD->begin(map_baudrate(g.serial2_baud, SERIAL2_BAUD), 128, 128);
        gcs[2].init(hal.uartD);
    }
#endif

	mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
	DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash card inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
		do_erase_logs();
    }
	if (g.log_bitmask != 0) {
		start_logging();
	}
#endif

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    adc.Init();      // APM ADC library initialization
#endif

	if (g.compass_enabled==true) {
		if (!compass.init()|| !compass.read()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
            //compass.get_offsets();						// load offsets to account for airframe magnetic interference
        }
	}

	// initialise sonar
    init_sonar();

	// Do GPS init
	g_gps = &g_gps_driver;
    // GPS initialisation
	g_gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_4G);

	//mavlink_system.sysid = MAV_SYSTEM_ID;				// Using g.sysid_this_mav
	mavlink_system.compid = 1;	//MAV_COMP_ID_IMU;   // We do not check for comp id
	mavlink_system.type = MAV_TYPE_GROUND_ROVER;

    rc_override_active = hal.rcin->set_overrides(rc_override, 8);

	init_rc_in();		// sets up rc channels from radio
	init_rc_out();		// sets up the timer libs

    relay.init();

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
    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
    if (gcs[1].initialised) {
#if CONFIG_HAL_BOARD != HAL_BOARD_REVOMINI
        hal.uartC->println_P(msg);
#endif
    }
    if (num_gcs > 2 && gcs[2].initialised) {
#if CONFIG_HAL_BOARD != HAL_BOARD_REVOMINI
        hal.uartD->println_P(msg);
#endif
    }

	startup_ground();

	if (g.log_bitmask & MASK_LOG_CMD)
			Log_Write_Startup(TYPE_GROUNDSTART_MSG);

    set_mode((enum mode)g.initial_mode.get());

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

	//IMU ground start
	//------------------------
    //

	startup_INS_ground(false);

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	// initialize commands
	// -------------------
	init_commands();

    hal.uartA->set_blocking_writes(false);
#if CONFIG_HAL_BOARD != HAL_BOARD_REVOMINI
    hal.uartC->set_blocking_writes(false);
#endif

    gcs_send_text_P(SEVERITY_LOW,PSTR("\n\n Ready to drive."));
}

/*
  set the in_reverse flag
  reset the throttle integrator if this changes in_reverse
 */
static void set_reverse(bool reverse)
{
    if (in_reverse == reverse) {
        return;
    }
    g.pidSpeedThrottle.reset_I();    
    in_reverse = reverse;
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
    set_reverse(false);
    g.pidSpeedThrottle.reset_I();

    if (control_mode != AUTO) {
        auto_triggered = false;
    }
        
	switch(control_mode)
	{
		case MANUAL:
		case HOLD:
		case LEARNING:
		case STEERING:
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

	if (g.log_bitmask & MASK_LOG_MODE)
		Log_Write_Mode();
}

/*
  called to set/unset a failsafe event. 
 */
static void failsafe_trigger(uint8_t failsafe_type, bool on)
{
    uint8_t old_bits = failsafe.bits;
    if (on) {
        failsafe.bits |= failsafe_type;
    } else {
        failsafe.bits &= ~failsafe_type;
    }
    if (old_bits == 0 && failsafe.bits != 0) {
        // a failsafe event has started
        failsafe.start_time = millis();
    }
    if (failsafe.triggered != 0 && failsafe.bits == 0) {
        // a failsafe event has ended
        gcs_send_text_fmt(PSTR("Failsafe ended"));
    }

    failsafe.triggered &= failsafe.bits;

    if (failsafe.triggered == 0 && 
        failsafe.bits != 0 && 
        millis() - failsafe.start_time > g.fs_timeout*1000 &&
        control_mode != RTL &&
        control_mode != HOLD) {
        failsafe.triggered = failsafe.bits;
        gcs_send_text_fmt(PSTR("Failsafe trigger 0x%x"), (unsigned)failsafe.triggered);
        switch (g.fs_action) {
        case 0:
            break;
        case 1:
            set_mode(RTL);
            break;
        case 2:
            set_mode(HOLD);
            break;
        }
    }
}

static void startup_INS_ground(bool force_accel_level)
{
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Warming up ADC..."));
 	mavlink_delay(500);

	// Makes the servos wiggle twice - about to begin INS calibration - HOLD LEVEL AND STILL!!
	// -----------------------
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Beginning INS calibration; do not move vehicle"));
	mavlink_delay(1000);

    ahrs.init();
	ahrs.set_fly_forward(true);

    AP_InertialSensor::Start_style style;
    if (g.skip_gyro_cal && !force_accel_level) {
        style = AP_InertialSensor::WARM_START;
    } else {
        style = AP_InertialSensor::COLD_START;
    }

	ins.init(style, ins_sample_rate);

    if (force_accel_level) {
        // when MANUAL_LEVEL is set to 1 we don't do accelerometer
        // levelling on each boot, and instead rely on the user to do
        // it once via the ground station	
        ins.init_accel();
        ahrs.set_trim(Vector3f(0, 0, 0));
	}
    ahrs.reset();
}

// updates the notify state
// should be called at 50hz
static void update_notify()
{
    notify.update();
}

static void resetPerfData(void) {
	mainLoop_count 			= 0;
	G_Dt_max 				= 0;
	ahrs.renorm_range_count 	= 0;
	ahrs.renorm_blowup_count = 0;
	gps_fix_count 			= 0;
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
    cliSerial->println_P(PSTR("Invalid baudrate"));
    return default_baud;
}


static void check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    usb_connected = usb_check;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // the APM2 has a MUX setup where the first serial port switches
    // between USB and a TTL serial connection. When on USB we use
    // SERIAL0_BAUD, but when connected as a TTL serial port we run it
    // at SERIAL1_BAUD.
    if (usb_connected) {
        hal.uartA->begin(SERIAL0_BAUD);
    } else {
        hal.uartA->begin(map_baudrate(g.serial1_baud, SERIAL1_BAUD));
    }
#endif
}


/*
 * Read board voltage in millivolts
 */
uint16_t board_voltage(void)
{
    return vcc_pin->voltage_latest() * 1000;
}

static void
print_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case MANUAL:
        port->print_P(PSTR("Manual"));
        break;
    case HOLD:
        port->print_P(PSTR("HOLD"));
        break;
    case LEARNING:
        port->print_P(PSTR("Learning"));
        break;
    case STEERING:
        port->print_P(PSTR("Steering"));
        break;
    case AUTO:
        port->print_P(PSTR("AUTO"));
        break;
    case RTL:
        port->print_P(PSTR("RTL"));
        break;
    default:
        port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
        break;
    }
}

/*
  check a digitial pin for high,low (1/0)
 */
static uint8_t check_digital_pin(uint8_t pin)
{
    int8_t dpin = hal.gpio->analogPinToDigitalPin(pin);
    if (dpin == -1) {
        return 0;
    }
    // ensure we are in input mode
    hal.gpio->pinMode(dpin, GPIO_INPUT);

    // enable pullup
    hal.gpio->write(dpin, 1);

    return hal.gpio->read(dpin);
}

/*
  write to a servo
 */
static void servo_write(uint8_t ch, uint16_t pwm)
{
#if HIL_MODE != HIL_MODE_DISABLED
    if (ch < 8) {
        RC_Channel::rc_channel(ch)->radio_out = pwm;
    }
#else
    hal.rcout->enable_ch(ch);
    hal.rcout->write(ch, pwm);
#endif
}

/*
  should we log a message type now?
 */
static bool should_log(uint32_t mask)
{
    if (!(mask & g.log_bitmask) || in_mavlink_delay) {
        return false;
    }
    bool armed;
    armed = (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);

    bool ret = armed || (g.log_bitmask & MASK_LOG_WHEN_DISARMED) != 0;
    if (ret && !DataFlash.logging_started() && !in_log_download) {
        // we have to set in_mavlink_delay to prevent logging while
        // writing headers
        in_mavlink_delay = true;
        start_logging();
        in_mavlink_delay = false;
    }
    return ret;
}
#line 1 "./APMRover2/test.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t	test_radio_pwm(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_radio(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_passthru(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_failsafe(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_gps(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_ins(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_relay(uint8_t argc,	 	const Menu::arg *argv);
static int8_t	test_wp(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_sonar(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_mag(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_modeswitch(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_logging(uint8_t argc, 		const Menu::arg *argv);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
static int8_t   test_shell(uint8_t argc,              const Menu::arg *argv);
#endif

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
static const struct Menu::command test_menu_commands[] PROGMEM = {
	{"pwm",				test_radio_pwm},
	{"radio",			test_radio},
	{"passthru",		test_passthru},
	{"failsafe",		test_failsafe},
	{"relay",			test_relay},
	{"waypoints",		test_wp},
	{"modeswitch",		test_modeswitch},

	// Tests below here are for hardware sensors only present
	// when real sensors are attached or they are emulated
	{"gps",			test_gps},
	{"ins",			test_ins},
	{"sonartest",	test_sonar},
	{"compass",		test_mag},
	{"logging",		test_logging},
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    {"shell", 				test_shell},
#endif
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
							channel_steer->radio_in,
							g.rc_2.radio_in,
							channel_throttle->radio_in,
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
        if (hal.rcin->valid_channels() > 0) {
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

		channel_steer->calc_pwm();
		channel_throttle->calc_pwm();

		// write out the servo PWM values
		// ------------------------------
		set_servos();

		cliSerial->printf_P(PSTR("IN 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
							channel_steer->control_in,
							g.rc_2.control_in,
							channel_throttle->control_in,
							g.rc_4.control_in,
							g.rc_5.control_in,
							g.rc_6.control_in,
							g.rc_7.control_in,
							g.rc_8.control_in);

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
	while(channel_throttle->control_in > 0){
		delay(20);
		read_radio();
	}

	while(1){
		delay(20);
		read_radio();

		if(channel_throttle->control_in > 0){
			cliSerial->printf_P(PSTR("THROTTLE CHANGED %d \n"), channel_throttle->control_in);
			fail_test++;
		}

		if (oldSwitchPosition != readSwitch()){
			cliSerial->printf_P(PSTR("CONTROL MODE CHANGED: "));
            print_mode(cliSerial, readSwitch());
            cliSerial->println();
			fail_test++;
		}

		if (g.fs_throttle_enabled && channel_throttle->get_failsafe()){
			cliSerial->printf_P(PSTR("THROTTLE FAILSAFE ACTIVATED: %d, "), channel_throttle->radio_in);
            print_mode(cliSerial, readSwitch());
            cliSerial->println();
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
test_relay(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		cliSerial->printf_P(PSTR("Relay on\n"));
		relay.on(0);
		delay(3000);
		if(cliSerial->available() > 0){
			return (0);
		}

		cliSerial->printf_P(PSTR("Relay off\n"));
		relay.off(0);
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
test_wp_print(const struct Location *cmd, uint8_t wp_index)
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

	cliSerial->println(MODE_CHANNEL, BASE_DEC);

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
	cliSerial->println_P(PSTR("Testing dataflash logging"));
    DataFlash.ShowDeviceInfo(cliSerial);
    return 0;
}


//-------------------------------------------------------------------------------------------
// tests in this section are for real sensors or sensors that have been simulated

static int8_t
test_gps(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(100);

		g_gps->update();

		if (g_gps->new_data){
			cliSerial->printf_P(PSTR("Lat: %ld, Lon %ld, Alt: %ldm, #sats: %d\n"),
					g_gps->latitude,
					g_gps->longitude,
					g_gps->altitude_cm/100,
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
             ins_sample_rate);
    ahrs.reset();

	print_hit_enter();
	delay(1000);

    uint8_t medium_loopCounter = 0;

	while(1){
        ins.wait_for_sample(1000);

        ahrs.update();

        if(g.compass_enabled) {
            medium_loopCounter++;
            if(medium_loopCounter >= 5){
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


static int8_t
test_mag(uint8_t argc, const Menu::arg *argv)
{
	if (!g.compass_enabled) {
        cliSerial->printf_P(PSTR("Compass: "));
		print_enabled(false);
		return (0);
    }

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
             ins_sample_rate);
    ahrs.reset();

	int counter = 0;
    float heading = 0;

    print_hit_enter();

    uint8_t medium_loopCounter = 0;

    while(1) {
        ins.wait_for_sample(1000);
        ahrs.update();

        medium_loopCounter++;
        if(medium_loopCounter >= 5){
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
            if (compass.healthy()) {
                const Vector3f mag_ofs = compass.get_offsets();
                const Vector3f mag = compass.get_field();
                cliSerial->printf_P(PSTR("Heading: %ld, XYZ: %.0f, %.0f, %.0f,\tXYZoff: %6.2f, %6.2f, %6.2f\n"),
                                    (wrap_360_cd(ToDeg(heading) * 100)) /100,
                                    mag.x, mag.y, mag.z,
                                    mag_ofs.x, mag_ofs.y, mag_ofs.z);
            } else {
                cliSerial->println_P(PSTR("compass not healthy"));
            }
            counter=0;
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

//-------------------------------------------------------------------------------------------
// real sensors that have not been simulated yet go here

static int8_t
test_sonar(uint8_t argc, const Menu::arg *argv)
{
    if (!sonar.enabled()) {
        cliSerial->println_P(PSTR("WARNING: Sonar is not enabled"));
    }

    print_hit_enter();
    init_sonar();
    
    float sonar_dist_cm_min = 0.0f;
    float sonar_dist_cm_max = 0.0f;
    float voltage_min=0.0f, voltage_max = 0.0f;
    float sonar2_dist_cm_min = 0.0f;
    float sonar2_dist_cm_max = 0.0f;
    float voltage2_min=0.0f, voltage2_max = 0.0f;
    uint32_t last_print = 0;

	while (true) {
        delay(20);
        uint32_t now = millis();

        float dist_cm = sonar.distance_cm();
        float voltage = sonar.voltage();
        if (sonar_dist_cm_min == 0.0f) {
            sonar_dist_cm_min = dist_cm;
            voltage_min = voltage;
        }
        sonar_dist_cm_max = max(sonar_dist_cm_max, dist_cm);
        sonar_dist_cm_min = min(sonar_dist_cm_min, dist_cm);
        voltage_min = min(voltage_min, voltage);
        voltage_max = max(voltage_max, voltage);

        dist_cm = sonar2.distance_cm();
        voltage = sonar2.voltage();
        if (sonar2_dist_cm_min == 0.0f) {
            sonar2_dist_cm_min = dist_cm;
            voltage2_min = voltage;
        }
        sonar2_dist_cm_max = max(sonar2_dist_cm_max, dist_cm);
        sonar2_dist_cm_min = min(sonar2_dist_cm_min, dist_cm);
        voltage2_min = min(voltage2_min, voltage);
        voltage2_max = max(voltage2_max, voltage);

        if (now - last_print >= 200) {
            cliSerial->printf_P(PSTR("sonar1 dist=%.1f:%.1fcm volt1=%.2f:%.2f   sonar2 dist=%.1f:%.1fcm volt2=%.2f:%.2f\n"), 
                                sonar_dist_cm_min, 
                                sonar_dist_cm_max, 
                                voltage_min,
                                voltage_max,
                                sonar2_dist_cm_min, 
                                sonar2_dist_cm_max, 
                                voltage2_min,
                                voltage2_max);
            voltage_min = voltage_max = 0.0f;
            voltage2_min = voltage2_max = 0.0f;
            sonar_dist_cm_min = sonar_dist_cm_max = 0.0f;
            sonar2_dist_cm_min = sonar2_dist_cm_max = 0.0f;
            last_print = now;
        }
        if (cliSerial->available() > 0) {
            break;
	    }
    }
    return (0);
}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
/*
 *  run a debug shell
 */
static int8_t
test_shell(uint8_t argc, const Menu::arg *argv)
{
    hal.util->run_debug_shell(cliSerial);
    return 0;
}
#endif

#endif // CLI_ENABLED
