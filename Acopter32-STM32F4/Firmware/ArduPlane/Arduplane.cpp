#line 1 "./Firmware/Arduplane/ArduPlane.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduPlane V2.69"
/*
 *  Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher
 *  Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon
 *  Please contribute your ideas!
 *
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <AP_GPS.h>         // ArduPilot GPS library
#include <AP_Baro.h>        // ArduPilot barometer library
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_InertialSensor.h> // Inertial Sensor Library
#include <AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <PID.h>            // PID library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder.h>     // Range finder library
#include <Filter.h>                     // Filter library
#include <AP_Buffer.h>      // APM FIFO Buffer
#include <AP_Relay.h>       // APM relay
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Airspeed.h>
#include <memcheck.h>

#include <APM_OBC.h>
#include <APM_Control.h>
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <DataFlash.h>
#include <SITL.h>
#include <wirish.h>

// optional new controller library
#if APM_CONTROL == ENABLED
#include <APM_Control.h>
#endif

// Pre-AP_HAL compatibility
#include "compat.h"

// Configuration
#include "config.h"

// Local modules
#include "defines.h"
#include "Parameters.h"
#include "GCS.h"

#include <AP_HAL_VRBRAIN.h>
//#include <AP_HAL_AVR_SITL.h>
//#include <AP_HAL_PX4.h>
//#include <AP_HAL_Empty.h>

  void setup() ;
  void loop() ;
 static void fast_loop() ;
  static void medium_loop() ;
  static void slow_loop() ;
  static void one_second_loop() ;
  static void update_GPS(void) ;
  static void update_current_flight_mode(void) ;
  static void update_navigation() ;
   static void update_alt() ;
 static float get_speed_scaler(void) ;
 static bool stick_mixing_enabled(void) ;
 static void stabilize_roll(float speed_scaler) ;
 static void stabilize_pitch(float speed_scaler) ;
 static void stabilize_stick_mixing() ;
 static void stabilize_yaw(float speed_scaler) ;
 static void stabilize_training(float speed_scaler) ;
 static void stabilize() ;
   static void crash_checker() ;
   static void calc_throttle() ;
 static void calc_nav_yaw(float speed_scaler, float ch4_inf) ;
   static void calc_nav_pitch() ;
   static void calc_nav_roll() ;
 static void throttle_slew_limit(int16_t last_throttle) ;
 static bool suppress_throttle(void) ;
 static void set_servos(void) ;
  static void demo_servos(uint8_t i) ;
 static bool alt_control_airspeed(void) ;
  static NOINLINE void send_heartbeat(mavlink_channel_t chan) ;
  static NOINLINE void send_attitude(mavlink_channel_t chan) ;
 static NOINLINE void send_fence_status(mavlink_channel_t chan) ;
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
  static void NOINLINE send_raw_imu2(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu3(mavlink_channel_t chan) ;
  static void NOINLINE send_ahrs(mavlink_channel_t chan) ;
 static void NOINLINE send_simstate(mavlink_channel_t chan) ;
  static void NOINLINE send_hwstatus(mavlink_channel_t chan) ;
  static void NOINLINE send_wind(mavlink_channel_t chan) ;
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
  static void do_erase_logs(void) ;
 static void Log_Write_Attitude(void) ;
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
 static void Log_Write_GPS(void) ;
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
 static void Log_Write_GPS() ;
 static void Log_Write_Performance() ;
 static void Log_Write_Attitude() ;
 static void Log_Write_Control_Tuning() ;
 static void Log_Write_IMU() ;
   static void load_parameters(void) ;
  void add_altitude_data(unsigned long xl, long y) ;
  static void init_commands() ;
  static void update_auto() ;
 static void reload_commands_airstart() ;
 static struct Location get_cmd_with_index_raw(int16_t i) ;
 static struct Location get_cmd_with_index(int16_t i) ;
 static void set_cmd_with_index(struct Location temp, int16_t i) ;
  static void decrement_cmd_index() ;
  static int32_t read_alt_to_hold() ;
 static void set_next_WP(struct Location *wp) ;
  static void set_guided_WP(void) ;
 void init_home() ;
 static void handle_process_nav_cmd() ;
  static void handle_process_condition_command() ;
  static void handle_process_do_command() ;
  static void handle_no_commands() ;
  static void do_RTL(void) ;
  static void do_takeoff() ;
  static void do_nav_wp() ;
  static void do_land() ;
  static void loiter_set_direction_wp(struct Location *nav_command)  ;
  static void do_loiter_unlimited() ;
  static void do_loiter_turns() ;
  static void do_loiter_time() ;
 static bool verify_takeoff() ;
 static bool verify_land() ;
  static bool verify_nav_wp() ;
  static bool verify_loiter_unlim() ;
  static bool verify_loiter_time() ;
  static bool verify_loiter_turns() ;
  static bool verify_RTL() ;
  static void do_wait_delay() ;
  static void do_change_alt() ;
  static void do_within_distance() ;
  static bool verify_wait_delay() ;
  static bool verify_change_alt() ;
  static bool verify_within_distance() ;
  static void do_loiter_at_location() ;
  static void do_jump() ;
  static void do_change_speed() ;
  static void do_set_home() ;
  static void do_set_servo() ;
  static void do_set_relay() ;
  static void do_repeat_servo(uint8_t channel, uint16_t servo_value,                             int16_t repeat, uint8_t delay_time) ;
  static void do_repeat_relay() ;
 void change_command(uint8_t cmd_index) ;
 static void update_commands(void) ;
  static void verify_commands(void) ;
   static void process_next_command() ;
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
   static void failsafe_short_on_event(int16_t fstype) ;
  static void failsafe_long_on_event(int16_t fstype) ;
  static void failsafe_short_off_event() ;
 void low_battery_event(void) ;
 static void update_events(void) ;
 void failsafe_check(uint32_t tnow) ;
 static Vector2l get_fence_point_with_index(unsigned i) ;
 static void set_fence_point_with_index(Vector2l &point, unsigned i) ;
 static void geofence_load(void) ;
 static bool geofence_enabled(void) ;
 static bool geofence_check_minalt(void) ;
 static bool geofence_check_maxalt(void) ;
 static void geofence_check(bool altitude_check_only) ;
 static bool geofence_stickmixing(void) ;
 static void geofence_send_status(mavlink_channel_t chan) ;
 bool geofence_breached(void) ;
  static void geofence_check(bool altitude_check_only) ;
 static bool geofence_stickmixing(void) ;
 static bool geofence_enabled(void) ;
 static void navigate() ;
 void calc_distance_error() ;
  static void calc_airspeed_errors() ;
  static void calc_gndspeed_undershoot() ;
  static void calc_bearing_error() ;
  static void calc_altitude_error() ;
  static int32_t wrap_360_cd(int32_t error) ;
  static int32_t wrap_180_cd(int32_t error) ;
  static void update_loiter() ;
  static void update_crosstrack(void) ;
  static void reset_crosstrack() ;
  static void init_rc_in() ;
  static void init_rc_out() ;
  static void read_radio() ;
  static void control_failsafe(uint16_t pwm) ;
  static void trim_control_surfaces() ;
  static void trim_radio() ;
   static void init_barometer(void) ;
 static int32_t read_barometer(void) ;
 static void read_airspeed(void) ;
  static void zero_airspeed(void) ;
  static void read_battery(void) ;
 void read_receiver_rssi(void) ;
 static int32_t adjusted_altitude_cm(void) ;
  static void report_batt_monitor() ;
 static void report_radio() ;
  static void report_gains() ;
  static void report_xtrack() ;
  static void report_throttle() ;
  static void report_ins() ;
  static void report_compass() ;
  static void report_flight_modes() ;
  static void print_PID(PID * pid) ;
  static void print_radio_values() ;
  static void print_switch(uint8_t p, uint8_t m) ;
  static void print_done() ;
  static void print_blanks(int16_t num) ;
  static void print_divider(void) ;
  static int8_t radio_input_switch(void) ;
   static void zero_eeprom(void) ;
  static void print_enabled(bool b) ;
  static void print_accel_offsets_and_scaling(void) ;
  static void print_gyro_offsets(void) ;
  static void init_ardupilot() ;
 static void startup_ground(void) ;
  static void set_mode(enum FlightMode mode) ;
  static void check_long_failsafe() ;
  static void check_short_failsafe() ;
   static void startup_INS_ground(bool force_accel_level) ;
   static void update_GPS_light(void) ;
   static void resetPerfData(void) ;
 static uint32_t map_baudrate(int8_t rate, uint32_t default_baud) ;
   static void check_usb_mux(void) ;
 void flash_leds(bool on) ;
 uint16_t board_voltage(void) ;
 static void reboot_apm(void) ;
   static void print_flight_mode(uint8_t mode) ;
  static void print_comma(void) ;
  static void print_hit_enter() ;
  static void test_wp_print(struct Location *cmd, uint8_t wp_index) ;
#line 77 "./Firmware/Arduplane/ArduPlane.pde"
AP_HAL::BetterStream* cliSerial;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


////////////////////////////////////////////////////////////////////////////////
// Outback Challenge Failsafe Support
////////////////////////////////////////////////////////////////////////////////
#if OBC_FAILSAFE == ENABLED
APM_OBC obc;
#endif

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
static Parameters g;

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
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
DataFlash_MP32 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
DataFlash_SITL DataFlash;
#else
// no dataflash driver
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
static AP_Int8          *flight_modes = &g.flight_mode1;

#if CONFIG_ADC == ENABLED
static AP_ADC_ADS7844 adc;
#endif

#if CONFIG_BARO == AP_BARO_BMP085
static AP_Baro_BMP085 barometer;
#elif CONFIG_BARO == AP_BARO_PX4
static AP_Baro_PX4 barometer;
#elif CONFIG_BARO == AP_BARO_HIL
static AP_Baro_BMP085_HIL barometer;
#elif CONFIG_BARO == AP_BARO_MS5611
 #if CONFIG_MS5611_SERIAL == AP_BARO_MS5611_SPI
 static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
 #elif CONFIG_MS5611_SERIAL == AP_BARO_MS5611_I2C
 static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
 #else
 #error Unrecognized CONFIG_MS5611_SERIAL setting.
 #endif
#else
 #error Unrecognized CONFIG_BARO setting
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
#elif CONFIG_INS_TYPE == CONFIG_INS_STUB
AP_InertialSensor_Stub ins;
#elif CONFIG_INS_TYPE == CONFIG_INS_OILPAN
AP_InertialSensor_Oilpan ins( &adc );
#else
  #error Unrecognised CONFIG_INS_TYPE setting.
#endif // CONFIG_INS_TYPE

#if HIL_MODE == HIL_MODE_ATTITUDE
AP_AHRS_HIL ahrs(&ins, g_gps);
#else
AP_AHRS_DCM ahrs(&ins, g_gps);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
SITL sitl;
#endif

// Training mode
static bool training_manual_roll;  // user has manual roll control
static bool training_manual_pitch; // user has manual pitch control

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
GCS_MAVLINK gcs0;
GCS_MAVLINK gcs3;

////////////////////////////////////////////////////////////////////////////////
// Analog Inputs
////////////////////////////////////////////////////////////////////////////////

AP_HAL::AnalogSource *pitot_analog_source;

// a pin for reading the receiver RSSI voltage. The scaling by 0.25 
// is to take the 0 to 1024 range down to an 8 bit range for MAVLink
AP_HAL::AnalogSource *rssi_analog_source;

AP_HAL::AnalogSource *vcc_pin;

AP_HAL::AnalogSource * batt_volt_pin;
AP_HAL::AnalogSource * batt_curr_pin;

////////////////////////////////////////////////////////////////////////////////
// Relay
////////////////////////////////////////////////////////////////////////////////
AP_Relay relay;

// Camera
#if CAMERA == ENABLED
AP_Camera camera(&relay);
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

// APM2 only
#if USB_MUX_PIN > 0
static bool usb_connected;
#endif

/* Radio values
 *               Channel assignments
 *                       1   Ailerons
 *                       2   Elevator
 *                       3   Throttle
 *                       4   Rudder
 *                       5   Aux5
 *                       6   Aux6
 *                       7   Aux7
 *                       8   Aux8/Mode
 *               Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
 *               See libraries/RC_Channel/RC_Channel_aux.h for more information
 */

////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as MANUAL, FBW-A, AUTO
enum FlightMode control_mode  = INITIALISING;
// Used to maintain the state of the previous control switch position
// This is set to -1 when we need to re-read the switch
uint8_t oldSwitchPosition;
// This is used to enable the inverted flight feature
bool inverted_flight     = false;
// These are trim values used for elevon control
// For elevons radio_in[CH_ROLL] and radio_in[CH_PITCH] are equivalent aileron and elevator, not left and right elevon
static uint16_t elevon1_trim  = 1500;
static uint16_t elevon2_trim  = 1500;
// These are used in the calculation of elevon1_trim and elevon2_trim
static uint16_t ch1_temp      = 1500;
static uint16_t ch2_temp        = 1500;
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
static int16_t failsafe;
// Used to track if the value on channel 3 (throtttle) has fallen below the failsafe threshold
// RC receiver should be set up to output a low throttle value when signal is lost
static bool ch3_failsafe;
// A timer used to help recovery from unusual attitudes.  If we enter an unusual attitude
// while in autonomous flight this variable is used  to hold roll at 0 for a recovery period
static uint8_t crash_timer;

// the time when the last HEARTBEAT message arrived from a GCS
static uint32_t last_heartbeat_ms;

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
static const float t7                        = 10000000.0;
// We use atan2 and other trig techniques to calaculate angles
// A counter used to count down valid gps fixes to allow the gps estimate to settle
// before recording our home position (and executing a ground start if we booted with an air start)
static uint8_t ground_start_count      = 5;
// Used to compute a speed estimate from the first valid gps fixes to decide if we are
// on the ground or in the air.  Used to decide if a ground start is appropriate if we
// booted with an air start.
static int16_t ground_start_avg;

// true if we have a position estimate from AHRS
static bool have_position;

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// Constants
const float radius_of_earth   = 6378100;        // meters

// This is the currently calculated direction to fly.
// deg * 100 : 0 to 360
static int32_t nav_bearing_cd;

// This is the direction to the next waypoint or loiter center
// deg * 100 : 0 to 360
static int32_t target_bearing_cd;

//This is the direction from the last waypoint to the next waypoint
// deg * 100 : 0 to 360
static int32_t crosstrack_bearing_cd;

// Direction held during phases of takeoff and landing
// deg * 100 dir of plane,  A value of -1 indicates the course has not been set/is not in use
static int32_t hold_course                   = -1;              // deg * 100 dir of plane

// There may be two active commands in Auto mode.
// This indicates the active navigation command by index number
static uint8_t nav_command_index;
// This indicates the active non-navigation command by index number
static uint8_t non_nav_command_index;
// This is the command type (eg navigate to waypoint) of the active navigation command
static uint8_t nav_command_ID          = NO_COMMAND;
static uint8_t non_nav_command_ID      = NO_COMMAND;

////////////////////////////////////////////////////////////////////////////////
// Airspeed
////////////////////////////////////////////////////////////////////////////////
// The calculated airspeed to use in FBW-B.  Also used in higher modes for insuring min ground speed is met.
// Also used for flap deployment criteria.  Centimeters per second.
static int32_t target_airspeed_cm;

// The difference between current and desired airspeed.  Used in the pitch controller.  Centimeters per second.
static float airspeed_error_cm;

// The calculated total energy error (kinetic (altitude) plus potential (airspeed)).
// Used by the throttle controller
static int32_t energy_error;

// kinetic portion of energy error (m^2/s^2)
static int32_t airspeed_energy_error;

// An amount that the airspeed should be increased in auto modes based on the user positioning the
// throttle stick in the top half of the range.  Centimeters per second.
static int16_t airspeed_nudge_cm;

// Similar to airspeed_nudge, but used when no airspeed sensor.
// 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel
static int16_t throttle_nudge = 0;

// receiver RSSI
static uint8_t receiver_rssi;


////////////////////////////////////////////////////////////////////////////////
// Ground speed
////////////////////////////////////////////////////////////////////////////////
// The amount current ground speed is below min ground speed.  Centimeters per second
static int32_t groundspeed_undershoot = 0;

////////////////////////////////////////////////////////////////////////////////
// Location Errors
////////////////////////////////////////////////////////////////////////////////
// Difference between current bearing and desired bearing.  Hundredths of a degree
static int32_t bearing_error_cd;

// Difference between current altitude and desired altitude.  Centimeters
static int32_t altitude_error_cm;

// Distance perpandicular to the course line that we are off trackline.  Meters
static float crosstrack_error;

////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
// Battery pack 1 voltage.  Initialized above the low voltage threshold to pre-load the filter and prevent low voltage events at startup.
static float battery_voltage1        = LOW_VOLTAGE * 1.05;
// Battery pack 1 instantaneous currrent draw.  Amperes
static float current_amps1;
// Totalized current (Amp-hours) from battery 1
static float current_total1;

// To Do - Add support for second battery pack
//static float  battery_voltage2    = LOW_VOLTAGE * 1.05;		// Battery 2 Voltage, initialized above threshold for filter
//static float	current_amps2;									// Current (Amperes) draw from battery 2
//static float	current_total2;									// Totalized current (Amp-hours) from battery 2

////////////////////////////////////////////////////////////////////////////////
// Airspeed Sensors
////////////////////////////////////////////////////////////////////////////////
AP_Airspeed airspeed;

////////////////////////////////////////////////////////////////////////////////
// Altitude Sensor variables

////////////////////////////////////////////////////////////////////////////////
// flight mode specific
////////////////////////////////////////////////////////////////////////////////
// Flag for using gps ground course instead of INS yaw.  Set false when takeoff command in process.
static bool takeoff_complete    = true;
// Flag to indicate if we have landed.
//Set land_complete if we are within 2 seconds distance or within 3 meters altitude of touchdown
static bool land_complete;
// Altitude threshold to complete a takeoff command in autonomous modes.  Centimeters
static int32_t takeoff_altitude;

// Minimum pitch to hold during takeoff command execution.  Hundredths of a degree
static int16_t takeoff_pitch_cd;

// this controls throttle suppression in auto modes
static bool throttle_suppressed;

////////////////////////////////////////////////////////////////////////////////
// Loiter management
////////////////////////////////////////////////////////////////////////////////
// Previous target bearing.  Used to calculate loiter rotations.  Hundredths of a degree
static int32_t old_target_bearing_cd;

// Total desired rotation in a loiter.  Used for Loiter Turns commands.  Degrees
static int32_t loiter_total;

// Direction for loiter. 1 for clockwise, -1 for counter-clockwise
static int8_t loiter_direction = 1;

// The amount in degrees we have turned since recording old_target_bearing
static int16_t loiter_delta;

// Total rotation in a loiter.  Used for Loiter Turns commands and to check for missed waypoints.  Degrees
static int32_t loiter_sum;

// The amount of time we have been in a Loiter.  Used for the Loiter Time command.  Milliseconds.
static uint32_t loiter_time_ms;

// The amount of time we should stay in a loiter for the Loiter Time command.  Milliseconds.
static uint32_t loiter_time_max_ms;

////////////////////////////////////////////////////////////////////////////////
// Navigation control variables
////////////////////////////////////////////////////////////////////////////////
// The instantaneous desired bank angle.  Hundredths of a degree
static int32_t nav_roll_cd;

// The instantaneous desired pitch angle.  Hundredths of a degree
static int32_t nav_pitch_cd;

////////////////////////////////////////////////////////////////////////////////
// Waypoint distances
////////////////////////////////////////////////////////////////////////////////
// Distance between plane and next waypoint.  Meters
// is not static because AP_Camera uses it
uint32_t wp_distance;

// Distance between previous and next waypoint.  Meters
static uint32_t wp_totalDistance;

// event control state
enum event_type { 
    EVENT_TYPE_RELAY=0,
    EVENT_TYPE_SERVO=1
};

static struct {
    enum event_type type;

	// when the event was started in ms
    uint32_t start_time_ms;

	// how long to delay the next firing of event in millis
    uint16_t delay_ms;

	// how many times to cycle : -1 (or -2) = forever, 2 = do one cycle, 4 = do two cycles
    int16_t repeat;

    // RC channel for servos
    uint8_t rc_channel;

	// PWM for servos
	uint16_t servo_value;

	// the value used to cycle events (alternate value to event_value)
    uint16_t undo_value;
} event_state;


////////////////////////////////////////////////////////////////////////////////
// Conditional command
////////////////////////////////////////////////////////////////////////////////
// A value used in condition commands (eg delay, change alt, etc.)
// For example in a change altitude command, it is the altitude to change to.
static int32_t condition_value;
// A starting value used to check the status of a conditional command.
// For example in a delay command the condition_start records that start time for the delay
static uint32_t condition_start;
// A value used in condition commands.  For example the rate at which to change altitude.
static int16_t condition_rate;

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
// Location structure defined in AP_Common
////////////////////////////////////////////////////////////////////////////////
// The home location used for RTL.  The location is set when we first get stable GPS lock
static struct   Location home;
// Flag for if we have g_gps lock and have set the home location
static bool home_is_set;
// The location of the previous waypoint.  Used for track following and altitude ramp calculations
static struct   Location prev_WP;
// The plane's current location
static struct   Location current_loc;
// The location of the current/active waypoint.  Used for altitude ramp, track following and loiter calculations.
static struct   Location next_WP;
// The location of the active waypoint in Guided mode.
static struct   Location guided_WP;
// The location structure information from the Nav command being processed
static struct   Location next_nav_command;
// The location structure information from the Non-Nav command being processed
static struct   Location next_nonnav_command;

////////////////////////////////////////////////////////////////////////////////
// Altitude / Climb rate control
////////////////////////////////////////////////////////////////////////////////
// The current desired altitude.  Altitude is linearly ramped between waypoints.  Centimeters
static int32_t target_altitude_cm;
// Altitude difference between previous and current waypoint.  Centimeters
static int32_t offset_altitude_cm;

////////////////////////////////////////////////////////////////////////////////
// INS variables
////////////////////////////////////////////////////////////////////////////////
// The main loop execution time.  Seconds
//This is the time between calls to the DCM algorithm and is the Integration time for the gyros.
static float G_Dt                                               = 0.02;

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
// Timer used to accrue data and trigger recording of the performanc monitoring log message
static int32_t perf_mon_timer;
// The maximum main loop execution time recorded in the current performance monitoring interval
static int16_t G_Dt_max = 0;
// The number of gps fixes recorded in the current performance monitoring interval
static uint8_t gps_fix_count = 0;
// A variable used by developers to track performanc metrics.
// Currently used to record the number of GCS heartbeat messages received
static int16_t pmTest1 = 0;


////////////////////////////////////////////////////////////////////////////////
// System Timers
////////////////////////////////////////////////////////////////////////////////
// Time in miliseconds of start of main control loop.  Milliseconds
static uint32_t fast_loopTimer_ms;

// Time Stamp when fast loop was complete.  Milliseconds
static uint32_t fast_loopTimeStamp_ms;

// Number of milliseconds used in last main loop cycle
static uint8_t delta_ms_fast_loop;

// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;

// Time in miliseconds of start of medium control loop.  Milliseconds
static uint32_t medium_loopTimer_ms;

// Counters for branching from main control loop to slower loops
static uint8_t medium_loopCounter;
// Number of milliseconds used in last medium loop cycle
static uint8_t delta_ms_medium_loop;

// Counters for branching from medium control loop to slower loops
static uint8_t slow_loopCounter;
// Counter to trigger execution of very low rate processes
static uint8_t superslow_loopCounter;
// Counter to trigger execution of 1 Hz processes
static uint8_t counter_one_herz;

// % MCU cycles used
static float load;


// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
// mabe one could use current_loc for lat/lon too and eliminate g_gps alltogether?
AP_Mount camera_mount(&current_loc, g_gps, &ahrs, 0);
#endif

#if MOUNT2 == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
// mabe one could use current_loc for lat/lon too and eliminate g_gps alltogether?
AP_Mount camera_mount2(&current_loc, g_gps, &ahrs, 1);
#endif

#if CAMERA == ENABLED
//pinMode(camtrig, OUTPUT);			// these are free pins PE3(5), PH3(15), PH6(18), PB4(23), PB5(24), PL1(36), PL3(38), PA6(72), PA7(71), PK0(89), PK1(88), PK2(87), PK3(86), PK4(83), PK5(84), PK6(83), PK7(82)
#endif

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

// setup the var_info table
AP_Param param_loader(var_info, WP_START_BYTE);

void setup() {
    // this needs to be the first call, as it fills memory with
    // sentinel values
    memcheck_init();

    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    rssi_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE, 0.25);

#if CONFIG_PITOT_SOURCE == PITOT_SOURCE_ADC
    pitot_analog_source = new AP_ADC_AnalogSource( &adc,
                                         CONFIG_PITOT_SOURCE_ADC_CHANNEL, 1.0);
#elif CONFIG_PITOT_SOURCE == PITOT_SOURCE_ANALOG_PIN
    pitot_analog_source = hal.analogin->channel(CONFIG_PITOT_SOURCE_ANALOG_PIN, CONFIG_PITOT_SCALING);
#endif
    vcc_pin = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);

    batt_volt_pin = hal.analogin->channel(g.battery_volt_pin);
    batt_curr_pin = hal.analogin->channel(g.battery_curr_pin);
    
    airspeed.init(pitot_analog_source);
    init_ardupilot();
}

void loop()
{
    // We want this to execute at 50Hz, but synchronised with the gyro/accel
    uint16_t num_samples = ins.num_samples_available();
    if (num_samples >= 1) {
        delta_ms_fast_loop      = millis() - fast_loopTimer_ms;
        load                = (float)(fast_loopTimeStamp_ms - fast_loopTimer_ms)/delta_ms_fast_loop;
        G_Dt                = (float)delta_ms_fast_loop / 1000.f;
        fast_loopTimer_ms   = millis();

        mainLoop_count++;

        // Execute the fast loop
        // ---------------------
        fast_loop();

        // Execute the medium loop
        // -----------------------
        medium_loop();

        counter_one_herz++;
        if(counter_one_herz == 50) {
            one_second_loop();
            counter_one_herz = 0;
        }

        if (millis() - perf_mon_timer > 20000) {
            if (mainLoop_count != 0) {
                if (g.log_bitmask & MASK_LOG_PM)
                    Log_Write_Performance();
                    resetPerfData();
            }
        }

        fast_loopTimeStamp_ms = millis();
    } else if (millis() - fast_loopTimeStamp_ms < 19) {
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

    // check for loss of control signal failsafe condition
    // ------------------------------------
    check_short_failsafe();

#if HIL_MODE == HIL_MODE_SENSORS
    // update hil before AHRS update
    gcs_update();
#endif

    ahrs.update();

    // uses the yaw from the DCM to give more accurate turns
    calc_bearing_error();

    if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST)
        Log_Write_Attitude();

    if (g.log_bitmask & MASK_LOG_IMU)
        Log_Write_IMU();

    // inertial navigation
    // ------------------
#if INERTIAL_NAVIGATION == ENABLED
    // TODO: implement inertial nav function
    inertialNavigation();
#endif

    // custom code/exceptions for flight modes
    // ---------------------------------------
    update_current_flight_mode();

    // apply desired roll, pitch and yaw to the plane
    // ----------------------------------------------
    if (control_mode > MANUAL)
        stabilize();

    // write out the servo PWM values
    // ------------------------------
    set_servos();

    gcs_update();
    gcs_data_stream_send();
}

static void medium_loop()
{
#if MOUNT == ENABLED
    camera_mount.update_mount_position();
#endif

#if MOUNT2 == ENABLED
    camera_mount2.update_mount_position();
#endif

#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
#endif

    // This is the start of the medium (10 Hz) loop pieces
    // -----------------------------------------
    switch(medium_loopCounter) {

    // This case deals with the GPS
    //-------------------------------
    case 0:
        medium_loopCounter++;
        update_GPS();
        calc_gndspeed_undershoot();

#if HIL_MODE != HIL_MODE_ATTITUDE
        if (g.compass_enabled && compass.read()) {
            ahrs.set_compass(&compass);
            compass.null_offsets();
        } else {
            ahrs.set_compass(NULL);
        }
#endif

        break;

    // This case performs some navigation computations
    //------------------------------------------------
    case 1:
        medium_loopCounter++;

        // Read 6-position switch on radio
        // -------------------------------
        read_control_switch();

        // calculate the plane's desired bearing
        // -------------------------------------
        navigate();

        break;

    // command processing
    //------------------------------
    case 2:
        medium_loopCounter++;

        // Read Airspeed
        // -------------
#if HIL_MODE != HIL_MODE_ATTITUDE
        if (airspeed.enabled()) {
            read_airspeed();
        }
#endif

        read_receiver_rssi();

        // Read altitude from sensors
        // ------------------
        update_alt();

        // altitude smoothing
        // ------------------
        if (control_mode != FLY_BY_WIRE_B)
            calc_altitude_error();

        // perform next command
        // --------------------
        update_commands();
        break;

    // This case deals with sending high rate telemetry
    //-------------------------------------------------
    case 3:
        medium_loopCounter++;

        if ((g.log_bitmask & MASK_LOG_ATTITUDE_MED) && !(g.log_bitmask & MASK_LOG_ATTITUDE_FAST))
            Log_Write_Attitude();

        if (g.log_bitmask & MASK_LOG_CTUN)
            Log_Write_Control_Tuning();

        if (g.log_bitmask & MASK_LOG_NTUN)
            Log_Write_Nav_Tuning();

        if (g.log_bitmask & MASK_LOG_GPS)
            Log_Write_GPS();
        break;

    // This case controls the slow loop
    //---------------------------------
    case 4:
        medium_loopCounter = 0;
        delta_ms_medium_loop    = millis() - medium_loopTimer_ms;
        medium_loopTimer_ms     = millis();

        if (g.battery_monitoring != 0) {
            read_battery();
        }

        slow_loop();

#if OBC_FAILSAFE == ENABLED
        // perform OBC failsafe checks
        obc.check(OBC_MODE(control_mode),
                  last_heartbeat_ms,
                  g_gps ? g_gps->last_fix_time : 0);
#endif

        break;
    }
}

static void slow_loop()
{
    // This is the slow (3 1/3 Hz) loop pieces
    //----------------------------------------
    switch (slow_loopCounter) {
    case 0:
        slow_loopCounter++;
        check_long_failsafe();
        superslow_loopCounter++;
        if(superslow_loopCounter >=200) {                                               //	200 = Execute every minute
#if HIL_MODE != HIL_MODE_ATTITUDE
            if(g.compass_enabled) {
                compass.save_offsets();
            }
#endif

            superslow_loopCounter = 0;
        }
        break;

    case 1:
        slow_loopCounter++;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
        update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_9, &g.rc_10, &g.rc_11);
#else
        update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8);
#endif
        enable_aux_servos();

#if MOUNT == ENABLED
        camera_mount.update_mount_type();
#endif
#if MOUNT2 == ENABLED
        camera_mount2.update_mount_type();
#endif
        break;

    case 2:
        slow_loopCounter = 0;
        update_events();

        mavlink_system.sysid = g.sysid_this_mav;                // This is just an ugly hack to keep mavlink_system.sysid sync'd with our parameter

        check_usb_mux();

        break;
    }
}

static void one_second_loop()
{
    if (g.log_bitmask & MASK_LOG_CURRENT)
        Log_Write_Current();

    // send a heartbeat
    gcs_send_message(MSG_HEARTBEAT);
}

static void update_GPS(void)
{
    g_gps->update();
    update_GPS_light();

    // get position from AHRS
    have_position = ahrs.get_position(&current_loc);

    if (g_gps->new_data && g_gps->fix) {
        g_gps->new_data = false;

        // for performance
        // ---------------
        gps_fix_count++;

        if(ground_start_count > 1) {
            ground_start_count--;
            ground_start_avg += g_gps->ground_speed;

        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0) {
                ground_start_count = 5;

            } else {
                if(ENABLE_AIR_START == 1 && (ground_start_avg / 5) < SPEEDFILT) {
                    startup_ground();

                    if (g.log_bitmask & MASK_LOG_CMD)
                        Log_Write_Startup(TYPE_GROUNDSTART_MSG);

                    init_home();
                } else if (ENABLE_AIR_START == 0) {
                    init_home();
                }

                if (g.compass_enabled) {
                    // Set compass declination automatically
                    compass.set_initial_location(g_gps->latitude, g_gps->longitude);
                }
                ground_start_count = 0;
            }
        }

        // see if we've breached the geo-fence
        geofence_check(false);
    }
}

static void update_current_flight_mode(void)
{
    if(control_mode == AUTO) {
        crash_checker();

        switch(nav_command_ID) {
        case MAV_CMD_NAV_TAKEOFF:
            if (hold_course != -1 && g.rudder_steer == 0) {
                calc_nav_roll();
            } else {
                nav_roll_cd = 0;
            }

            if (alt_control_airspeed()) {
                calc_nav_pitch();
                if (nav_pitch_cd < takeoff_pitch_cd)
                    nav_pitch_cd = takeoff_pitch_cd;
            } else {
                nav_pitch_cd = (g_gps->ground_speed / (float)g.airspeed_cruise_cm) * takeoff_pitch_cd;
                nav_pitch_cd = constrain_int32(nav_pitch_cd, 500, takeoff_pitch_cd);
            }

#if APM_CONTROL == DISABLED
            float aspeed;
            if (ahrs.airspeed_estimate(&aspeed)) {
                // don't use a pitch/roll integrators during takeoff if we are
                // below minimum speed
                if (aspeed < g.flybywire_airspeed_min) {
                    g.pidServoPitch.reset_I();
                    g.pidServoRoll.reset_I();
                }
            }
#endif

            // max throttle for takeoff
            g.channel_throttle.servo_out = g.throttle_max;

            break;

        case MAV_CMD_NAV_LAND:
            if (g.rudder_steer == 0 || !land_complete) {
                calc_nav_roll();
            } else {
                nav_roll_cd = 0;
            }

            if (land_complete) {
                // hold pitch constant in final approach
                nav_pitch_cd = g.land_pitch_cd;
            } else {
                calc_nav_pitch();
                if (!alt_control_airspeed()) {
                    // when not under airspeed control, don't allow
                    // down pitch in landing
                    nav_pitch_cd = constrain_int32(nav_pitch_cd, 0, nav_pitch_cd);
                }
            }
            calc_throttle();

            if (land_complete) {
                // we are in the final stage of a landing - force
                // zero throttle
                g.channel_throttle.servo_out = 0;
            }
            break;

        default:
            // we are doing normal AUTO flight, the special cases
            // are for takeoff and landing
            hold_course = -1;
            land_complete = false;
            calc_nav_roll();
            calc_nav_pitch();
            calc_throttle();
            break;
        }
    }else{
        // hold_course is only used in takeoff and landing
        hold_course = -1;

        switch(control_mode) {
        case RTL:
        case LOITER:
        case GUIDED:
            crash_checker();
            calc_nav_roll();
            calc_nav_pitch();
            calc_throttle();
            break;

        case TRAINING: {
            training_manual_roll = false;
            training_manual_pitch = false;

            // if the roll is past the set roll limit, then
            // we set target roll to the limit
            if (ahrs.roll_sensor >= g.roll_limit_cd) {
                nav_roll_cd = g.roll_limit_cd;
            } else if (ahrs.roll_sensor <= -g.roll_limit_cd) {
                nav_roll_cd = -g.roll_limit_cd;                
            } else {
                training_manual_roll = true;
                nav_roll_cd = 0;
            }

            // if the pitch is past the set pitch limits, then
            // we set target pitch to the limit
            if (ahrs.pitch_sensor >= g.pitch_limit_max_cd) {
                nav_pitch_cd = g.pitch_limit_max_cd;
            } else if (ahrs.pitch_sensor <= g.pitch_limit_min_cd) {
                nav_pitch_cd = g.pitch_limit_min_cd;
            } else {
                training_manual_pitch = true;
                nav_pitch_cd = 0;
            }
            if (inverted_flight) {
                nav_pitch_cd = -nav_pitch_cd;
            }
            break;
        }

        case FLY_BY_WIRE_A: {
            // set nav_roll and nav_pitch using sticks
            nav_roll_cd  = g.channel_roll.norm_input() * g.roll_limit_cd;
            float pitch_input = g.channel_pitch.norm_input();
            if (pitch_input > 0) {
                nav_pitch_cd = pitch_input * g.pitch_limit_max_cd;
            } else {
                nav_pitch_cd = -(pitch_input * g.pitch_limit_min_cd);
            }
            nav_pitch_cd = constrain_int32(nav_pitch_cd, g.pitch_limit_min_cd.get(), g.pitch_limit_max_cd.get());
            if (inverted_flight) {
                nav_pitch_cd = -nav_pitch_cd;
            }
            break;
        }

        case FLY_BY_WIRE_B:
            // Substitute stick inputs for Navigation control output
            // We use g.pitch_limit_min because its magnitude is
            // normally greater than g.pitch_limit_max

            // Thanks to Yury MonZon for the altitude limit code!

            nav_roll_cd = g.channel_roll.norm_input() * g.roll_limit_cd;

            float elevator_input;
            elevator_input = g.channel_pitch.norm_input();

            if (g.flybywire_elev_reverse) {
                elevator_input = -elevator_input;
            }
            if ((adjusted_altitude_cm() >= home.alt+g.FBWB_min_altitude_cm) || (g.FBWB_min_altitude_cm == 0)) {
                altitude_error_cm = elevator_input * g.pitch_limit_min_cd;
            } else {
                altitude_error_cm = (home.alt + g.FBWB_min_altitude_cm) - adjusted_altitude_cm();
                if (elevator_input < 0) {
                    altitude_error_cm += elevator_input * g.pitch_limit_min_cd;
                }
            }
            calc_throttle();
            calc_nav_pitch();
            break;

        case STABILIZE:
            nav_roll_cd        = 0;
            nav_pitch_cd       = 0;
            // throttle is passthrough
            break;

        case CIRCLE:
            // we have no GPS installed and have lost radio contact
            // or we just want to fly around in a gentle circle w/o GPS,
            // holding altitude at the altitude we set when we
            // switched into the mode
            nav_roll_cd  = g.roll_limit_cd / 3;
            calc_nav_pitch();
            calc_throttle();
            break;

        case MANUAL:
            // servo_out is for Sim control only
            // ---------------------------------
            g.channel_roll.servo_out = g.channel_roll.pwm_to_angle();
            g.channel_pitch.servo_out = g.channel_pitch.pwm_to_angle();
            g.channel_rudder.servo_out = g.channel_rudder.pwm_to_angle();
            break;
            //roll: -13788.000,  pitch: -13698.000,   thr: 0.000, rud: -13742.000

        case INITIALISING:
        case AUTO:
            // handled elsewhere
            break;
        }
    }
}

static void update_navigation()
{
    // wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
    // ------------------------------------------------------------------------

    // distance and bearing calcs only
    switch(control_mode) {
    case AUTO:
        verify_commands();
        break;
            
    case LOITER:
    case RTL:
    case GUIDED:
        update_loiter();
        calc_bearing_error();
        break;

    case MANUAL:
    case STABILIZE:
    case TRAINING:
    case INITIALISING:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
    case CIRCLE:
        // nothing to do
        break;
    }
}


static void update_alt()
{
#if HIL_MODE == HIL_MODE_ATTITUDE
    current_loc.alt = g_gps->altitude;
#else
    // this function is in place to potentially add a sonar sensor in the future
    //altitude_sensor = BARO;

    if (barometer.healthy) {
        current_loc.alt = (1 - g.altitude_mix) * g_gps->altitude;                       // alt_MSL centimeters (meters * 100)
        current_loc.alt += g.altitude_mix * (read_barometer() + home.alt);
    } else if (g_gps->fix) {
        current_loc.alt = g_gps->altitude;     // alt_MSL centimeters (meters * 100)
    }
#endif

    geofence_check(true);

    // Calculate new climb rate
    //if(medium_loopCounter == 0 && slow_loopCounter == 0)
    //	add_altitude_data(millis() / 100, g_gps->altitude / 10);
}

AP_HAL_MAIN();
#line 1 "./Firmware/Arduplane/Attitude.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that controls aileron/rudder, elevator, rudder (if 4 channel control) and throttle to produce desired attitude and airspeed.
//****************************************************************


/*
  get a speed scaling number for control surfaces. This is applied to
  PIDs to change the scaling of the PID with speed. At high speed we
  move the surfaces less, and at low speeds we move them more.
 */
static float get_speed_scaler(void)
{
    float aspeed, speed_scaler;
    if (ahrs.airspeed_estimate(&aspeed)) {
        if (aspeed > 0) {
            speed_scaler = g.scaling_speed / aspeed;
        } else {
            speed_scaler = 2.0;
        }
        speed_scaler = constrain(speed_scaler, 0.5, 2.0);
    } else {
        if (g.channel_throttle.servo_out > 0) {
            speed_scaler = 0.5 + ((float)THROTTLE_CRUISE / g.channel_throttle.servo_out / 2.0);                 // First order taylor expansion of square root
            // Should maybe be to the 2/7 power, but we aren't goint to implement that...
        }else{
            speed_scaler = 1.67;
        }
        // This case is constrained tighter as we don't have real speed info
        speed_scaler = constrain(speed_scaler, 0.6, 1.67);
    }
    return speed_scaler;
}

/*
  return true if the current settings and mode should allow for stick mixing
 */
static bool stick_mixing_enabled(void)
{
    if (control_mode == CIRCLE || control_mode > FLY_BY_WIRE_B) {
        // we're in an auto mode. Check the stick mixing flag
        if (g.stick_mixing &&
            geofence_stickmixing() &&
            failsafe == FAILSAFE_NONE) {
            // we're in an auto mode, and haven't triggered failsafe
            return true;
        } else {
            return false;
        }
    }
    // non-auto mode. Always do stick mixing
    return true;
}


/*
  this is the main roll stabilization function. It takes the
  previously set nav_roll calculates roll servo_out to try to
  stabilize the plane at the given roll
 */
static void stabilize_roll(float speed_scaler)
{
    if (crash_timer > 0) {
        nav_roll_cd = 0;
    }

    if (inverted_flight) {
        // we want to fly upside down. We need to cope with wrap of
        // the roll_sensor interfering with wrap of nav_roll, which
        // would really confuse the PID code. The easiest way to
        // handle this is to ensure both go in the same direction from
        // zero
        nav_roll_cd += 18000;
        if (ahrs.roll_sensor < 0) nav_roll_cd -= 36000;
    }

#if APM_CONTROL == DISABLED
	// Calculate dersired servo output for the roll
	// ---------------------------------------------
    g.channel_roll.servo_out = g.pidServoRoll.get_pid((nav_roll_cd - ahrs.roll_sensor), speed_scaler);
#else // APM_CONTROL == ENABLED
    // calculate roll and pitch control using new APM_Control library
    g.channel_roll.servo_out = g.rollController.get_servo_out(nav_roll_cd, speed_scaler, control_mode == STABILIZE);
#endif
}

/*
  this is the main pitch stabilization function. It takes the
  previously set nav_pitch and calculates servo_out values to try to
  stabilize the plane at the given attitude.
 */
static void stabilize_pitch(float speed_scaler)
{
#if APM_CONTROL == DISABLED
    int32_t tempcalc = nav_pitch_cd +
        fabsf(ahrs.roll_sensor * g.kff_pitch_compensation) +
        (g.channel_throttle.servo_out * g.kff_throttle_to_pitch) -
        (ahrs.pitch_sensor - g.pitch_trim_cd);
    if (inverted_flight) {
        // when flying upside down the elevator control is inverted
        tempcalc = -tempcalc;
    }
    g.channel_pitch.servo_out = g.pidServoPitch.get_pid(tempcalc, speed_scaler);
#else // APM_CONTROL == ENABLED
    g.channel_pitch.servo_out = g.pitchController.get_servo_out(nav_pitch_cd, speed_scaler, control_mode == STABILIZE);    
#endif
}

/*
  this gives the user control of the aircraft in stabilization modes
 */
static void stabilize_stick_mixing()
{
    if (!stick_mixing_enabled() ||
        control_mode == FLY_BY_WIRE_A ||
        control_mode == FLY_BY_WIRE_B ||
        control_mode == TRAINING) {
        return;
    }
    // do stick mixing on aileron/elevator
    float ch1_inf;
    float ch2_inf;
        
    ch1_inf = (float)g.channel_roll.radio_in - (float)g.channel_roll.radio_trim;
    ch1_inf = fabsf(ch1_inf);
    ch1_inf = min(ch1_inf, 400.0);
    ch1_inf = ((400.0 - ch1_inf) /400.0);
        
    ch2_inf = (float)g.channel_pitch.radio_in - g.channel_pitch.radio_trim;
    ch2_inf = fabsf(ch2_inf);
    ch2_inf = min(ch2_inf, 400.0);
    ch2_inf = ((400.0 - ch2_inf) /400.0);
        
    // scale the sensor input based on the stick input
    // -----------------------------------------------
    g.channel_roll.servo_out  *= ch1_inf;
    g.channel_pitch.servo_out *= ch2_inf;
            
    // Mix in stick inputs
    // -------------------
    g.channel_roll.servo_out  +=     g.channel_roll.pwm_to_angle();
    g.channel_pitch.servo_out +=    g.channel_pitch.pwm_to_angle();
}


/*
  stabilize the yaw axis
 */
static void stabilize_yaw(float speed_scaler)
{
    float ch4_inf = 1.0;

    if (stick_mixing_enabled()) {
        // stick mixing performed for rudder for all cases including FBW
        // important for steering on the ground during landing
        // -----------------------------------------------
        ch4_inf = (float)g.channel_rudder.radio_in - (float)g.channel_rudder.radio_trim;
        ch4_inf = fabsf(ch4_inf);
        ch4_inf = min(ch4_inf, 400.0);
        ch4_inf = ((400.0 - ch4_inf) /400.0);
    }

    // Apply output to Rudder
    calc_nav_yaw(speed_scaler, ch4_inf);
    g.channel_rudder.servo_out *= ch4_inf;
    g.channel_rudder.servo_out += g.channel_rudder.pwm_to_angle();
}


/*
  a special stabilization function for training mode
 */
static void stabilize_training(float speed_scaler)
{
    if (training_manual_roll) {
        g.channel_roll.servo_out = g.channel_roll.control_in;
    } else {
        // calculate what is needed to hold
        stabilize_roll(speed_scaler);
        if ((nav_roll_cd > 0 && g.channel_roll.control_in < g.channel_roll.servo_out) ||
            (nav_roll_cd < 0 && g.channel_roll.control_in > g.channel_roll.servo_out)) {
            // allow user to get out of the roll
            g.channel_roll.servo_out = g.channel_roll.control_in;            
        }
    }

    if (training_manual_pitch) {
        g.channel_pitch.servo_out = g.channel_pitch.control_in;
    } else {
        stabilize_pitch(speed_scaler);
        if ((nav_pitch_cd > 0 && g.channel_pitch.control_in < g.channel_pitch.servo_out) ||
            (nav_pitch_cd < 0 && g.channel_pitch.control_in > g.channel_pitch.servo_out)) {
            // allow user to get back to level
            g.channel_pitch.servo_out = g.channel_pitch.control_in;            
        }
    }

    stabilize_stick_mixing();
    stabilize_yaw(speed_scaler);
}


/*
  main stabilization function for all 3 axes
 */
static void stabilize()
{
    float speed_scaler = get_speed_scaler();

    if (control_mode == TRAINING) {
        stabilize_training(speed_scaler);
    } else {
        stabilize_roll(speed_scaler);
        stabilize_pitch(speed_scaler);
        stabilize_stick_mixing();
        stabilize_yaw(speed_scaler);
    }
}


static void crash_checker()
{
    if(ahrs.pitch_sensor < -4500) {
        crash_timer = 255;
    }
    if(crash_timer > 0)
        crash_timer--;
}


static void calc_throttle()
{
    if (!alt_control_airspeed()) {
        int16_t throttle_target = g.throttle_cruise + throttle_nudge;

        // TODO: think up an elegant way to bump throttle when
        // groundspeed_undershoot > 0 in the no airspeed sensor case; PID
        // control?

        // no airspeed sensor, we use nav pitch to determine the proper throttle output
        // AUTO, RTL, etc
        // ---------------------------------------------------------------------------
        if (nav_pitch_cd >= 0) {
            g.channel_throttle.servo_out = throttle_target + (g.throttle_max - throttle_target) * nav_pitch_cd / g.pitch_limit_max_cd;
        } else {
            g.channel_throttle.servo_out = throttle_target - (throttle_target - g.throttle_min) * nav_pitch_cd / g.pitch_limit_min_cd;
        }

        g.channel_throttle.servo_out = constrain_int16(g.channel_throttle.servo_out, g.throttle_min.get(), g.throttle_max.get());
    } else {
        // throttle control with airspeed compensation
        // -------------------------------------------
        energy_error = airspeed_energy_error + altitude_error_cm * 0.098f;

        // positive energy errors make the throttle go higher
        g.channel_throttle.servo_out = g.throttle_cruise + g.pidTeThrottle.get_pid(energy_error);
        g.channel_throttle.servo_out += (g.channel_pitch.servo_out * g.kff_pitch_to_throttle);

        g.channel_throttle.servo_out = constrain_int16(g.channel_throttle.servo_out,
                                                       g.throttle_min.get(), g.throttle_max.get());
    }

}

/*****************************************
* Calculate desired roll/pitch/yaw angles (in medium freq loop)
*****************************************/

//  Yaw is separated into a function for heading hold on rolling take-off
// ----------------------------------------------------------------------
static void calc_nav_yaw(float speed_scaler, float ch4_inf)
{
    if (hold_course != -1) {
        // steering on or close to ground
        g.channel_rudder.servo_out = g.pidWheelSteer.get_pid(bearing_error_cd, speed_scaler) + 
            g.kff_rudder_mix * g.channel_roll.servo_out;
        return;
    }

#if APM_CONTROL == DISABLED
    // always do rudder mixing from roll
    g.channel_rudder.servo_out = g.kff_rudder_mix * g.channel_roll.servo_out;

    // a PID to coordinate the turn (drive y axis accel to zero)
    Vector3f temp = ins.get_accel();
    int32_t error = -temp.y*100.0;

    g.channel_rudder.servo_out += g.pidServoRudder.get_pid(error, speed_scaler);
#else // APM_CONTROL == ENABLED
    // use the new APM_Control library
	g.channel_rudder.servo_out = g.yawController.get_servo_out(speed_scaler, ch4_inf < 0.25f) + g.channel_roll.servo_out * g.kff_rudder_mix;
#endif
}


static void calc_nav_pitch()
{
    // Calculate the Pitch of the plane
    // --------------------------------
    if (alt_control_airspeed()) {
        nav_pitch_cd = -g.pidNavPitchAirspeed.get_pid(airspeed_error_cm);
    } else {
        nav_pitch_cd = g.pidNavPitchAltitude.get_pid(altitude_error_cm);
    }
    nav_pitch_cd = constrain_int32(nav_pitch_cd, g.pitch_limit_min_cd.get(), g.pitch_limit_max_cd.get());
}


static void calc_nav_roll()
{
#define NAV_ROLL_BY_RATE 0
#if NAV_ROLL_BY_RATE
    // Scale from centidegrees (PID input) to radians per second. A P gain of 1.0 should result in a
    // desired rate of 1 degree per second per degree of error - if you're 15 degrees off, you'll try
    // to turn at 15 degrees per second.
    float turn_rate = ToRad(g.pidNavRoll.get_pid(bearing_error_cd) * .01);

    // Use airspeed_cruise as an analogue for airspeed if we don't have airspeed.
    float speed;
    if (!ahrs.airspeed_estimate(&speed)) {
        speed = g.airspeed_cruise_cm*0.01;

        // Floor the speed so that the user can't enter a bad value
        if(speed < 6) {
            speed = 6;
        }
    }

    // Bank angle = V*R/g, where V is airspeed, R is turn rate, and g is gravity.
    nav_roll = ToDeg(atanf(speed*turn_rate/GRAVITY_MSS)*100);

#else
    // this is the old nav_roll calculation. We will use this for 2.50
    // then remove for a future release
    float nav_gain_scaler = 0.01 * g_gps->ground_speed / g.scaling_speed;
    nav_gain_scaler = constrain(nav_gain_scaler, 0.2, 1.4);
    nav_roll_cd = g.pidNavRoll.get_pid(bearing_error_cd, nav_gain_scaler); //returns desired bank angle in degrees*100
#endif

    nav_roll_cd = constrain_int32(nav_roll_cd, -g.roll_limit_cd.get(), g.roll_limit_cd.get());
}


/*****************************************
* Roll servo slew limit
*****************************************/
/*
 *  float roll_slew_limit(float servo)
 *  {
 *       static float last;
 *       float temp = constrain(servo, last-ROLL_SLEW_LIMIT * delta_ms_fast_loop/1000.f, last + ROLL_SLEW_LIMIT * delta_ms_fast_loop/1000.f);
 *       last = servo;
 *       return temp;
 *  }*/

/*****************************************
* Throttle slew limit
*****************************************/
static void throttle_slew_limit(int16_t last_throttle)
{
    // if slew limit rate is set to zero then do not slew limit
    if (g.throttle_slewrate) {                   
        // limit throttle change by the given percentage per second
        float temp = g.throttle_slewrate * G_Dt * 0.01 * fabsf(g.channel_throttle.radio_max - g.channel_throttle.radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        g.channel_throttle.radio_out = constrain_int16(g.channel_throttle.radio_out, last_throttle - temp, last_throttle + temp);
    }
}


/* We want to supress the throttle if we think we are on the ground and in an autopilot controlled throttle mode.

   Disable throttle if following conditions are met:
   *       1 - We are in Circle mode (which we use for short term failsafe), or in FBW-B or higher
   *       AND
   *       2 - Our reported altitude is within 10 meters of the home altitude.
   *       3 - Our reported speed is under 5 meters per second.
   *       4 - We are not performing a takeoff in Auto mode
   *       OR
   *       5 - Home location is not set
*/
static bool suppress_throttle(void)
{
    if (!throttle_suppressed) {
        // we've previously met a condition for unsupressing the throttle
        return false;
    }
    if (control_mode != CIRCLE && control_mode <= FLY_BY_WIRE_A) {
        // the user controls the throttle
        throttle_suppressed = false;
        return false;
    }

    if (control_mode==AUTO && takeoff_complete == false) {
        // we're in auto takeoff 
        throttle_suppressed = false;
        return false;
    }
    
    if (labs(home.alt - current_loc.alt) >= 1000) {
        // we're more than 10m from the home altitude
        throttle_suppressed = false;
        return false;
    }

    if (g_gps != NULL && 
        g_gps->status() == GPS::GPS_OK && 
        g_gps->ground_speed >= 500) {
        // we're moving at more than 5 m/s
        throttle_suppressed = false;
        return false;        
    }

    // throttle remains suppressed
    return true;
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void set_servos(void)
{
    int16_t last_throttle = g.channel_throttle.radio_out;

    if (control_mode == MANUAL) {
        // do a direct pass through of radio values
        if (g.mix_mode == 0) {
            g.channel_roll.radio_out                = g.channel_roll.radio_in;
            g.channel_pitch.radio_out               = g.channel_pitch.radio_in;
        } else {
            g.channel_roll.radio_out                = hal.rcin->read(CH_ROLL);
            g.channel_pitch.radio_out               = hal.rcin->read(CH_PITCH);
        }
        g.channel_throttle.radio_out    = g.channel_throttle.radio_in;
        g.channel_rudder.radio_out              = g.channel_rudder.radio_in;

        // setup extra aileron channel. We want this to come from the
        // main aileron input channel, but using the 2nd channels dead
        // zone, reverse and min/max settings. We need to use
        // pwm_to_angle_dz() to ensure we don't trim the value for the
        // deadzone of the main aileron channel, otherwise the 2nd
        // aileron won't quite follow the first one
        int16_t aileron_in = g.channel_roll.pwm_to_angle_dz(0);
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron, aileron_in);

        // setup extra elevator channel following the same logic
        int16_t elevator_in = g.channel_pitch.pwm_to_angle_dz(0);
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_elevator, elevator_in);

        // this aileron variant assumes you have the corresponding
        // input channel setup in your transmitter for manual control
        // of the 2nd aileron
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_aileron_with_input);

        // do the same for the corresponding elevator variant
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_elevator_with_input);

        // copy flap control from transmitter
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_flap_auto);

        if (g.mix_mode != 0) {
            // set any differential spoilers to follow the elevons in
            // manual mode. 
            RC_Channel_aux::set_radio(RC_Channel_aux::k_dspoiler1, g.channel_roll.radio_out);
            RC_Channel_aux::set_radio(RC_Channel_aux::k_dspoiler2, g.channel_pitch.radio_out);
        }
    } else {
        if (g.mix_mode == 0) {
            // both types of secondary aileron are slaved to the roll servo out
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron, g.channel_roll.servo_out);
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron_with_input, g.channel_roll.servo_out);

            // both types of secondary elevator are slaved to the pitch servo out
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_elevator, g.channel_pitch.servo_out);
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_elevator_with_input, g.channel_pitch.servo_out);
        }else{
            /*Elevon mode*/
            float ch1;
            float ch2;
            ch1 = g.channel_pitch.servo_out - (BOOL_TO_SIGN(g.reverse_elevons) * g.channel_roll.servo_out);
            ch2 = g.channel_pitch.servo_out + (BOOL_TO_SIGN(g.reverse_elevons) * g.channel_roll.servo_out);

			/* Differential Spoilers
               If differential spoilers are setup, then we translate
               rudder control into splitting of the two ailerons on
               the side of the aircraft where we want to induce
               additional drag.
             */
			if (RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler1) && RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler2)) {
				float ch3 = ch1;
				float ch4 = ch2;
				if ( BOOL_TO_SIGN(g.reverse_elevons) * g.channel_rudder.servo_out < 0) {
				    ch1 += abs(g.channel_rudder.servo_out);
				    ch3 -= abs(g.channel_rudder.servo_out);
				} else {
					ch2 += abs(g.channel_rudder.servo_out);
				    ch4 -= abs(g.channel_rudder.servo_out);
				}
				RC_Channel_aux::set_servo_out(RC_Channel_aux::k_dspoiler1, ch3);
				RC_Channel_aux::set_servo_out(RC_Channel_aux::k_dspoiler2, ch4);
			}

            // directly set the radio_out values for elevon mode
            g.channel_roll.radio_out  =     elevon1_trim + (BOOL_TO_SIGN(g.reverse_ch1_elevon) * (ch1 * 500.0/ SERVO_MAX));
            g.channel_pitch.radio_out =     elevon2_trim + (BOOL_TO_SIGN(g.reverse_ch2_elevon) * (ch2 * 500.0/ SERVO_MAX));
        }

#if OBC_FAILSAFE == ENABLED
        // this is to allow the failsafe module to deliberately crash 
        // the plane. Only used in extreme circumstances to meet the
        // OBC rules
        if (obc.crash_plane()) {
            g.channel_roll.servo_out = -4500;
            g.channel_pitch.servo_out = -4500;
            g.channel_rudder.servo_out = -4500;
            g.channel_throttle.servo_out = 0;
        }
#endif
        

        // push out the PWM values
        if (g.mix_mode == 0) {
            g.channel_roll.calc_pwm();
            g.channel_pitch.calc_pwm();
        }
        g.channel_rudder.calc_pwm();

#if THROTTLE_OUT == 0
        g.channel_throttle.servo_out = 0;
#else
        // convert 0 to 100% into PWM
        g.channel_throttle.servo_out = constrain_int16(g.channel_throttle.servo_out, 
                                                       g.throttle_min.get(), 
                                                       g.throttle_max.get());

        if (suppress_throttle()) {
            // throttle is suppressed in auto mode
            g.channel_throttle.servo_out = 0;
            if (g.throttle_suppress_manual) {
                // manual pass through of throttle while throttle is suppressed
                g.channel_throttle.radio_out = g.channel_throttle.radio_in;
            } else {
                g.channel_throttle.calc_pwm();                
            }
        } else if (g.throttle_passthru_stabilize && 
                   (control_mode == STABILIZE || 
                    control_mode == TRAINING ||
                    control_mode == FLY_BY_WIRE_A)) {
            // manual pass through of throttle while in FBWA or
            // STABILIZE mode with THR_PASS_STAB set
            g.channel_throttle.radio_out = g.channel_throttle.radio_in;
        } else {
            // normal throttle calculation based on servo_out
            g.channel_throttle.calc_pwm();
        }
#endif
    }

    // Auto flap deployment
    if(control_mode < FLY_BY_WIRE_B) {
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_flap_auto);
    } else if (control_mode >= FLY_BY_WIRE_B) {
        int16_t flapSpeedSource = 0;

        // FIXME: use target_airspeed in both FBW_B and g.airspeed_enabled cases - Doug?
        if (control_mode == FLY_BY_WIRE_B) {
            flapSpeedSource = target_airspeed_cm * 0.01;
        } else if (airspeed.use()) {
            flapSpeedSource = g.airspeed_cruise_cm * 0.01;
        } else {
            flapSpeedSource = g.throttle_cruise;
        }
        if ( g.flap_1_speed != 0 && flapSpeedSource > g.flap_1_speed) {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, 0);
        } else if (g.flap_2_speed != 0 && flapSpeedSource > g.flap_2_speed) {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, g.flap_1_percent);
        } else {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, g.flap_2_percent);
        }
    }

    if (control_mode >= FLY_BY_WIRE_B) {
        /* only do throttle slew limiting in modes where throttle
         *  control is automatic */
        throttle_slew_limit(last_throttle);
    }

#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
    // send values to the PWM timers for output
    // ----------------------------------------
    hal.rcout->write(CH_1, g.channel_roll.radio_out);     // send to Servos
    hal.rcout->write(CH_2, g.channel_pitch.radio_out);     // send to Servos
    hal.rcout->write(CH_3, g.channel_throttle.radio_out);     // send to Servos
    hal.rcout->write(CH_4, g.channel_rudder.radio_out);     // send to Servos
    // Route configurable aux. functions to their respective servos
    g.rc_5.output_ch(CH_5);
    g.rc_6.output_ch(CH_6);
    g.rc_7.output_ch(CH_7);
    g.rc_8.output_ch(CH_8);
 # if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    g.rc_9.output_ch(CH_9);
    g.rc_10.output_ch(CH_10);
    g.rc_11.output_ch(CH_11);
 # endif
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

// return true if we should use airspeed for altitude/throttle control
static bool alt_control_airspeed(void)
{
    return airspeed.use() && g.alt_control_algorithm == ALT_CONTROL_DEFAULT;
}
#line 1 "./Firmware/Arduplane/GCS_Mavlink.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;

// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_ ## id ## _LEN) return false

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
    case TRAINING:
        base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case STABILIZE:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
        base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        base_mode = MAV_MODE_FLAG_GUIDED_ENABLED |
                    MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case INITIALISING:
        system_status = MAV_STATE_CALIBRATING;
        break;
    }

    if (!training_manual_pitch || !training_manual_roll) {
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;        
    }

    if (control_mode != MANUAL && control_mode != INITIALISING) {
        // stabiliser of some form is enabled
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    if (g.stick_mixing && control_mode != INITIALISING) {
        // all modes except INITIALISING have some form of manual
        // override if stick mixing is enabled
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }

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
        ahrs.pitch - radians(g.pitch_trim_cd*0.01),
        ahrs.yaw,
        omega.x,
        omega.y,
        omega.z);
}

#if GEOFENCE_ENABLED == ENABLED
static NOINLINE void send_fence_status(mavlink_channel_t chan)
{
    geofence_send_status(chan);
}
#endif


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

    case STABILIZE:
    case FLY_BY_WIRE_A:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        break;

    case FLY_BY_WIRE_B:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        control_sensors_enabled |= (1<<15); // motor control
        break;

    case TRAINING:
        if (!training_manual_roll || !training_manual_pitch) {
            control_sensors_enabled |= (1<<10); // 3D angular rate control
            control_sensors_enabled |= (1<<11); // attitude stabilisation        
        }
        break;

    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
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
    uint8_t battery_remaining = -1;

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
        (current_loc.alt-home.alt) * 10,           // millimeters above ground
        g_gps->velocity_north() * 100,  // X speed cm/s (+ve North)
        g_gps->velocity_east()  * 100,  // Y speed cm/s (+ve East)
        g_gps->velocity_down()  * -100, // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);
}

static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)
{
    int16_t bearing = (hold_course==-1 ? nav_bearing_cd : hold_course) / 100;
    mavlink_msg_nav_controller_output_send(
        chan,
        nav_roll_cd * 0.01,
        nav_pitch_cd * 0.01,
        bearing,
        target_bearing_cd * 0.01,
        wp_distance,
        altitude_error_cm * 0.01,
        airspeed_error_cm,
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
        10000 * g.channel_roll.norm_output(),
        10000 * g.channel_pitch.norm_output(),
        10000 * g.channel_throttle.norm_output(),
        10000 * g.channel_rudder.norm_output(),
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
    float aspeed;
    if (airspeed.enabled()) {
        aspeed = airspeed.get_airspeed();
    } else if (!ahrs.airspeed_estimate(&aspeed)) {
        aspeed = 0;
    }
    float throttle_norm = g.channel_throttle.norm_output() * 100;
    throttle_norm = constrain_int16(throttle_norm, -100, 100);
    uint16_t throttle = ((uint16_t)(throttle_norm + 100)) / 2;
    mavlink_msg_vfr_hud_send(
        chan,
        aspeed,
        (float)g_gps->ground_speed * 0.01,
        (ahrs.yaw_sensor / 100) % 360,
        throttle,
        current_loc.alt / 100.0,
        barometer.get_climb_rate());
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

static void NOINLINE send_raw_imu2(mavlink_channel_t chan)
{
    int32_t pressure = barometer.get_pressure();
    mavlink_msg_scaled_pressure_send(
        chan,
        millis(),
        pressure/100.0,
        (pressure - barometer.get_ground_pressure())/100.0,
        barometer.get_temperature());
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
                                    barometer.get_raw_pressure(),
                                    barometer.get_raw_temp(),
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

static void NOINLINE send_wind(mavlink_channel_t chan)
{
    Vector3f wind = ahrs.wind_estimate();
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)), // use negative, to give
                                          // direction wind is coming from
        wind.length(),
        wind.z);
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

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_raw_imu2(chan);
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

#if GEOFENCE_ENABLED == ENABLED
    case MSG_FENCE_STATUS:
        CHECK_PAYLOAD_SIZE(FENCE_STATUS);
        send_fence_status(chan);
        break;
#endif

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

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind(chan);
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

/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  1),
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  1),
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[2],  1),
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRates[3],  1),
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  1),
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[5],  1),
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[6],  1),
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[7],  1),
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[8],  50),
    AP_GROUPEND
};


GCS_MAVLINK::GCS_MAVLINK() :
    packet_drops(0),
    waypoint_send_timeout(1000), // 1 second
    waypoint_receive_timeout(1000) // 1 second
{
    AP_Param::setup_object_defaults(this, var_info);
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
    uint16_t nbytes = comm_get_available(chan);
    for (uint16_t i=0; i<nbytes; i++)
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
    if (waypoint_receiving && (millis() - waypoint_timelast_receive) > waypoint_receive_timeout) {
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
        if (streamRates[STREAM_PARAMS].get() <= 0) {
            streamRates[STREAM_PARAMS].set(50);
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
        send_message(MSG_RAW_IMU2);
        send_message(MSG_RAW_IMU3);
    }

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);            // TODO - remove this message after location message is working
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
        send_message(MSG_FENCE_STATUS);
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
        send_message(MSG_WIND);
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
            freq = 0;                     // stop sending
        else if (packet.start_stop == 1)
            freq = packet.req_message_rate;                     // start sending
        else
            break;

        switch (packet.req_stream_id) {
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

        case MAV_CMD_NAV_LOITER_UNLIM:
            set_mode(LOITER);
            result = MAV_RESULT_ACCEPTED;
            break;

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
                packet.param2 == 1) {
                startup_INS_ground(true);
            } else if (packet.param3 == 1) {
                init_barometer();
                if (airspeed.enabled()) {
                    zero_airspeed();
                }
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
                set_mode(FLY_BY_WIRE_A);
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

        case MAV_CMD_DO_REPEAT_SERVO:
            do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4);
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
        case CIRCLE:
        case STABILIZE:
        case TRAINING:
        case FLY_BY_WIRE_A:
        case FLY_BY_WIRE_B:
        case AUTO:
        case RTL:
        case LOITER:
            set_mode((enum FlightMode)packet.custom_mode);
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
            g.command_total + 1);     // + home

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
        tell_command = get_cmd_with_index_raw(packet.seq);

        // set frame of waypoint
        uint8_t frame;

        if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;     // reference frame
        } else {
            frame = MAV_FRAME_GLOBAL;     // reference frame
        }

        float param1 = 0, param2 = 0, param3 = 0, param4 = 0;

        // time that the mav should loiter in milliseconds
        uint8_t current = 0;     // 1 (true), 0 (false)

        if (packet.seq == (uint16_t)g.command_index)
            current = 1;

        uint8_t autocontinue = 1;     // 1 (true), 0 (false)

        float x = 0, y = 0, z = 0;

        if (tell_command.id < MAV_CMD_NAV_LAST || tell_command.id == MAV_CMD_CONDITION_CHANGE_ALT) {
            // command needs scaling
            x = tell_command.lat/1.0e7;     // local (x), global (latitude)
            y = tell_command.lng/1.0e7;     // local (y), global (longitude)
            z = tell_command.alt/1.0e2;
        }

        switch (tell_command.id) {                                              // Switch to map APM command fields inot MAVLink command fields

        case MAV_CMD_NAV_LOITER_TIME:
        case MAV_CMD_NAV_LOITER_TURNS:
            if (tell_command.options & MASK_OPTIONS_LOITER_DIRECTION) {
                param3 = -g.loiter_radius;
            } else {
                param3 = g.loiter_radius;
            }
        case MAV_CMD_NAV_TAKEOFF:
        case MAV_CMD_DO_SET_HOME:
            param1 = tell_command.p1;
            break;

        case MAV_CMD_NAV_LOITER_UNLIM:
            if (tell_command.options & MASK_OPTIONS_LOITER_DIRECTION) {
                param3 = -g.loiter_radius;;
            } else {
                param3 = g.loiter_radius;
            }
            break;
        case MAV_CMD_CONDITION_CHANGE_ALT:
            x=0;                                // Clear fields loaded above that we don't want sent for this command
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
            tell_command.lat = 1.0e7f*packet.x;                                     // in as DD converted to * t7
            tell_command.lng = 1.0e7f*packet.y;                                     // in as DD converted to * t7
            tell_command.alt = packet.z*1.0e2f;                                     // in as m converted to cm
            tell_command.options = 0;                                     // absolute altitude
            break;
        }

#ifdef MAV_FRAME_LOCAL_NED
        case MAV_FRAME_LOCAL_NED:                         // local (relative to home position)
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
        case MAV_FRAME_LOCAL:                         // local (relative to home position)
        {
            tell_command.lat = 1.0e7f*ToDeg(packet.x/
                                           (radius_of_earth*cosf(ToRad(home.lat/1.0e7f)))) + home.lat;
            tell_command.lng = 1.0e7f*ToDeg(packet.y/radius_of_earth) + home.lng;
            tell_command.alt = packet.z*1.0e2f;
            tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
            break;
        }
#endif

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:                         // absolute lat/lng, relative altitude
        {
            tell_command.lat = 1.0e7f * packet.x;                                     // in as DD converted to * t7
            tell_command.lng = 1.0e7f * packet.y;                                     // in as DD converted to * t7
            tell_command.alt = packet.z * 1.0e2f;
            tell_command.options = MASK_OPTIONS_RELATIVE_ALT;                                     // store altitude relative!! Always!!
            break;
        }

        default:
            result = MAV_MISSION_UNSUPPORTED_FRAME;
            break;
        }


        if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

        // Switch to map APM command fields into MAVLink command fields
        switch (tell_command.id) {
        case MAV_CMD_NAV_LOITER_UNLIM:
		    if (packet.param3 < 0) {
		        tell_command.options |= MASK_OPTIONS_LOITER_DIRECTION;
		    }
        case MAV_CMD_NAV_WAYPOINT:
        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        case MAV_CMD_NAV_LAND:
            break;

        case MAV_CMD_NAV_LOITER_TURNS:
        case MAV_CMD_NAV_LOITER_TIME:
            if (packet.param3 < 0) {
                tell_command.options |= MASK_OPTIONS_LOITER_DIRECTION;
            } 
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

        if(packet.current == 2) {                                               //current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
            guided_WP = tell_command;

            // add home alt if needed
            if (guided_WP.options & MASK_OPTIONS_RELATIVE_ALT) {
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

        } else if(packet.current == 3) {                                               //current = 3 is a flag to tell us this is a alt change only

            // add home alt if needed
            if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
                tell_command.alt += home.alt;
            }

            next_WP.alt = tell_command.alt;

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

#if GEOFENCE_ENABLED == ENABLED
    // receive a fence point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_FENCE_POINT: {
        mavlink_fence_point_t packet;
        mavlink_msg_fence_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        if (g.fence_action != FENCE_ACTION_NONE) {
            send_text_P(SEVERITY_LOW,PSTR("fencing must be disabled"));
        } else if (packet.count != g.fence_total) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point;
            point.x = packet.lat*1.0e7;
            point.y = packet.lng*1.0e7;
            set_fence_point_with_index(point, packet.idx);
        }
        break;
    }

    // send a fence point to GCS
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
        mavlink_fence_fetch_point_t packet;
        mavlink_msg_fence_fetch_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        if (packet.idx >= g.fence_total) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point = get_fence_point_with_index(packet.idx);
            mavlink_msg_fence_point_send(chan, 0, 0, packet.idx, g.fence_total,
                                         point.x*1.0e-7, point.y*1.0e-7);
        }
        break;
    }
#endif // GEOFENCE_ENABLED

    case MAVLINK_MSG_ID_PARAM_SET:
    {
        AP_Param                  *vp;
        enum ap_var_type var_type;

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
        if ((NULL != vp) &&                                 // exists
            !isnan(packet.param_value) &&                       // not nan
            !isinf(packet.param_value)) {                       // not inf

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
                v = constrain(v, -2147483648.0, 2147483647.0);
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
                -1);     // XXX we don't actually know what its index is...
        }

        break;
    }     // end case

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

        // a RC override message is consiered to be a 'heartbeat' from
        // the ground station for failsafe purposes
        last_heartbeat_ms = millis();
        break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        // We keep track of the last time we received a heartbeat from
        // our GCS for failsafe purposes
        if (msg->sysid != g.sysid_my_gcs) break;
        last_heartbeat_ms = millis();
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

        if (g_gps != NULL) {
            // set gps hil sensor
            g_gps->setHIL(packet.time_usec/1000,
                          packet.lat*1.0e-7, packet.lon*1.0e-7, packet.alt*1.0e-3,
                          vel*1.0e-2, cog*1.0e-2, 0, 10);
        }

        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;

        // m/s/s
        Vector3f accels;
        accels.x = packet.xacc * (GRAVITY_MSS/1000.0);
        accels.y = packet.yacc * (GRAVITY_MSS/1000.0);
        accels.z = packet.zacc * (GRAVITY_MSS/1000.0);

        ins.set_gyro(gyros);
        ins.set_accel(accels);

        // approximate a barometer
        float y;
        const float Temp = 312;

        y = (packet.alt - 584000.0) / 29271.267;
        y /= (Temp / 10.0) + 273.15;
        y = 1.0/exp(y);
        y *= 95446.0;

        barometer.setHIL(Temp, y);

 #if HIL_MODE == HIL_MODE_ATTITUDE
        // set AHRS hil sensor. We don't do this in sensors mode, as
        // in that case the attitude is computed via DCM
        ahrs.setHil(packet.roll,packet.pitch,packet.yaw,packet.rollspeed,
                    packet.pitchspeed,packet.yawspeed);

 #endif

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
        float value;

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
    check_usb_mux();

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

#line 1 "./Firmware/Arduplane/Log.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash.log memory
// Code to interact with the user to dump or erase logs

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
static int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
static const struct Menu::command log_menu_commands[] PROGMEM = {
    {"dump",        dump_log},
    {"erase",       erase_logs},
    {"enable",      select_logs},
    {"disable",     select_logs}
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

    cliSerial->println_P(PSTR("logs enabled: "));

    if (0 == g.log_bitmask) {
        cliSerial->println_P(PSTR("none"));
    }else{
        // Macro to make the following code a bit easier on the eye.
        // Pass it the capitalised name of the log option, as defined
        // in defines.h but without the LOG_ prefix.  It will check for
        // the bit being set and print the name of the log option to suit.
 #define PLOG(_s) if (g.log_bitmask & MASK_LOG_ ## _s) cliSerial->printf_P(PSTR(" %S"), PSTR(# _s))
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
        cliSerial->printf_P(PSTR("\n%u logs\n"), (unsigned)num_logs);

        for(int16_t i=num_logs; i>=1; i--) {
            int16_t last_log_start = log_start, last_log_end = log_end;
            temp = last_log_num-i+1;
            DataFlash.get_log_boundaries(temp, log_start, log_end);
            cliSerial->printf_P(PSTR("Log %d,    start %d,   end %d\n"), (int)temp, (int)log_start, (int)log_end);
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
    int16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = DataFlash.find_last_log();

    if (dump_log == -2) {
        for(uint16_t count=1; count<=DataFlash.df_NumPages; count++) {
            DataFlash.StartRead(count);
            cliSerial->printf_P(PSTR("DF page, log file #, log page: %d,\t"), (int)count);
            cliSerial->printf_P(PSTR("%d,\t"), (int)DataFlash.GetFileNumber());
            cliSerial->printf_P(PSTR("%d\n"), (int)DataFlash.GetFilePage());
        }
        return(-1);
    } else if (dump_log <= 0) {
        cliSerial->printf_P(PSTR("dumping all\n"));
        Log_Read(1, DataFlash.df_NumPages);
        return(-1);
    } else if ((argc != 2)
        || (dump_log <= (last_log_num - DataFlash.get_num_logs()))
        || (dump_log > last_log_num))
    {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    cliSerial->printf_P(PSTR("Dumping Log %d,    start pg %d,   end pg %d\n"),
                    (int)dump_log,
                    (int)dump_log_start,
                    (int)dump_log_end);

    Log_Read(dump_log_start, dump_log_end);
    cliSerial->printf_P(PSTR("Done\n"));
    return 0;
}

static void do_erase_logs(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Erasing logs"));
    DataFlash.EraseAll();
    gcs_send_text_P(SEVERITY_LOW, PSTR("Log erase complete"));
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
    uint16_t bits;

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
 #define TARG(_s)        if (!strcasecmp_P(argv[1].str, PSTR(# _s))) bits |= MASK_LOG_ ## _s
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
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
};

// Write an attitude packet. Total length : 10 bytes
static void Log_Write_Attitude(void)
{
    struct log_Attitute pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        roll  : ahrs.roll_sensor,
        pitch : ahrs.pitch_sensor,
        yaw   : ahrs.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read an attitude packet
static void Log_Read_Attitude()
{
    struct log_Attitute pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("ATT, %ld, %ld, %ld\n"),
                        (long)pkt.roll, 
                        (long)pkt.pitch,
                        (long)pkt.yaw);
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
    int16_t roll_out;
    int16_t nav_roll_cd;
    int16_t roll;
    int16_t pitch_out;
    int16_t nav_pitch_cd;
    int16_t pitch;
    int16_t throttle_out;
    int16_t rudder_out;
    int16_t accel_y;
};

// Write a control tuning packet. Total length : 22 bytes
static void Log_Write_Control_Tuning()
{
    Vector3f accel = ins.get_accel();
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        roll_out        : (int16_t)g.channel_roll.servo_out,
        nav_roll_cd     : (int16_t)nav_roll_cd,
        roll            : (int16_t)ahrs.roll_sensor,
        pitch_out       : (int16_t)g.channel_pitch.servo_out,
        nav_pitch_cd    : (int16_t)nav_pitch_cd,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)g.channel_throttle.servo_out,
        rudder_out      : (int16_t)g.channel_rudder.servo_out,
        accel_y         : (int16_t)(accel.y * 10000)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read an control tuning packet
static void Log_Read_Control_Tuning()
{
    struct log_Control_Tuning pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    cliSerial->printf_P(PSTR("CTUN, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.4f\n"),
        (float)pkt.roll_out / 100.f,
        (float)pkt.nav_roll_cd / 100.f,
        (float)pkt.roll / 100.f,
        (float)pkt.pitch_out / 100.f,
        (float)pkt.nav_pitch_cd / 100.f,
        (float)pkt.pitch / 100.f,
        (float)pkt.throttle_out / 100.f,
        (float)pkt.rudder_out / 100.f,
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
    int16_t airspeed_cm;
};

// Write a navigation tuning packet. Total length : 18 bytes
static void Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        yaw                 : (uint16_t)ahrs.yaw_sensor,
        wp_distance         : wp_distance,
        target_bearing_cd   : (uint16_t)target_bearing_cd,
        nav_bearing_cd      : (uint16_t)nav_bearing_cd,
        altitude_error_cm   : (int16_t)altitude_error_cm,
        airspeed_cm         : (int16_t)airspeed.get_airspeed_cm()
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
                    (float)(pkt.airspeed_cm/100.0f));
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
    print_flight_mode(pkt.mode);
}

struct log_GPS {
    LOG_PACKET_HEADER;
    uint32_t gps_time;
    uint8_t  num_sats;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  rel_altitude;
    int32_t  altitude;
    uint32_t ground_speed;
    int32_t  ground_course;
};

// Write an GPS packet. Total length : 30 bytes
static void Log_Write_GPS(void)
{
    struct log_GPS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GPS_MSG),
    	gps_time      : g_gps->time,
        num_sats      : g_gps->num_sats,
        latitude      : g_gps->latitude,
        longitude     : g_gps->longitude,
        rel_altitude  : current_loc.alt,
        altitude      : g_gps->altitude,
        ground_speed  : g_gps->ground_speed,
        ground_course : g_gps->ground_course
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a GPS packet
static void Log_Read_GPS()
{
    struct log_GPS pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("GPS, %ld, %u, "),
                        (long)pkt.gps_time,
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
static void Log_Write_IMU()
{
    struct log_IMU pkt = {
        LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
        gyro  : ins.get_gyro(),
        accel : ins.get_accel()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

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

// Read the DataFlash.log memory : Packet Parser
static void Log_Read(int16_t start_page, int16_t end_page)
{
    cliSerial->printf_P(PSTR("\n" THISFIRMWARE
                             "\nFree RAM: %u\n"),
                        memcheck_available_memory());

    DataFlash.log_read_process(start_page, end_page, log_callback);
}

// Read the DataFlash.log memory : Packet Parser
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
static void Log_Write_GPS() {}
static void Log_Write_Performance() {}
static void Log_Write_Attitude() {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_IMU() {}

static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}


#endif // LOGGING_ENABLED
#line 1 "./Firmware/Arduplane/Parameters.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
 *  ArduPlane parameter definitions
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info : class::var_info} }

const AP_Param::Info var_info[] PROGMEM = {
    GSCALAR(format_version,         "FORMAT_VERSION", 0),
    GSCALAR(software_type,          "SYSID_SW_TYPE",  Parameters::k_software_type),
    GSCALAR(sysid_this_mav,         "SYSID_THISMAV",  MAV_SYSTEM_ID),
    GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",    255),

    // @Param: SERIAL0_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the main uart
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
    GSCALAR(serial0_baud,           "SERIAL0_BAUD",   SERIAL0_BAUD/1000),

    // @Param: SERIAL3_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the telemetry port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
    GSCALAR(serial3_baud,           "SERIAL3_BAUD",   SERIAL3_BAUD/1000),

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay 
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Standard
    // @Units: seconds
    // @Range: 0 10
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: KFF_PTCHCOMP
    // @DisplayName: Pitch Compensation
    // @Description: Adds pitch input to compensate for the loss of lift due to roll control. 0 = 0 %, 1 = 100%
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    GSCALAR(kff_pitch_compensation, "KFF_PTCHCOMP",   PITCH_COMP),

    // @Param: KFF_RDDRMIX
    // @DisplayName: Rudder Mix
    // @Description: The amount of rudder mix to apply during aileron movement 0 = 0 %, 1 = 100%
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(kff_rudder_mix,         "KFF_RDDRMIX",    RUDDER_MIX),

    // @Param: KFF_PTCH2THR
    // @DisplayName: Pitch to Throttle Mix
    // @Description: Pitch to throttle feed-forward gain.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    GSCALAR(kff_pitch_to_throttle,  "KFF_PTCH2THR",   P_TO_T),

    // @Param: KFF_THR2PTCH
    // @DisplayName: Throttle to Pitch Mix
    // @Description: Throttle to pitch feed-forward gain.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    GSCALAR(kff_throttle_to_pitch,  "KFF_THR2PTCH",   T_TO_P),

    // @Param: MANUAL_LEVEL
    // @DisplayName: Manual Level
    // @Description: Setting this to Disabled(0) will enable autolevel on every boot. Setting it to Enabled(1) will do a calibration only when you tell it to
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(manual_level,           "MANUAL_LEVEL",   MANUAL_LEVEL),

    // @Param: STICK_MIXING
    // @DisplayName: Stick Mixing
    // @Description: When enabled, this adds user stick input to the control surfaces in auto modes, allowing the user to have some degree of flight control without changing modes
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(stick_mixing,           "STICK_MIXING",   1),

    // @Param: RUDDER_STEER
    // @DisplayName: Rudder steering on takeoff and landing
    // @Description: When enabled, only rudder will be used for steering during takeoff and landing, with the ailerons used to hold the plane level
    // @Values: 0:Disabled,1:Enabled
    // @User: User
    GSCALAR(rudder_steer,           "RUDDER_STEER",   0),

    // @Param: land_pitch_cd
    // @DisplayName: Landing Pitch
    // @Description: Used in autoland for planes without airspeed sensors in hundredths of a degree
    // @Units: centi-Degrees
    // @User: Advanced
    GSCALAR(land_pitch_cd,          "LAND_PITCH_CD",  0),

    // @Param: land_flare_alt
    // @DisplayName: Landing flare altitude
    // @Description: Altitude in autoland at which to lock heading and flare to the LAND_PITCH_CD pitch
    // @Units: meters
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(land_flare_alt,          "LAND_FLARE_ALT",  3.0),

    // @Param: land_flare_sec
    // @DisplayName: Landing flare time
    // @Description: Time before landing point at which to lock heading and flare to the LAND_PITCH_CD pitch
    // @Units: seconds
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(land_flare_sec,          "LAND_FLARE_SEC",  2.0),

    // @Param: XTRK_GAIN_SC
    // @DisplayName: Crosstrack Gain
    // @Description: The scale between distance off the line and angle to meet the line (in Degrees * 100)
    // @Range: 0 2000
    // @Increment: 1
    // @User: Standard
    GSCALAR(crosstrack_gain,        "XTRK_GAIN_SC",   XTRACK_GAIN_SCALED),

    // @Param: XTRK_ANGLE_CD
    // @DisplayName: Crosstrack Entry Angle
    // @Description: Maximum angle used to correct for track following.
    // @Units: centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    GSCALAR(crosstrack_entry_angle, "XTRK_ANGLE_CD",  XTRACK_ENTRY_ANGLE_CENTIDEGREE),

    // @Param: XTRK_USE_WIND
    // @DisplayName: Crosstrack Wind correction
    // @Description: If enabled, use wind estimation for navigation crosstrack when using a compass for yaw
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(crosstrack_use_wind, "XTRK_USE_WIND",     1),

    // @Param: XTRK_MIN_DIST
    // @DisplayName: Crosstrack mininum distance
    // @Description: Minimum distance in meters between waypoints to do crosstrack correction.
    // @Units: Meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(crosstrack_min_distance, "XTRK_MIN_DIST",  50),

    // @Param: ALT_MIX
    // @DisplayName: Gps to Baro Mix
    // @Description: The percent of mixing between gps altitude and baro altitude. 0 = 100% gps, 1 = 100% baro
    // @Units: Percent
    // @Range: 0 1
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(altitude_mix,           "ALT_MIX",        ALTITUDE_MIX),

    // @Param: ALT_CTRL_ALG
    // @DisplayName: Altitude control algorithm
    // @Description: This sets what algorithm will be used for altitude control. The default is to select the algorithm based on whether airspeed is enabled. If you set it to 1, then the airspeed based algorithm won't be used for altitude control, but airspeed can be used for other flight control functions
    // @Values: 0:Default Method,1:non-airspeed
    // @User: Advanced
    GSCALAR(alt_control_algorithm, "ALT_CTRL_ALG",    ALT_CONTROL_DEFAULT),

    // @Param: ALT_OFFSET
    // @DisplayName: Altitude offset
    // @Description: This is added to the target altitude in automatic flight. It can be used to add a global altitude offset to a mission, or to adjust for barometric pressure changes
    // @Units: Meters
    // @Range: -32767 32767
    // @Increment: 1
    // @User: Advanced
    GSCALAR(alt_offset, "ALT_OFFSET",                 0),

    GSCALAR(command_total,          "CMD_TOTAL",      0),
    GSCALAR(command_index,          "CMD_INDEX",      0),

    // @Param: WP_RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: Meters
    // @Range: 1 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_radius,        "WP_RADIUS",      WP_RADIUS_DEFAULT),

    // @Param: WP_LOITER_RAD
    // @DisplayName: Waypoint Loiter Radius
    // @Description: Defines the distance from the waypoint center, the plane will maintain during a loiter
    // @Units: Meters
    // @Range: 1 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(loiter_radius,          "WP_LOITER_RAD",  LOITER_RADIUS_DEFAULT),

#if GEOFENCE_ENABLED == ENABLED
    // @Param: FENCE_ACTION
    // @DisplayName: Action on geofence breach
    // @Description: What to do on fence breach
    // @Values: 0:None,1:GuidedMode,2:ReportOnly
    // @User: Standard
    GSCALAR(fence_action,           "FENCE_ACTION",   0),

    // @Param: FENCE_TOTAL
    // @DisplayName: Fence Total
    // @Description: Number of geofence points currently loaded
    // @User: Standard
    GSCALAR(fence_total,            "FENCE_TOTAL",    0),

    // @Param: FENCE_CHANNEL
    // @DisplayName: Fence Channel
    // @Description: RC Channel to use to enable geofence. PWM input above 1750 enables the geofence
    // @User: Standard
    GSCALAR(fence_channel,          "FENCE_CHANNEL",  0),

    // @Param: FENCE_MINALT
    // @DisplayName: Fence Minimum Altitude
    // @Description: Minimum altitude allowed before geofence triggers
    // @Units: meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(fence_minalt,           "FENCE_MINALT",   0),

    // @Param: FENCE_MAXALT
    // @DisplayName: Fence Maximum Altitude
    // @Description: Maximum altitude allowed before geofence triggers
    // @Units: meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(fence_maxalt,           "FENCE_MAXALT",   0),
#endif

    // @Param: ARSPD_FBW_MIN
    // @DisplayName: Fly By Wire Minimum Airspeed
    // @Description: Airspeed corresponding to minimum throttle in Fly By Wire B mode.
    // @Units: m/s
    // @Range: 5 50
    // @Increment: 1
    // @User: Standard
    GSCALAR(flybywire_airspeed_min, "ARSPD_FBW_MIN",  AIRSPEED_FBW_MIN),

    // @Param: ARSPD_FBW_MAX
    // @DisplayName: Fly By Wire Maximum Airspeed
    // @Description: Airspeed corresponding to maximum throttle in Fly By Wire B mode.
    // @Units: m/s
    // @Range: 5 50
    // @Increment: 1
    // @User: Standard
    GSCALAR(flybywire_airspeed_max, "ARSPD_FBW_MAX",  AIRSPEED_FBW_MAX),

    // @Param: FBWB_ELEV_REV
    // @DisplayName: Fly By Wire elevator reverse
    // @Description: Reverse sense of elevator in FBWB. When set to 0 up elevator (pulling back on the stick) means to lower altitude. When set to 1, up elevator means to raise altitude.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(flybywire_elev_reverse, "FBWB_ELEV_REV",  0),

    // @Param: THR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: The minimum throttle setting to which the autopilot will apply.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_min,           "THR_MIN",        THROTTLE_MIN),

    // @Param: THR_MAX
    // @DisplayName: Maximum Throttle
    // @Description: The maximum throttle setting to which the autopilot will apply.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_max,           "THR_MAX",        THROTTLE_MAX),

    // @Param: THR_SLEWRATE
    // @DisplayName: Throttle slew rate
    // @Description: maximum percentage change in throttle per second. A setting of 10 means to not change the throttle by more than 10% of the full throttle range in one second
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_slewrate,      "THR_SLEWRATE",   THROTTLE_SLEW_LIMIT),

    // @Param: THR_SUPP_MAN
    // @DisplayName: Throttle suppress manual passthru
    // @Description: When throttle is supressed in auto mode it is normally forced to zero. If you enable this option, then while suppressed it will be manual throttle. This is useful on petrol engines to hold the idle throttle manually while waiting for takeoff
	// @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(throttle_suppress_manual,"THR_SUPP_MAN",   0),

    // @Param: THR_PASS_STAB
    // @DisplayName: Throttle passthru in stabilize
    // @Description: If this is set then when in STABILIZE or FBWA mode the throttle is a direct passthru from the transmitter. This means the THR_MIN and THR_MAX settings are not used in these modes. This is useful for petrol engines where you setup a throttle cut switch that suppresses the throttle below the normal minimum.
	// @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(throttle_passthru_stabilize,"THR_PASS_STAB",   0),

    // @Param: THR_FAILSAFE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(throttle_fs_enabled,    "THR_FAILSAFE",   THROTTLE_FAILSAFE),


    // @Param: THR_FS_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers
    // @User: Standard
    GSCALAR(throttle_fs_value,      "THR_FS_VALUE",   THROTTLE_FS_VALUE),

    // @Param: TRIM_THROTTLE
    // @DisplayName: Throttle cruise percentage
    // @Description: The target percentage of throttle to apply for normal flight
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_cruise,        "TRIM_THROTTLE",  THROTTLE_CRUISE),

    // @Param: THROTTLE_NUDGE
    // @DisplayName: Throttle nudge enable
    // @Description: When enabled, this uses the throttle input in auto-throttle modes to 'nudge' the throttle to higher or lower values
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    // @User: Standard
    GSCALAR(throttle_nudge,         "THROTTLE_NUDGE",  1),

    // @Param: FS_SHORT_ACTN
    // @DisplayName: Short failsafe action
    // @Description: The action to take on a short (1 second) failsafe event
    // @Values: 0:None,1:ReturnToLaunch
    // @User: Standard
    GSCALAR(short_fs_action,        "FS_SHORT_ACTN",  SHORT_FAILSAFE_ACTION),

    // @Param: FS_LONG_ACTN
    // @DisplayName: Long failsafe action
    // @Description: The action to take on a long (20 second) failsafe event
    // @Values: 0:None,1:ReturnToLaunch
    // @User: Standard
    GSCALAR(long_fs_action,         "FS_LONG_ACTN",   LONG_FAILSAFE_ACTION),

    // @Param: FS_GCS_ENABL
    // @DisplayName: GCS failsafe enable
    // @Description: Enable ground control station telemetry failsafe. Failsafe will trigger after 20 seconds of no MAVLink heartbeat messages. WARNING: Enabling this option opens up the possibility of your plane going into failsafe mode and running the motor on the ground it it loses contact with your ground station. If this option is enabled on an electric plane then either use a separate motor arming switch or remove the propeller in any ground testing.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(gcs_heartbeat_fs_enabled, "FS_GCS_ENABL", GCS_HEARTBEAT_FAILSAFE),

    // @Param: FLTMODE_CH
    // @DisplayName: Flightmode channel
    // @Description: RC Channel to use for flight mode control
    // @User: Advanced
    GSCALAR(flight_mode_channel,    "FLTMODE_CH",     FLIGHT_MODE_CHANNEL),

    // @Param: FLTMODE1
    // @DisplayName: FlightMode1
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
    // @Description: Flight mode for switch position 1 (910 to 1230 and above 2049)
    GSCALAR(flight_mode1,           "FLTMODE1",       FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: FlightMode2
    // @Description: Flight mode for switch position 2 (1231 to 1360)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
    GSCALAR(flight_mode2,           "FLTMODE2",       FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: FlightMode3
    // @Description: Flight mode for switch position 3 (1361 to 1490)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
    GSCALAR(flight_mode3,           "FLTMODE3",       FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: FlightMode4
    // @Description: Flight mode for switch position 4 (1491 to 1620)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
    GSCALAR(flight_mode4,           "FLTMODE4",       FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: FlightMode5
    // @Description: Flight mode for switch position 5 (1621 to 1749)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
    GSCALAR(flight_mode5,           "FLTMODE5",       FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: FlightMode6
    // @Description: Flight mode for switch position 6 (1750 to 2049)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
    GSCALAR(flight_mode6,           "FLTMODE6",       FLIGHT_MODE_6),

    // @Param: LIM_ROLL_CD
    // @DisplayName: Maximum Bank Angle
    // @Description: The maximum commanded bank angle in either direction
    // @Units: centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    GSCALAR(roll_limit_cd,          "LIM_ROLL_CD",    HEAD_MAX_CENTIDEGREE),

    // @Param: LIM_PITCH_MAX
    // @DisplayName: Maximum Pitch Angle
    // @Description: The maximum commanded pitch up angle
    // @Units: centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    GSCALAR(pitch_limit_max_cd,     "LIM_PITCH_MAX",  PITCH_MAX_CENTIDEGREE),

    // @Param: LIM_PITCH_MIN
    // @DisplayName: Minimum Pitch Angle
    // @Description: The minimum commanded pitch down angle
    // @Units: centi-Degrees
    // @Range: -9000 0
    // @Increment: 1
    // @User: Standard
    GSCALAR(pitch_limit_min_cd,     "LIM_PITCH_MIN",  PITCH_MIN_CENTIDEGREE),

    // @Param: AUTO_TRIM
    // @DisplayName: Auto trim
    // @Description: Set RC trim PWM levels to current levels when switching away from manual mode
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(auto_trim,              "TRIM_AUTO",      AUTO_TRIM),

    // @Param: MIX_MODE
    // @DisplayName: Elevon mixing
    // @Description: Enable elevon mixing
    // @Values: 0:Disabled,1:Enabled
    // @User: User
    GSCALAR(mix_mode,               "ELEVON_MIXING",  ELEVON_MIXING),

    // @Param: ELEVON_REVERSE
    // @DisplayName: Elevon reverse
    // @Description: Reverse elevon mixing
    // @Values: 0:Disabled,1:Enabled
    // @User: User
    GSCALAR(reverse_elevons,        "ELEVON_REVERSE", ELEVON_REVERSE),


    // @Param: ELEVON_CH1_REV
    // @DisplayName: Elevon reverse
    // @Description: Reverse elevon channel 1
    // @Values: -1:Disabled,1:Enabled
    // @User: User
    GSCALAR(reverse_ch1_elevon,     "ELEVON_CH1_REV", ELEVON_CH1_REVERSE),

    // @Param: ELEVON_CH2_REV
    // @DisplayName: Elevon reverse
    // @Description: Reverse elevon channel 2
    // @Values: -1:Disabled,1:Enabled
    // @User: User
    GSCALAR(reverse_ch2_elevon,     "ELEVON_CH2_REV", ELEVON_CH2_REVERSE),

    // @Param: SYS_NUM_RESETS
    // @DisplayName: Num Resets
    // @Description: Number of APM board resets
    // @User: Advanced
    GSCALAR(num_resets,             "SYS_NUM_RESETS", 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: bitmap of log fields to enable
    // @User: Advanced
    GSCALAR(log_bitmask,            "LOG_BITMASK",    DEFAULT_LOG_BITMASK),

    // @Param: RST_SWITCH_CH
    // @DisplayName: Reset Switch Channel
    // @Description: RC channel to use to reset to last flight mode	after geofence takeover.
    // @User: Advanced
    GSCALAR(reset_switch_chan,      "RST_SWITCH_CH",  0),

    // @Param: RST_MISSION_CH
    // @DisplayName: Reset Mission Channel
    // @Description: RC channel to use to reset the mission to the first waypoint. When this channel goes above 1750 the mission is reset. Set RST_MISSION_CH to 0 to disable.
    // @User: Advanced
    GSCALAR(reset_mission_chan,      "RST_MISSION_CH",  0),

    // @Param: TRIM_ARSPD_CM
    // @DisplayName: Target airspeed
    // @Description: Airspeed in cm/s to aim for when airspeed is enabled in auto mode
    // @Units: cm/s
    // @User: User
    GSCALAR(airspeed_cruise_cm,     "TRIM_ARSPD_CM",  AIRSPEED_CRUISE_CM),

    // @Param: SCALING_SPEED
    // @DisplayName: speed used for speed scaling calculations
    // @Description: Airspeed in m/s to use when calculating surface speed scaling. Note that changing this value will affect all PID values
    // @Units: m/s
    // @User: Advanced
    GSCALAR(scaling_speed,        "SCALING_SPEED",    SCALING_SPEED),

    // @Param: MIN_GNDSPD_CM
    // @DisplayName: Minimum ground speed
    // @Description: Minimum ground speed in cm/s when under airspeed control
    // @Units: cm/s
    // @User: Advanced
    GSCALAR(min_gndspeed_cm,      "MIN_GNDSPD_CM",  MIN_GNDSPEED_CM),

    // @Param: TRIM_PITCH_CD
    // @DisplayName: Pitch angle offset
    // @Description: offset to add to pitch - used for trimming tail draggers
    // @Units: centi-Degrees
    // @User: Advanced
    GSCALAR(pitch_trim_cd,        "TRIM_PITCH_CD",  0),

    // @Param: ALT_HOLD_RTL
    // @DisplayName: RTL altitude
    // @Description: Return to launch target altitude
    // @Units: centimeters
    // @User: User
    GSCALAR(RTL_altitude_cm,        "ALT_HOLD_RTL",   ALT_HOLD_HOME_CM),

    GSCALAR(FBWB_min_altitude_cm,   "ALT_HOLD_FBWCM", ALT_HOLD_FBW_CM),

    // @Param: MAG_ENABLE
    // @DisplayName: Enable Compass
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(compass_enabled,        "MAG_ENABLE",     MAGNETOMETER),

    GSCALAR(flap_1_percent,         "FLAP_1_PERCNT",  FLAP_1_PERCENT),
    GSCALAR(flap_1_speed,           "FLAP_1_SPEED",   FLAP_1_SPEED),
    GSCALAR(flap_2_percent,         "FLAP_2_PERCNT",  FLAP_2_PERCENT),
    GSCALAR(flap_2_speed,           "FLAP_2_SPEED",   FLAP_2_SPEED),


    GSCALAR(battery_monitoring,     "BATT_MONITOR",   0),
    GSCALAR(volt_div_ratio,         "VOLT_DIVIDER",   VOLT_DIV_RATIO),

    // @Param: APM_PER_VOLT
    // @DisplayName: Apms per volt
    // @Description: Number of amps that a 1V reading on the current sensor corresponds to
    // @Units: A/V
    // @User: Standard
    GSCALAR(curr_amp_per_volt,      "AMP_PER_VOLT",   CURR_AMP_PER_VOLT),

    // @Param: AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor
    // @Units: Volts
    // @User: Standard
    GSCALAR(curr_amp_offset,        "AMP_OFFSET",     0),

    GSCALAR(input_voltage,          "INPUT_VOLTS",    INPUT_VOLTAGE),

    // @Param: BATT_CAPACITY
    // @DisplayName: Battery capacity
    // @Description: Capacity of the battery in mAh when full
    // @Units: mAh
    // @User: Standard
    GSCALAR(pack_capacity,          "BATT_CAPACITY",  1760),

    // @Param: BATT_VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 0:A0, 1:A1, 13:A13
    // @User: Standard
    GSCALAR(battery_volt_pin,    "BATT_VOLT_PIN",    BATTERY_VOLT_PIN),

    // @Param: BATT_CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 1:A1, 2:A2, 12:A12
    // @User: Standard
    GSCALAR(battery_curr_pin,    "BATT_CURR_PIN",    BATTERY_CURR_PIN),

    // @Param: RSSI_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: This selects an analog pin for the receiver RSSI voltage. It assumes the voltage is 5V for max rssi, 0V for minimum
    // @Values: -1:Disabled, 0:A0, 1:A1, 13:A13
    // @User: Standard
    GSCALAR(rssi_pin,            "RSSI_PIN",         -1),

    GSCALAR(inverted_flight_ch,     "INVERTEDFLT_CH", 0),

    // barometer ground calibration. The GND_ prefix is chosen for
    // compatibility with previous releases of ArduPlane
    GOBJECT(barometer, "GND_", AP_Baro),

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,                  "CAM_", AP_Camera),
#endif

    // RC channel
    //-----------
    // @Group: RC1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(channel_roll,            "RC1_", RC_Channel),
    // @Group: RC2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(channel_pitch,           "RC2_", RC_Channel),
    // @Group: RC3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(channel_throttle,        "RC3_", RC_Channel),
    // @Group: RC4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(channel_rudder,          "RC4_", RC_Channel),

    // @Group: RC5_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_5,                    "RC5_", RC_Channel_aux),

    // @Group: RC6_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_6,                    "RC6_", RC_Channel_aux),

    // @Group: RC7_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_7,                    "RC7_", RC_Channel_aux),

    // @Group: RC8_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_8,                    "RC8_", RC_Channel_aux),

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // @Group: RC9_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_9,                    "RC9_", RC_Channel_aux),

    // @Group: RC10_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_10,                    "RC10_", RC_Channel_aux),

    // @Group: RC11_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_11,                    "RC11_", RC_Channel_aux),
#endif

	GGROUP(pidNavRoll,              "HDNG2RLL_",  PID),
	GGROUP(pidNavPitchAirspeed,     "ARSP2PTCH_", PID),
	GGROUP(pidTeThrottle,           "ENRGY2THR_", PID),
	GGROUP(pidNavPitchAltitude,     "ALT2PTCH_",  PID),
	GGROUP(pidWheelSteer,           "WHEELSTEER_",PID),

#if APM_CONTROL == DISABLED
	GGROUP(pidServoRoll,            "RLL2SRV_",   PID),
	GGROUP(pidServoPitch,           "PTCH2SRV_",  PID),
	GGROUP(pidServoRudder,          "YW2SRV_",    PID),
#else
	GGROUP(rollController,          "RLL_",       AP_RollController),
	GGROUP(pitchController,         "PTCH_",      AP_PitchController),
	GGROUP(yawController,           "YWCTL_",     AP_YawController),
#endif

	// variables not in the g class which contain EEPROM saved variables

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/Compass.cpp
    GOBJECT(compass,                "COMPASS_",     Compass),
    GOBJECT(gcs0,                                   "SR0_",     GCS_MAVLINK),
    GOBJECT(gcs3,                                   "SR3_",     GCS_MAVLINK),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                    "INS_", AP_InertialSensor),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

    // @Group: ARSPD_
    // @Path: ../libraries/AP_Airspeed/AP_Airspeed.cpp
    GOBJECT(airspeed,                               "ARSPD_",   AP_Airspeed),

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

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL),
#endif

#if OBC_FAILSAFE == ENABLED
    GOBJECT(obc,  "FS_", APM_OBC),
#endif

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
        uint32_t before = micros();
        // Load all auto-loaded EEPROM variables
        AP_Param::load_all();

        cliSerial->printf_P(PSTR("load_all took %luus\n"), micros() - before);
    }
}
#line 1 "./Firmware/Arduplane/climb_rate.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if 0 // currently unused

struct DataPoint {
    unsigned long x;
    long y;
};

DataPoint history[ALTITUDE_HISTORY_LENGTH];               // Collection of (x,y) points to regress a rate of change from
unsigned char hindex;   // Index in history for the current data point

unsigned long xoffset;
unsigned char n;

// Intermediate variables for regression calculation
long xi;
long yi;
long xiyi;
unsigned long xi2;

void add_altitude_data(unsigned long xl, long y)
{
    //Reset the regression if our X variable overflowed
    if (xl < xoffset)
        n = 0;

    //To allow calculation of sum(xi*yi), make sure X hasn't exceeded 2^32/2^15/length
    if (xl - xoffset > 131072/ALTITUDE_HISTORY_LENGTH)
        n = 0;

    if (n == ALTITUDE_HISTORY_LENGTH) {
        xi -= history[hindex].x;
        yi -= history[hindex].y;
        xiyi -= (long)history[hindex].x * history[hindex].y;
        xi2 -= history[hindex].x * history[hindex].x;
    } else {
        if (n == 0) {
            xoffset = xl;
            xi = 0;
            yi = 0;
            xiyi = 0;
            xi2 = 0;
        }
        n++;
    }

    history[hindex].x = xl - xoffset;
    history[hindex].y = y;

    xi += history[hindex].x;
    yi += history[hindex].y;
    xiyi += (long)history[hindex].x * history[hindex].y;
    xi2 += history[hindex].x * history[hindex].x;

    if (++hindex >= ALTITUDE_HISTORY_LENGTH)
        hindex = 0;
}
#endif

#line 1 "./Firmware/Arduplane/commands.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  logic for dealing with the current command in the mission and home location
 */

static void init_commands()
{
    g.command_index.set_and_save(0);
    nav_command_ID  = NO_COMMAND;
    non_nav_command_ID      = NO_COMMAND;
    next_nav_command.id     = CMD_BLANK;
    nav_command_index = 0;
}

static void update_auto()
{
    if (g.command_index >= g.command_total) {
        handle_no_commands();
        if(g.command_total == 0) {
            next_WP.lat             = home.lat + 1000;                  // so we don't have bad calcs
            next_WP.lng             = home.lng + 1000;                  // so we don't have bad calcs
        }
    } else {
        if(g.command_index != 0) {
            g.command_index = nav_command_index;
            nav_command_index--;
        }
        nav_command_ID  = NO_COMMAND;
        non_nav_command_ID      = NO_COMMAND;
        next_nav_command.id     = CMD_BLANK;
        process_next_command();
    }
}

// this is only used by an air-start
static void reload_commands_airstart()
{
    init_commands();
    decrement_cmd_index();
}

/*
  fetch a mission item from EEPROM
*/
static struct Location get_cmd_with_index_raw(int16_t i)
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
        temp.alt = hal.storage->read_dword(mem);

        mem += 4;
        temp.lat = hal.storage->read_dword(mem);

        mem += 4;
        temp.lng = hal.storage->read_dword(mem);
    }

    return temp;
}

/*
  fetch a mission item from EEPROM. Adjust altitude to be absolute
*/
static struct Location get_cmd_with_index(int16_t i)
{
    struct Location temp;

    temp = get_cmd_with_index_raw(i);

    // Add on home altitude if we are a nav command (or other command with altitude) and stored alt is relative
    if ((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) &&
        (temp.options & MASK_OPTIONS_RELATIVE_ALT) &&
        (temp.lat != 0 || temp.lng != 0 || temp.alt != 0)) {
        temp.alt += home.alt;
    }

    return temp;
}

// Setters
// -------
static void set_cmd_with_index(struct Location temp, int16_t i)
{
    i = constrain_int16(i, 0, g.command_total.get());
    uint16_t mem = WP_START_BYTE + (i * WP_SIZE);

    // force home wp to absolute height
    if (i == 0) {
        temp.options &= ~(MASK_OPTIONS_RELATIVE_ALT);
    }
    // zero unused bits
    temp.options &= (MASK_OPTIONS_RELATIVE_ALT | MASK_OPTIONS_LOITER_DIRECTION);

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

static void decrement_cmd_index()
{
    if (g.command_index > 0) {
        g.command_index.set_and_save(g.command_index - 1);
    }
}

static int32_t read_alt_to_hold()
{
    if (g.RTL_altitude_cm < 0) {
        return current_loc.alt;
    }
    return g.RTL_altitude_cm + home.alt;
}


/*
 *  This function stores waypoint commands
 *  It looks to see what the next command type is and finds the last command.
 */
static void set_next_WP(struct Location *wp)
{
    // copy the current WP into the OldWP slot
    // ---------------------------------------
    prev_WP = next_WP;

    // Load the next_WP slot
    // ---------------------
    next_WP = *wp;

    // if lat and lon is zero, then use current lat/lon
    // this allows a mission to contain a "loiter on the spot"
    // command
    if (next_WP.lat == 0 && next_WP.lng == 0) {
        next_WP.lat = current_loc.lat;
        next_WP.lng = current_loc.lng;
        // additionally treat zero altitude as current altitude
        if (next_WP.alt == 0) {
            next_WP.alt = current_loc.alt;
        }
    }


    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Resetting prev_WP"));
        prev_WP = current_loc;
    }

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    target_altitude_cm = current_loc.alt;

    if (prev_WP.id != MAV_CMD_NAV_TAKEOFF && 
        prev_WP.alt != home.alt && 
        (next_WP.id == MAV_CMD_NAV_WAYPOINT || next_WP.id == MAV_CMD_NAV_LAND)) {
        offset_altitude_cm = next_WP.alt - prev_WP.alt;
    } else {
        offset_altitude_cm = 0;        
    }

    // zero out our loiter vals to watch for missed waypoints
    loiter_delta            = 0;
    loiter_sum                      = 0;
    loiter_total            = 0;

    // this is handy for the groundstation
    wp_totalDistance        = get_distance(&current_loc, &next_WP);
    wp_distance             = wp_totalDistance;
    target_bearing_cd       = get_bearing_cd(&current_loc, &next_WP);
    nav_bearing_cd          = target_bearing_cd;

    // to check if we have missed the WP
    // ----------------------------
    old_target_bearing_cd   = target_bearing_cd;

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

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    target_altitude_cm = current_loc.alt;
    offset_altitude_cm = next_WP.alt - prev_WP.alt;

    // this is handy for the groundstation
    wp_totalDistance        = get_distance(&current_loc, &next_WP);
    wp_distance             = wp_totalDistance;
    target_bearing_cd       = get_bearing_cd(&current_loc, &next_WP);

    // to check if we have missed the WP
    // ----------------------------
    old_target_bearing_cd = target_bearing_cd;

    // set a new crosstrack bearing
    // ----------------------------
    reset_crosstrack();
}

// run this at setup on the ground
// -------------------------------
void init_home()
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("init home"));

    // block until we get a good fix
    // -----------------------------
    while (!g_gps->new_data || !g_gps->fix) {
        g_gps->update();
#if HIL_MODE != HIL_MODE_DISABLED
        // update hil gps so we have new_data
        gcs_update();
#endif
    }

    home.id         = MAV_CMD_NAV_WAYPOINT;
    home.lng        = g_gps->longitude;                                 // Lon * 10**7
    home.lat        = g_gps->latitude;                                  // Lat * 10**7
    home.alt        = max(g_gps->altitude, 0);
    home_is_set = true;

    gcs_send_text_fmt(PSTR("gps alt: %lu"), (unsigned long)home.alt);

    // Save Home to EEPROM - Command 0
    // -------------------
    set_cmd_with_index(home, 0);

    // Save prev loc
    // -------------
    next_WP = prev_WP = home;

    // Load home for a default guided_WP
    // -------------
    guided_WP = home;
    guided_WP.alt += g.RTL_altitude_cm;

}



#line 1 "./Firmware/Arduplane/commands_logic.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
static void
handle_process_nav_cmd()
{
    // set land_complete to false to stop us zeroing the throttle
    land_complete = false;

    // set takeoff_complete to true so we don't add extra evevator
    // except in a takeoff 
    takeoff_complete = true;

    gcs_send_text_fmt(PSTR("Executing nav command ID #%i"),next_nav_command.id);
    switch(next_nav_command.id) {

    case MAV_CMD_NAV_TAKEOFF:
        do_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // Navigate to Waypoint
        do_nav_wp();
        break;

    case MAV_CMD_NAV_LAND:              // LAND to Waypoint
        do_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // Loiter indefinitely
        do_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              // Loiter N Times
        do_loiter_turns();
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        do_loiter_time();
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
    switch(next_nonnav_command.id) {

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
    switch(next_nonnav_command.id) {

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
        do_repeat_servo(next_nonnav_command.p1, next_nonnav_command.alt,
                        next_nonnav_command.lat, next_nonnav_command.lng);
        break;

    case MAV_CMD_DO_REPEAT_RELAY:
        do_repeat_relay();
        break;

#if CAMERA == ENABLED
    case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
        camera.trigger_pic();
        break;
#endif

#if MOUNT == ENABLED
    // Sets the region of interest (ROI) for a sensor set or the
    // vehicle itself. This can then be used by the vehicles control
    // system to control the vehicle attitude and the attitude of various
    // devices such as cameras.
    //    |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
    case MAV_CMD_NAV_ROI:
 #if 0
        // send the command to the camera mount
        camera_mount.set_roi_cmd(&command_nav_queue);
 #else
        gcs_send_text_P(SEVERITY_LOW, PSTR("DO_SET_ROI not supported"));
 #endif
        break;

    case MAV_CMD_DO_MOUNT_CONFIGURE:                    // Mission command to configure a camera mount |Mount operation mode (see MAV_CONFIGURE_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
        camera_mount.configure_cmd();
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // Mission command to control a camera mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|
        camera_mount.control_cmd();
        break;
#endif
    }
}

static void handle_no_commands()
{
    gcs_send_text_fmt(PSTR("Returning to Home"));
    next_nav_command = home;
    next_nav_command.alt = read_alt_to_hold();
    next_nav_command.id = MAV_CMD_NAV_LOITER_UNLIM;
    nav_command_ID = MAV_CMD_NAV_LOITER_UNLIM;
    non_nav_command_ID = WAIT_COMMAND;
    handle_process_nav_cmd();

}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

static bool verify_nav_command()        // Returns true if command complete
{
    switch(nav_command_ID) {

    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();

    case MAV_CMD_NAV_LAND:
        return verify_land();

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp();

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlim();

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_loiter_turns();

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();

    default:
        gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_nav: Invalid or no current Nav cmd"));
    }
    return false;
}

static bool verify_condition_command()          // Returns true if command complete
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
    control_mode    = RTL;
    crash_timer     = 0;
    next_WP                 = home;
    loiter_direction = 1;

    // Altitude to hold over home
    // Set by configuration tool
    // -------------------------
    next_WP.alt = read_alt_to_hold();

    if (g.log_bitmask & MASK_LOG_MODE)
        Log_Write_Mode(control_mode);
}

static void do_takeoff()
{
    set_next_WP(&next_nav_command);
    // pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0
    takeoff_pitch_cd                = (int)next_nav_command.p1 * 100;
    takeoff_altitude        = next_nav_command.alt;
    next_WP.lat             = home.lat + 1000;          // so we don't have bad calcs
    next_WP.lng             = home.lng + 1000;          // so we don't have bad calcs
    takeoff_complete        = false;                            // set flag to use gps ground course during TO.  IMU will be doing yaw drift correction
    // Flag also used to override "on the ground" throttle disable
}

static void do_nav_wp()
{
    set_next_WP(&next_nav_command);
}

static void do_land()
{
    set_next_WP(&next_nav_command);
}

static void loiter_set_direction_wp(struct Location *nav_command) 
{
    if (nav_command->options & MASK_OPTIONS_LOITER_DIRECTION) {
        loiter_direction = -1;
    } else {
        loiter_direction=1;
    }
}

static void do_loiter_unlimited()
{
    set_next_WP(&next_nav_command);
    loiter_set_direction_wp(&next_nav_command);
}

static void do_loiter_turns()
{
    set_next_WP(&next_nav_command);
    loiter_total = next_nav_command.p1 * 360;
    loiter_set_direction_wp(&next_nav_command);
}

static void do_loiter_time()
{
    set_next_WP(&next_nav_command);
    loiter_time_ms = millis();
    loiter_time_max_ms = next_nav_command.p1 * (uint32_t)1000;     // units are seconds
    loiter_set_direction_wp(&next_nav_command);
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
static bool verify_takeoff()
{
    if (ahrs.yaw_initialised()) {
        if (hold_course == -1) {
            // save our current course to take off
            hold_course = ahrs.yaw_sensor;
            gcs_send_text_fmt(PSTR("Holding course %ld"), hold_course);
        }
    }

    if (hold_course != -1) {
        // recalc bearing error with hold_course;
        nav_bearing_cd = hold_course;
        // recalc bearing error
        calc_bearing_error();
    }

    if (adjusted_altitude_cm() > takeoff_altitude)  {
        hold_course = -1;
        takeoff_complete = true;
        next_WP = current_loc;
        return true;
    } else {
        return false;
    }
}

// we are executing a landing
static bool verify_land()
{
    // we don't 'verify' landing in the sense that it never completes,
    // so we don't verify command completion. Instead we use this to
    // adjust final landing parameters

    // Set land_complete if we are within 2 seconds distance or within
    // 3 meters altitude of the landing point
    if ((wp_distance <= (g.land_flare_sec*g_gps->ground_speed*0.01))
        || (adjusted_altitude_cm() <= next_WP.alt + g.land_flare_alt*100)) {

        land_complete = true;

        if (hold_course == -1) {
            // we have just reached the threshold of to flare for landing.
            // We now don't want to do any radical
            // turns, as rolling could put the wings into the runway.
            // To prevent further turns we set hold_course to the
            // current heading. Previously we set this to
            // crosstrack_bearing, but the xtrack bearing can easily
            // be quite large at this point, and that could induce a
            // sudden large roll correction which is very nasty at
            // this point in the landing.
            hold_course = ahrs.yaw_sensor;
            gcs_send_text_fmt(PSTR("Land Complete - Hold course %ld"), hold_course);
        }

        if (g_gps->ground_speed*0.01 < 3.0) {
            // reload any airspeed or groundspeed parameters that may have
            // been set for landing. We don't do this till ground
            // speed drops below 3.0 m/s as otherwise we will change
            // target speeds too early.
            g.airspeed_cruise_cm.load();
            g.min_gndspeed_cm.load();
            g.throttle_cruise.load();
        }
    }

    if (hold_course != -1) {
        // recalc bearing error with hold_course;
        nav_bearing_cd = hold_course;
        // recalc bearing error
        calc_bearing_error();
    } else {
        update_crosstrack();
    }
    return false;
}

static bool verify_nav_wp()
{
    hold_course = -1;
    update_crosstrack();
    if (wp_distance <= (uint32_t)max(g.waypoint_radius,0)) {
        gcs_send_text_fmt(PSTR("Reached Waypoint #%i dist %um"),
                          (unsigned)nav_command_index,
                          (unsigned)get_distance(&current_loc, &next_WP));
        return true;
    }

    // have we circled around the waypoint?
    if (loiter_sum > 300) {
        gcs_send_text_P(SEVERITY_MEDIUM,PSTR("Missed WP"));
        return true;
    }

    // have we flown past the waypoint?
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_fmt(PSTR("Passed Waypoint #%i dist %um"),
                          (unsigned)nav_command_index,
                          (unsigned)get_distance(&current_loc, &next_WP));
        return true;
    }

    return false;
}

static bool verify_loiter_unlim()
{
    update_loiter();
    calc_bearing_error();
    return false;
}

static bool verify_loiter_time()
{
    update_loiter();
    calc_bearing_error();
    if ((millis() - loiter_time_ms) > loiter_time_max_ms) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("verify_nav: LOITER time complete"));
        return true;
    }
    return false;
}

static bool verify_loiter_turns()
{
    update_loiter();
    calc_bearing_error();
    if(loiter_sum > loiter_total) {
        loiter_total = 0;
        gcs_send_text_P(SEVERITY_LOW,PSTR("verify_nav: LOITER orbits complete"));
        // clear the command queue;
        return true;
    }
    return false;
}

static bool verify_RTL()
{
    if (wp_distance <= (uint32_t)max(g.waypoint_radius,0)) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Reached home"));
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
    condition_value  = next_nonnav_command.lat * 1000;          // convert to milliseconds
}

static void do_change_alt()
{
    condition_rate          = labs((int)next_nonnav_command.lat);
    condition_value         = next_nonnav_command.alt;
    if (condition_value < adjusted_altitude_cm()) {
        condition_rate = -condition_rate;
    }
    target_altitude_cm      = adjusted_altitude_cm() + (condition_rate / 10);                  // Divide by ten for 10Hz update
    next_WP.alt             = condition_value;                                                                  // For future nav calculations
    offset_altitude_cm      = 0;                                                                                        // For future nav calculations
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
    if ((unsigned)(millis() - condition_start) > (unsigned)condition_value) {
        condition_value         = 0;
        return true;
    }
    return false;
}

static bool verify_change_alt()
{
    if( (condition_rate>=0 && adjusted_altitude_cm() >= condition_value) || 
        (condition_rate<=0 && adjusted_altitude_cm() <= condition_value)) {
        condition_value = 0;
        return true;
    }
    target_altitude_cm += condition_rate / 10;
    return false;
}

static bool verify_within_distance()
{
    if (wp_distance < max(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

static void do_loiter_at_location()
{
    loiter_direction = 1;
    next_WP = current_loc;
}

static void do_jump()
{
    if (next_nonnav_command.lat == 0) {
        // the jump counter has reached zero - ignore
        gcs_send_text_fmt(PSTR("Jumps left: 0 - skipping"));
        return;
    }
    if (next_nonnav_command.p1 >= g.command_total) {
        gcs_send_text_fmt(PSTR("Skipping invalid jump to %i"), next_nonnav_command.p1);
        return;        
    }

    struct Location temp;
    temp = get_cmd_with_index(g.command_index);

    gcs_send_text_fmt(PSTR("Jump to WP %u. Jumps left: %d"),
                      (unsigned)next_nonnav_command.p1,
                      (int)next_nonnav_command.lat);
    if (next_nonnav_command.lat > 0) {
        // Decrement repeat counter
        temp.lat                        = next_nonnav_command.lat - 1;                                          
        set_cmd_with_index(temp, g.command_index);
    }

    nav_command_ID          = NO_COMMAND;
    next_nav_command.id     = NO_COMMAND;
    non_nav_command_ID      = NO_COMMAND;

    gcs_send_text_fmt(PSTR("setting command index: %i"), next_nonnav_command.p1);
    g.command_index.set_and_save(next_nonnav_command.p1);
    nav_command_index       = next_nonnav_command.p1;
    // Need to back "next_WP" up as it was set to the next waypoint following the jump
    next_WP = prev_WP;

    temp = get_cmd_with_index(g.command_index);

    next_nav_command = temp;
    nav_command_ID = next_nav_command.id;
    non_nav_command_index = g.command_index;
    non_nav_command_ID = WAIT_COMMAND;

    if (g.log_bitmask & MASK_LOG_CMD) {
        Log_Write_Cmd(g.command_index, &next_nav_command);
    }
    handle_process_nav_cmd();
}

static void do_change_speed()
{
    switch (next_nonnav_command.p1)
    {
    case 0:             // Airspeed
        if (next_nonnav_command.alt > 0) {
            g.airspeed_cruise_cm.set(next_nonnav_command.alt * 100);
            gcs_send_text_fmt(PSTR("Set airspeed %u m/s"), (unsigned)next_nonnav_command.alt);
        }
        break;
    case 1:             // Ground speed
        gcs_send_text_fmt(PSTR("Set groundspeed %u"), (unsigned)next_nonnav_command.alt);
        g.min_gndspeed_cm.set(next_nonnav_command.alt * 100);
        break;
    }

    if (next_nonnav_command.lat > 0) {
        gcs_send_text_fmt(PSTR("Set throttle %u"), (unsigned)next_nonnav_command.lat);
        g.throttle_cruise.set(next_nonnav_command.lat);
    }
}

static void do_set_home()
{
    if (next_nonnav_command.p1 == 1 && g_gps->status() == GPS::GPS_OK) {
        init_home();
    } else {
        home.id         = MAV_CMD_NAV_WAYPOINT;
        home.lng        = next_nonnav_command.lng;                                      // Lon * 10**7
        home.lat        = next_nonnav_command.lat;                                      // Lat * 10**7
        home.alt        = max(next_nonnav_command.alt, 0);
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
#if CONFIG_RELAY == ENABLED
    if (next_nonnav_command.p1 == 1) {
        relay.on();
    } else if (next_nonnav_command.p1 == 0) {
        relay.off();
    }else{
        relay.toggle();
    }
#endif
}

static void do_repeat_servo(uint8_t channel, uint16_t servo_value,
                            int16_t repeat, uint8_t delay_time)
{
    extern RC_Channel *rc_ch[8];
    channel = channel - 1;
    if (channel < 5 || channel > 8) {
        // not allowed
        return;
    }
    event_state.rc_channel = channel;
    event_state.type = EVENT_TYPE_SERVO;

    event_state.start_time_ms  = 0;
    event_state.delay_ms    = delay_time * 500UL;
    event_state.repeat      = repeat * 2;
    event_state.servo_value = servo_value;
    event_state.undo_value  = rc_ch[channel]->radio_trim;
    update_events();
}

static void do_repeat_relay()
{
    event_state.type = EVENT_TYPE_RELAY;
    event_state.start_time_ms  = 0;
    // /2 (half cycle time) * 1000 (convert to milliseconds)
    event_state.delay_ms        = next_nonnav_command.lat * 500.0;
    event_state.repeat          = next_nonnav_command.alt * 2;
    update_events();
}
#line 1 "./Firmware/Arduplane/commands_process.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// For changing active command mid-mission
//----------------------------------------
void change_command(uint8_t cmd_index)
{
    struct Location temp;

    if (cmd_index == 0) {
        init_commands();
        gcs_send_text_fmt(PSTR("Received Request - reset mission"));
        return;
    }

    temp = get_cmd_with_index(cmd_index);

    if (temp.id > MAV_CMD_NAV_LAST ) {
        gcs_send_text_fmt(PSTR("Cannot change to non-Nav cmd %u"), (unsigned)cmd_index);
    } else {
        gcs_send_text_fmt(PSTR("Received Request - jump to command #%i"),cmd_index);

        nav_command_ID          = NO_COMMAND;
        next_nav_command.id = NO_COMMAND;
        non_nav_command_ID      = NO_COMMAND;

        nav_command_index       = cmd_index - 1;
        g.command_index.set_and_save(cmd_index);
        update_commands();
    }
}

// called by 10 Hz loop
// --------------------
static void update_commands(void)
{
    if(control_mode == AUTO) {
        if(home_is_set == true && g.command_total > 1) {
            process_next_command();
        }
    }                                                                           // Other (eg GCS_Auto) modes may be implemented here
}

static void verify_commands(void)
{
    if(verify_nav_command()) {
        nav_command_ID = NO_COMMAND;
    }

    if(verify_condition_command()) {
        non_nav_command_ID = NO_COMMAND;
    }
}


static void process_next_command()
{
    // This function makes sure that we always have a current navigation command
    // and loads conditional or immediate commands if applicable

    struct Location temp;
    uint8_t old_index = nav_command_index;

    // these are Navigation/Must commands
    // ---------------------------------
    if (nav_command_ID == NO_COMMAND) {    // no current navigation command loaded
        temp.id = MAV_CMD_NAV_LAST;
        while(temp.id >= MAV_CMD_NAV_LAST && nav_command_index <= g.command_total) {
            nav_command_index++;
            temp = get_cmd_with_index(nav_command_index);
        }

        gcs_send_text_fmt(PSTR("Nav command index updated to #%i"),nav_command_index);

        if(nav_command_index > g.command_total) {
            // we are out of commands!
            gcs_send_text_P(SEVERITY_LOW,PSTR("out of commands!"));
            handle_no_commands();
        } else {
            next_nav_command = temp;
            nav_command_ID = next_nav_command.id;
            non_nav_command_index = NO_COMMAND;                                 // This will cause the next intervening non-nav command (if any) to be loaded
            non_nav_command_ID = NO_COMMAND;

            if (g.log_bitmask & MASK_LOG_CMD) {
                Log_Write_Cmd(g.command_index, &next_nav_command);
            }
            handle_process_nav_cmd();
        }
    }

    // these are Condition/May and Do/Now commands
    // -------------------------------------------
    if (non_nav_command_index == NO_COMMAND) {                  // If the index is NO_COMMAND then we have just loaded a nav command
        non_nav_command_index = old_index + 1;
        //gcs_send_text_fmt(PSTR("Non-Nav command index #%i"),non_nav_command_index);
    } else if (non_nav_command_ID == NO_COMMAND) {      // If the ID is NO_COMMAND then we have just completed a non-nav command
        non_nav_command_index++;
    }

    //gcs_send_text_fmt(PSTR("Nav command index #%i"),nav_command_index);
    //gcs_send_text_fmt(PSTR("Non-Nav command index #%i"),non_nav_command_index);
    //gcs_send_text_fmt(PSTR("Non-Nav command ID #%i"),non_nav_command_ID);
    if (nav_command_index <= (int)g.command_total && non_nav_command_ID == NO_COMMAND) {
        temp = get_cmd_with_index(non_nav_command_index);
        if (temp.id <= MAV_CMD_NAV_LAST) {                       
            // The next command is a nav command.  No non-nav commands to do
            g.command_index.set_and_save(nav_command_index);
            non_nav_command_index = nav_command_index;
            non_nav_command_ID = WAIT_COMMAND;
            gcs_send_text_fmt(PSTR("Non-Nav command ID updated to #%i idx=%u"),
                              (unsigned)non_nav_command_ID, 
                              (unsigned)non_nav_command_index);

        } else {                                                                        
            // The next command is a non-nav command.  Prepare to execute it.
            g.command_index.set_and_save(non_nav_command_index);
            next_nonnav_command = temp;
            non_nav_command_ID = next_nonnav_command.id;
            gcs_send_text_fmt(PSTR("(2) Non-Nav command ID updated to #%i idx=%u"),
                              (unsigned)non_nav_command_ID, (unsigned)non_nav_command_index);

            if (g.log_bitmask & MASK_LOG_CMD) {
                Log_Write_Cmd(g.command_index, &next_nonnav_command);
            }

            process_non_nav_command();
        }
    }
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
        if (non_nav_command_ID != WAIT_COMMAND) {
            non_nav_command_ID = NO_COMMAND;
        }
    }
}
#line 1 "./Firmware/Arduplane/compat.pde"


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

#line 1 "./Firmware/Arduplane/control_modes.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


static void read_control_switch()
{
    static bool switch_debouncer;
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

        if (switch_debouncer == false) {
            // this ensures that mode switches only happen if the
            // switch changes for 2 reads. This prevents momentary
            // spikes in the mode control channel from causing a mode
            // switch
            switch_debouncer = true;
            return;
        }

        set_mode((enum FlightMode)(flight_modes[switchPosition].get()));

        oldSwitchPosition = switchPosition;
        prev_WP = current_loc;
    }

    if (g.reset_mission_chan != 0 &&
        hal.rcin->read(g.reset_mission_chan-1) > RESET_SWITCH_CHAN_PWM) {
        // reset to first waypoint in mission
        prev_WP = current_loc;
        change_command(0);
    }

    switch_debouncer = false;

    if (g.inverted_flight_ch != 0) {
        // if the user has configured an inverted flight channel, then
        // fly upside down when that channel goes above INVERTED_FLIGHT_PWM
        inverted_flight = (control_mode != MANUAL && hal.rcin->read(g.inverted_flight_ch-1) > INVERTED_FLIGHT_PWM);
    }
}

static uint8_t readSwitch(void){
    uint16_t pulsewidth = hal.rcin->read(g.flight_mode_channel - 1);
    if (pulsewidth <= 910 || pulsewidth >= 2090) return 255;            // This is an error condition
    if (pulsewidth > 1230 && pulsewidth <= 1360) return 1;
    if (pulsewidth > 1360 && pulsewidth <= 1490) return 2;
    if (pulsewidth > 1490 && pulsewidth <= 1620) return 3;
    if (pulsewidth > 1620 && pulsewidth <= 1749) return 4;              // Software Manual
    if (pulsewidth >= 1750) return 5;                                                           // Hardware Manual
    return 0;
}

static void reset_control_switch()
{
    oldSwitchPosition = 0;
    read_control_switch();
}

#line 1 "./Firmware/Arduplane/events.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


static void failsafe_short_on_event(int16_t fstype)
{
    // This is how to handle a short loss of control signal failsafe.
    failsafe = fstype;
    ch3_failsafe_timer = millis();
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Short event on, "));
    switch(control_mode)
    {
    case MANUAL:
    case STABILIZE:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
    case TRAINING:
        set_mode(CIRCLE);
        break;

    case AUTO:
    case GUIDED:
    case LOITER:
        if(g.short_fs_action == 1) {
            set_mode(RTL);
        }
        break;

    case CIRCLE:
    case RTL:
    default:
        break;
    }
    gcs_send_text_fmt(PSTR("flight mode = %u"), (unsigned)control_mode);
}

static void failsafe_long_on_event(int16_t fstype)
{
    // This is how to handle a long loss of control signal failsafe.
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Long event on, "));
    //  If the GCS is locked up we allow control to revert to RC
    hal.rcin->clear_overrides();
    failsafe = fstype;
    switch(control_mode)
    {
    case MANUAL:
    case STABILIZE:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
    case TRAINING:
    case CIRCLE:
        set_mode(RTL);
        break;

    case AUTO:
    case GUIDED:
    case LOITER:
        if(g.long_fs_action == 1) {
            set_mode(RTL);
        }
        break;

    case RTL:
    default:
        break;
    }
    gcs_send_text_fmt(PSTR("flight mode = %u"), (unsigned)control_mode);
}

static void failsafe_short_off_event()
{
    // We're back in radio contact
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Short event off"));
    failsafe = FAILSAFE_NONE;

    // re-read the switch so we can return to our preferred mode
    // --------------------------------------------------------
    if (control_mode == CIRCLE ||
        (g.short_fs_action == 1 && control_mode == RTL)) {
        reset_control_switch();
    }
}

#if BATTERY_EVENT == ENABLED
void low_battery_event(void)
{
    gcs_send_text_P(SEVERITY_HIGH,PSTR("Low Battery!"));
    set_mode(RTL);
    g.throttle_cruise = THROTTLE_CRUISE;
}
#endif

////////////////////////////////////////////////////////////////////////////////
// repeating event control

/*
  update state for MAV_CMD_DO_REPEAT_SERVO and MAV_CMD_DO_REPEAT_RELAY
*/
static void update_events(void)
{
    if (event_state.repeat == 0 || (millis() - event_state.start_time_ms) < event_state.delay_ms) {
        return;
    }

    // event_repeat = -1 means repeat forever
    if (event_state.repeat != 0) {
        event_state.start_time_ms = millis();

        switch (event_state.type) {
        case EVENT_TYPE_SERVO:
            hal.rcout->enable_ch(event_state.rc_channel);
            if (event_state.repeat & 1) {
                hal.rcout->write(event_state.rc_channel, event_state.undo_value);                 
            } else {
                hal.rcout->write(event_state.rc_channel, event_state.servo_value);                 
            }
            break;

        case EVENT_TYPE_RELAY:
            relay.toggle();
            break;
        }

        if (event_state.repeat > 0) {
            event_state.repeat--;
        }
    }
}
#line 1 "./Firmware/Arduplane/failsafe.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  failsafe support
 *  Andrew Tridgell, December 2011
 */

/*
 *  our failsafe strategy is to detect main loop lockup and switch to
 *  passing inputs straight from the RC inputs to RC outputs.
 */

/*
 *  this failsafe_check function is called from the core timer interrupt
 *  at 1kHz.
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
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_aileron_with_input, true);
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_elevator_with_input, true);
    }
}
#line 1 "./Firmware/Arduplane/geofence.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  geo-fencing support
 *  Andrew Tridgell, December 2011
 */

#if GEOFENCE_ENABLED == ENABLED

/*
 *  The state of geo-fencing. This structure is dynamically allocated
 *  the first time it is used. This means we only pay for the pointer
 *  and not the structure on systems where geo-fencing is not being
 *  used.
 *
 *  We store a copy of the boundary in memory as we need to access it
 *  very quickly at runtime
 */
static struct geofence_state {
    uint8_t num_points;
    bool boundary_uptodate;
    bool fence_triggered;
    uint16_t breach_count;
    uint8_t breach_type;
    uint32_t breach_time;
    uint8_t old_switch_position;
    /* point 0 is the return point */
    Vector2l boundary[MAX_FENCEPOINTS];
} *geofence_state;


/*
 *  fence boundaries fetch/store
 */
static Vector2l get_fence_point_with_index(unsigned i)
{
    uint16_t mem;
    Vector2l ret;

    if (i > (unsigned)g.fence_total) {
        return Vector2l(0,0);
    }

    // read fence point
    mem = FENCE_START_BYTE + (i * FENCE_WP_SIZE);
    ret.x = hal.storage->read_dword(mem);
    mem += sizeof(uint32_t);
    ret.y = hal.storage->read_dword(mem);

    return ret;
}

// save a fence point
static void set_fence_point_with_index(Vector2l &point, unsigned i)
{
    uint16_t mem;

    if (i >= (unsigned)g.fence_total.get()) {
        // not allowed
        return;
    }

    mem = FENCE_START_BYTE + (i * FENCE_WP_SIZE);

    hal.storage->write_dword(mem, point.x);
    mem += sizeof(uint32_t);
    hal.storage->write_dword(mem, point.y);

    if (geofence_state != NULL) {
        geofence_state->boundary_uptodate = false;
    }
}

/*
 *  allocate and fill the geofence state structure
 */
static void geofence_load(void)
{
    uint8_t i;

    if (geofence_state == NULL) {
        if (memcheck_available_memory() < 512 + sizeof(struct geofence_state)) {
            // too risky to enable as we could run out of stack
            goto failed;
        }
        geofence_state = (struct geofence_state *)calloc(1, sizeof(struct geofence_state));
        if (geofence_state == NULL) {
            // not much we can do here except disable it
            goto failed;
        }
    }

    if (g.fence_total <= 0) {
        g.fence_total.set(0);
        return;
    }

    for (i=0; i<g.fence_total; i++) {
        geofence_state->boundary[i] = get_fence_point_with_index(i);
    }
    geofence_state->num_points = i;

    if (!Polygon_complete(&geofence_state->boundary[1], geofence_state->num_points-1)) {
        // first point and last point must be the same
        goto failed;
    }
    if (Polygon_outside(geofence_state->boundary[0], &geofence_state->boundary[1], geofence_state->num_points-1)) {
        // return point needs to be inside the fence
        goto failed;
    }

    geofence_state->boundary_uptodate = true;
    geofence_state->fence_triggered = false;

    gcs_send_text_P(SEVERITY_LOW,PSTR("geo-fence loaded"));
    gcs_send_message(MSG_FENCE_STATUS);
    return;

failed:
    g.fence_action.set(FENCE_ACTION_NONE);
    gcs_send_text_P(SEVERITY_HIGH,PSTR("geo-fence setup error"));
}

/*
 *  return true if geo-fencing is enabled
 */
static bool geofence_enabled(void)
{
    if (g.fence_action == FENCE_ACTION_NONE ||
        g.fence_total < 5 ||
        (g.fence_action != FENCE_ACTION_REPORT &&
         (g.fence_channel == 0 ||
          hal.rcin->read(g.fence_channel-1) < FENCE_ENABLE_PWM))) {
        // geo-fencing is disabled
        if (geofence_state != NULL) {
            // re-arm for when the channel trigger is switched on
            geofence_state->fence_triggered = false;
        }
        return false;
    }

    return true;
}


/*
 *  return true if we have breached the geo-fence minimum altiude
 */
static bool geofence_check_minalt(void)
{
    if (g.fence_maxalt <= g.fence_minalt) {
        return false;
    }
    if (g.fence_minalt == 0) {
        return false;
    }
    return (adjusted_altitude_cm() < (g.fence_minalt*100.0) + home.alt);
}

/*
 *  return true if we have breached the geo-fence maximum altiude
 */
static bool geofence_check_maxalt(void)
{
    if (g.fence_maxalt <= g.fence_minalt) {
        return false;
    }
    if (g.fence_maxalt == 0) {
        return false;
    }
    return (adjusted_altitude_cm() > (g.fence_maxalt*100.0) + home.alt);
}


/*
 *  check if we have breached the geo-fence
 */
static void geofence_check(bool altitude_check_only)
{
    if (!geofence_enabled()) {
        // switch back to the chosen control mode if still in
        // GUIDED to the return point
        if (geofence_state != NULL &&
            g.fence_action == FENCE_ACTION_GUIDED &&
            g.fence_channel != 0 &&
            control_mode == GUIDED &&
            g.fence_total >= 5 &&
            geofence_state->boundary_uptodate &&
            geofence_state->old_switch_position == oldSwitchPosition &&
            guided_WP.lat == geofence_state->boundary[0].x &&
            guided_WP.lng == geofence_state->boundary[0].y) {
            geofence_state->old_switch_position = 0;
            reset_control_switch();
        }
        return;
    }

    /* allocate the geo-fence state if need be */
    if (geofence_state == NULL || !geofence_state->boundary_uptodate) {
        geofence_load();
        if (!geofence_enabled()) {
            // may have been disabled by load
            return;
        }
    }

    bool outside = false;
    uint8_t breach_type = FENCE_BREACH_NONE;
    struct Location loc;

    if (geofence_check_minalt()) {
        outside = true;
        breach_type = FENCE_BREACH_MINALT;
    } else if (geofence_check_maxalt()) {
        outside = true;
        breach_type = FENCE_BREACH_MAXALT;
    } else if (!altitude_check_only && ahrs.get_position(&loc)) {
        Vector2l location;
        location.x = loc.lat;
        location.y = loc.lng;
        outside = Polygon_outside(location, &geofence_state->boundary[1], geofence_state->num_points-1);
        if (outside) {
            breach_type = FENCE_BREACH_BOUNDARY;
        }
    }

    if (!outside) {
        if (geofence_state->fence_triggered && !altitude_check_only) {
            // we have moved back inside the fence
            geofence_state->fence_triggered = false;
            gcs_send_text_P(SEVERITY_LOW,PSTR("geo-fence OK"));
 #if FENCE_TRIGGERED_PIN > 0
            digitalWrite(FENCE_TRIGGERED_PIN, LOW);
 #endif
            gcs_send_message(MSG_FENCE_STATUS);
        }
        // we're inside, all is good with the world
        return;
    }

    // we are outside the fence
    if (geofence_state->fence_triggered &&
        (control_mode == GUIDED || g.fence_action == FENCE_ACTION_REPORT)) {
        // we have already triggered, don't trigger again until the
        // user disables/re-enables using the fence channel switch
        return;
    }

    // we are outside, and have not previously triggered.
    geofence_state->fence_triggered = true;
    geofence_state->breach_count++;
    geofence_state->breach_time = millis();
    geofence_state->breach_type = breach_type;

 #if FENCE_TRIGGERED_PIN > 0
    digitalWrite(FENCE_TRIGGERED_PIN, HIGH);
 #endif

    gcs_send_text_P(SEVERITY_LOW,PSTR("geo-fence triggered"));
    gcs_send_message(MSG_FENCE_STATUS);

    // see what action the user wants
    switch (g.fence_action) {
    case FENCE_ACTION_REPORT:
        break;

    case FENCE_ACTION_GUIDED:
        // fly to the return point, with an altitude half way between
        // min and max
        if (g.fence_minalt >= g.fence_maxalt) {
            // invalid min/max, use RTL_altitude
            guided_WP.alt = home.alt + g.RTL_altitude_cm;
        } else {
            guided_WP.alt = home.alt + 100.0*(g.fence_minalt + g.fence_maxalt)/2;
        }
        guided_WP.id = 0;
        guided_WP.p1  = 0;
        guided_WP.options = 0;
        guided_WP.lat = geofence_state->boundary[0].x;
        guided_WP.lng = geofence_state->boundary[0].y;

        geofence_state->old_switch_position = oldSwitchPosition;

        if (control_mode == MANUAL && g.auto_trim) {
            // make sure we don't auto trim the surfaces on this change
            control_mode = STABILIZE;
        }

        set_mode(GUIDED);
        break;
    }

}

/*
 *  return true if geofencing allows stick mixing. When we have
 *  triggered failsafe and are in GUIDED mode then stick mixing is
 *  disabled. Otherwise the aircraft may not be able to recover from
 *  a breach of the geo-fence
 */
static bool geofence_stickmixing(void) {
    if (geofence_enabled() &&
        geofence_state != NULL &&
        geofence_state->fence_triggered &&
        control_mode == GUIDED) {
        // don't mix in user input
        return false;
    }
    // normal mixing rules
    return true;
}

/*
 *
 */
static void geofence_send_status(mavlink_channel_t chan)
{
    if (geofence_enabled() && geofence_state != NULL) {
        mavlink_msg_fence_status_send(chan,
                                      (int8_t)geofence_state->fence_triggered,
                                      geofence_state->breach_count,
                                      geofence_state->breach_type,
                                      geofence_state->breach_time);
    }
}

// public function for use in failsafe modules
bool geofence_breached(void)
{
    return geofence_state ? geofence_state->fence_triggered : false;
}


#else // GEOFENCE_ENABLED

static void geofence_check(bool altitude_check_only) {
}
static bool geofence_stickmixing(void) {
    return true;
}
static bool geofence_enabled(void) {
    return false;
}

#endif // GEOFENCE_ENABLED
#line 1 "./Firmware/Arduplane/navigation.pde"
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

    if(next_WP.lat == 0) {
        return;
    }

    // waypoint distance from plane
    // ----------------------------
    wp_distance = get_distance(&current_loc, &next_WP);

    if (wp_distance < 0) {
        gcs_send_text_P(SEVERITY_HIGH,PSTR("WP error - distance < 0"));
        return;
    }

    // target_bearing is where we should be heading
    // --------------------------------------------
    target_bearing_cd       = get_bearing_cd(&current_loc, &next_WP);

    // nav_bearing will includes xtrac correction
    // ------------------------------------------
    nav_bearing_cd = target_bearing_cd;

    // check if we have missed the WP
    loiter_delta = (target_bearing_cd - old_target_bearing_cd)/100;

    // reset the old value
    old_target_bearing_cd = target_bearing_cd;

    // wrap values
    if (loiter_delta > 180) loiter_delta -= 360;
    if (loiter_delta < -180) loiter_delta += 360;
    loiter_sum += abs(loiter_delta);

    // control mode specific updates to nav_bearing
    // --------------------------------------------
    update_navigation();
}


#if 0
// Disabled for now
void calc_distance_error()
{
    distance_estimate       += (float)g_gps->ground_speed * .0002f * cosf(radians(bearing_error_cd * .01f));
    distance_estimate       -= DST_EST_GAIN * (float)(distance_estimate - GPS_wp_distance);
    wp_distance             = max(distance_estimate,10);
}
#endif

static void calc_airspeed_errors()
{
    float aspeed_cm = airspeed.get_airspeed_cm();

    // Normal airspeed target
    target_airspeed_cm = g.airspeed_cruise_cm;

    // FBW_B airspeed target
    if (control_mode == FLY_BY_WIRE_B) {
        target_airspeed_cm = ((int)(g.flybywire_airspeed_max -
                                    g.flybywire_airspeed_min) *
                              g.channel_throttle.servo_out) +
                             ((int)g.flybywire_airspeed_min * 100);
    }

    // Set target to current airspeed + ground speed undershoot,
    // but only when this is faster than the target airspeed commanded
    // above.
    if (control_mode >= FLY_BY_WIRE_B && (g.min_gndspeed_cm > 0)) {
        int32_t min_gnd_target_airspeed = aspeed_cm + groundspeed_undershoot;
        if (min_gnd_target_airspeed > target_airspeed_cm)
            target_airspeed_cm = min_gnd_target_airspeed;
    }

    // Bump up the target airspeed based on throttle nudging
    if (control_mode >= AUTO && airspeed_nudge_cm > 0) {
        target_airspeed_cm += airspeed_nudge_cm;
    }

    // Apply airspeed limit
    if (target_airspeed_cm > (g.flybywire_airspeed_max * 100))
        target_airspeed_cm = (g.flybywire_airspeed_max * 100);

    airspeed_error_cm = target_airspeed_cm - aspeed_cm;
    airspeed_energy_error = ((target_airspeed_cm * target_airspeed_cm) - (aspeed_cm*aspeed_cm))*0.00005;
}

static void calc_gndspeed_undershoot()
{
    // Function is overkill, but here in case we want to add filtering
    // later
    if (g_gps && g_gps->status() == GPS::GPS_OK) {
        groundspeed_undershoot = (g.min_gndspeed_cm > 0) ? (g.min_gndspeed_cm - g_gps->ground_speed) : 0;
    }
}

static void calc_bearing_error()
{
    bearing_error_cd = nav_bearing_cd - ahrs.yaw_sensor;
    bearing_error_cd = wrap_180_cd(bearing_error_cd);
}

static void calc_altitude_error()
{
    if (control_mode == AUTO && offset_altitude_cm != 0) {
        // limit climb rates
        target_altitude_cm = next_WP.alt - (offset_altitude_cm*((float)(wp_distance-30) / (float)(wp_totalDistance-30)));

        // stay within a certain range
        if(prev_WP.alt > next_WP.alt) {
            target_altitude_cm = constrain_int32(target_altitude_cm, next_WP.alt, prev_WP.alt);
        }else{
            target_altitude_cm = constrain_int32(target_altitude_cm, prev_WP.alt, next_WP.alt);
        }
    } else if (non_nav_command_ID != MAV_CMD_CONDITION_CHANGE_ALT) {
        target_altitude_cm = next_WP.alt;
    }

    altitude_error_cm       = target_altitude_cm - adjusted_altitude_cm();
}

static int32_t wrap_360_cd(int32_t error)
{
    if (error > 36000) error -= 36000;
    if (error < 0) error += 36000;
    return error;
}

static int32_t wrap_180_cd(int32_t error)
{
    if (error > 18000) error -= 36000;
    if (error < -18000) error += 36000;
    return error;
}

static void update_loiter()
{
    float power;

    if(wp_distance <= (uint32_t)max(g.loiter_radius,0)) {
        power = float(wp_distance) / float(g.loiter_radius);
        power = constrain(power, 0.5, 1);
        nav_bearing_cd += 9000.0 * (2.0 + power) * loiter_direction;
    } else if(wp_distance < (uint32_t)max((g.loiter_radius + LOITER_RANGE),0)) {
        power = -((float)(wp_distance - g.loiter_radius - LOITER_RANGE) / LOITER_RANGE);
        power = constrain(power, 0.5, 1);                               //power = constrain(power, 0, 1);
        nav_bearing_cd -= power * 9000 * loiter_direction;
    } else{
        update_crosstrack();
        loiter_time_ms = millis();                              // keep start time for loiter updating till we get within LOITER_RANGE of orbit

    }
/*
 *       if (wp_distance < g.loiter_radius){
 *               nav_bearing += 9000;
 *       }else{
 *               nav_bearing -= 100 * M_PI / 180 * asinf(g.loiter_radius / wp_distance);
 *       }
 *
 *       update_crosstrack();
 */
    nav_bearing_cd = wrap_360_cd(nav_bearing_cd);
}

static void update_crosstrack(void)
{
    // if we are using a compass for navigation, then adjust the
    // heading to account for wind
    if (g.crosstrack_use_wind && compass.use_for_yaw()) {
        Vector3f wind = ahrs.wind_estimate();
        Vector2f wind2d = Vector2f(wind.x, wind.y);
        float speed;
        if (ahrs.airspeed_estimate(&speed)) {
            Vector2f nav_vector = Vector2f(cosf(radians(nav_bearing_cd*0.01)), sinf(radians(nav_bearing_cd*0.01))) * speed;
            Vector2f nav_adjusted = nav_vector - wind2d;
            nav_bearing_cd = degrees(atan2f(nav_adjusted.y, nav_adjusted.x)) * 100;
        }
    }

    // Crosstrack Error
    // ----------------
    // If we are too far off or too close we don't do track following
    if (wp_totalDistance >= (uint32_t)max(g.crosstrack_min_distance,0) &&
        abs(wrap_180_cd(target_bearing_cd - crosstrack_bearing_cd)) < 4500) {
        // Meters we are off track line
        crosstrack_error = sinf(radians((target_bearing_cd - crosstrack_bearing_cd) * 0.01)) * wp_distance;               
        nav_bearing_cd += constrain_int32(crosstrack_error * g.crosstrack_gain, -g.crosstrack_entry_angle.get(), g.crosstrack_entry_angle.get());
        nav_bearing_cd = wrap_360_cd(nav_bearing_cd);
    }

}

static void reset_crosstrack()
{
    crosstrack_bearing_cd   = get_bearing_cd(&prev_WP, &next_WP);       // Used for track following
}

#line 1 "./Firmware/Arduplane/radio.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------
static uint8_t failsafeCounter = 0;                // we wait a second to take over the throttle and send the plane circling


extern RC_Channel* rc_ch[8];

static void init_rc_in()
{
    // set rc channel ranges
    g.channel_roll.set_angle(SERVO_MAX);
    g.channel_pitch.set_angle(SERVO_MAX);
    g.channel_rudder.set_angle(SERVO_MAX);
    g.channel_throttle.set_range(0, 100);

    // set rc dead zones
    g.channel_roll.set_dead_zone(60);
    g.channel_pitch.set_dead_zone(60);
    g.channel_rudder.set_dead_zone(60);
    g.channel_throttle.set_dead_zone(6);

    //g.channel_roll.dead_zone  = 60;
    //g.channel_pitch.dead_zone     = 60;
    //g.channel_rudder.dead_zone    = 60;
    //g.channel_throttle.dead_zone = 6;

    rc_ch[CH_1] = &g.channel_roll;
    rc_ch[CH_2] = &g.channel_pitch;
    rc_ch[CH_3] = &g.channel_throttle;
    rc_ch[CH_4] = &g.channel_rudder;
    rc_ch[CH_5] = &g.rc_5;
    rc_ch[CH_6] = &g.rc_6;
    rc_ch[CH_7] = &g.rc_7;
    rc_ch[CH_8] = &g.rc_8;

    //set auxiliary ranges
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_9, &g.rc_10, &g.rc_11);
#else
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8);
#endif
}

static void init_rc_out()
{
    hal.rcout->enable_ch(CH_1);
    hal.rcout->enable_ch(CH_2);
    hal.rcout->enable_ch(CH_3);
    hal.rcout->enable_ch(CH_4);
    enable_aux_servos();

    // Initialization of servo outputs
    hal.rcout->write(CH_1,   g.channel_roll.radio_trim);
    hal.rcout->write(CH_2,   g.channel_pitch.radio_trim);
    hal.rcout->write(CH_3,   g.channel_throttle.radio_min);
    hal.rcout->write(CH_4,   g.channel_rudder.radio_trim);

    hal.rcout->write(CH_5,   g.rc_5.radio_trim);
    hal.rcout->write(CH_6,   g.rc_6.radio_trim);
    hal.rcout->write(CH_7,   g.rc_7.radio_trim);
    hal.rcout->write(CH_8,   g.rc_8.radio_trim);

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    hal.rcout->write(CH_9,   g.rc_9.radio_trim);
    hal.rcout->write(CH_10,  g.rc_10.radio_trim);
    hal.rcout->write(CH_11,  g.rc_11.radio_trim);
#endif
}

static void read_radio()
{
    ch1_temp = hal.rcin->read(CH_ROLL);
    ch2_temp = hal.rcin->read(CH_PITCH);
    uint16_t pwm_roll, pwm_pitch;

    if (g.mix_mode == 0) {
        pwm_roll = ch1_temp;
        pwm_pitch = ch2_temp;
    }else{
        pwm_roll = BOOL_TO_SIGN(g.reverse_elevons) * (BOOL_TO_SIGN(g.reverse_ch2_elevon) * int(ch2_temp - elevon2_trim) - BOOL_TO_SIGN(g.reverse_ch1_elevon) * int(ch1_temp - elevon1_trim)) / 2 + 1500;
        pwm_pitch = (BOOL_TO_SIGN(g.reverse_ch2_elevon) * int(ch2_temp - elevon2_trim) + BOOL_TO_SIGN(g.reverse_ch1_elevon) * int(ch1_temp - elevon1_trim)) / 2 + 1500;
    }
    
    if (control_mode == TRAINING) {
        // in training mode we don't want to use a deadzone, as we
        // want manual pass through when not exceeding attitude limits
        g.channel_roll.set_pwm_no_deadzone(pwm_roll);
        g.channel_pitch.set_pwm_no_deadzone(pwm_pitch);
        g.channel_throttle.set_pwm_no_deadzone(hal.rcin->read(CH_3));
        g.channel_rudder.set_pwm_no_deadzone(hal.rcin->read(CH_4));
    } else {
        g.channel_roll.set_pwm(pwm_roll);
        g.channel_pitch.set_pwm(pwm_pitch);
        g.channel_throttle.set_pwm(hal.rcin->read(CH_3));
        g.channel_rudder.set_pwm(hal.rcin->read(CH_4));
    }

    g.rc_5.set_pwm(hal.rcin->read(CH_5));
    g.rc_6.set_pwm(hal.rcin->read(CH_6));
    g.rc_7.set_pwm(hal.rcin->read(CH_7));
    g.rc_8.set_pwm(hal.rcin->read(CH_8));

    control_failsafe(g.channel_throttle.radio_in);

    g.channel_throttle.servo_out = g.channel_throttle.control_in;

    if (g.throttle_nudge && g.channel_throttle.servo_out > 50) {
        float nudge = (g.channel_throttle.servo_out - 50) * 0.02;
        if (alt_control_airspeed()) {
            airspeed_nudge_cm = (g.flybywire_airspeed_max * 100 - g.airspeed_cruise_cm) * nudge;
        } else {
            throttle_nudge = (g.throttle_max - g.throttle_cruise) * nudge;
        }
    } else {
        airspeed_nudge_cm = 0;
        throttle_nudge = 0;
    }

    /*
     *  cliSerial->printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d \n"),
     *                       (int)g.rc_1.control_in,
     *                       (int)g.rc_2.control_in,
     *                       (int)g.rc_3.control_in,
     *                       (int)g.rc_4.control_in);
     */
}

static void control_failsafe(uint16_t pwm)
{
    if(g.throttle_fs_enabled == 0)
        return;

    // Check for failsafe condition based on loss of GCS control
    if (rc_override_active) {
        if (millis() - last_heartbeat_ms > FAILSAFE_SHORT_TIME) {
            ch3_failsafe = true;
        } else {
            ch3_failsafe = false;
        }

        //Check for failsafe and debounce funky reads
    } else if (g.throttle_fs_enabled) {
        if (pwm < (unsigned)g.throttle_fs_value) {
            // we detect a failsafe from radio
            // throttle has dropped below the mark
            failsafeCounter++;
            if (failsafeCounter == 9) {
                gcs_send_text_fmt(PSTR("MSG FS ON %u"), (unsigned)pwm);
            }else if(failsafeCounter == 10) {
                ch3_failsafe = true;
            }else if (failsafeCounter > 10) {
                failsafeCounter = 11;
            }

        }else if(failsafeCounter > 0) {
            // we are no longer in failsafe condition
            // but we need to recover quickly
            failsafeCounter--;
            if (failsafeCounter > 3) {
                failsafeCounter = 3;
            }
            if (failsafeCounter == 1) {
                gcs_send_text_fmt(PSTR("MSG FS OFF %u"), (unsigned)pwm);
            } else if(failsafeCounter == 0) {
                ch3_failsafe = false;
            }
        }
    }
}

static void trim_control_surfaces()
{
    read_radio();
    // Store control surface trim values
    // ---------------------------------
    if(g.mix_mode == 0) {
        if (g.channel_roll.radio_in != 0) {
            g.channel_roll.radio_trim = g.channel_roll.radio_in;
        }
        if (g.channel_pitch.radio_in != 0) {
            g.channel_pitch.radio_trim = g.channel_pitch.radio_in;
        }

        // the secondary aileron/elevator is trimmed only if it has a
        // corresponding transmitter input channel, which k_aileron
        // doesn't have
        RC_Channel_aux::set_radio_trim(RC_Channel_aux::k_aileron_with_input);
        RC_Channel_aux::set_radio_trim(RC_Channel_aux::k_elevator_with_input);
    } else{
        if (ch1_temp != 0) {
            elevon1_trim = ch1_temp;
        }
        if (ch2_temp != 0) {
            elevon2_trim = ch2_temp;
        }
        //Recompute values here using new values for elevon1_trim and elevon2_trim
        //We cannot use radio_in[CH_ROLL] and radio_in[CH_PITCH] values from read_radio() because the elevon trim values have changed
        uint16_t center                         = 1500;
        g.channel_roll.radio_trim       = center;
        g.channel_pitch.radio_trim      = center;
    }
    if (g.channel_rudder.radio_in != 0) {
        g.channel_rudder.radio_trim = g.channel_rudder.radio_in;
    }

    // save to eeprom
    g.channel_roll.save_eeprom();
    g.channel_pitch.save_eeprom();
    g.channel_rudder.save_eeprom();
}

static void trim_radio()
{
    for (uint8_t y = 0; y < 30; y++) {
        read_radio();
    }

    trim_control_surfaces();
}
#line 1 "./Firmware/Arduplane/sensors.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// filter altitude from the barometer with a low pass filter
static LowPassFilterInt32 altitude_filter;


static void init_barometer(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));    
    barometer.calibrate();

    // filter at 100ms sampling, with 0.7Hz cutoff frequency
    altitude_filter.set_cutoff_frequency(0.1, 0.7);

    ahrs.set_barometer(&barometer);
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// read the barometer and return the updated altitude in centimeters
// above the calibration altitude
static int32_t read_barometer(void)
{
    barometer.read();
    return altitude_filter.apply(barometer.get_altitude() * 100.0);
}

// in M/S * 100
static void read_airspeed(void)
{
    airspeed.read();
    calc_airspeed_errors();
}

static void zero_airspeed(void)
{
    airspeed.calibrate();
    gcs_send_text_P(SEVERITY_LOW,PSTR("zero airspeed calibrated"));
}

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

#if BATTERY_EVENT == ENABLED
    if(battery_voltage1 < LOW_VOLTAGE) low_battery_event();
    if(g.battery_monitoring == 4 && current_total1 > g.pack_capacity) low_battery_event();
#endif
}


// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    rssi_analog_source->set_pin(g.rssi_pin);
    float ret = rssi_analog_source->read_average();
    receiver_rssi = constrain_int16(ret, 0, 255);
}


/*
  return current_loc.alt adjusted for ALT_OFFSET
  This is useful during long flights to account for barometer changes
  from the GCS, or to adjust the flying height of a long mission
 */
static int32_t adjusted_altitude_cm(void)
{
    return current_loc.alt - (g.alt_offset*100);
}
#line 1 "./Firmware/Arduplane/setup.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// Functions called from the setup menu
static int8_t   setup_radio                             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_show                              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_factory                   (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_flightmodes               (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_level                             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_accel_scale                       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_erase                             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_compass                   (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_declination               (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_batt_monitor              (uint8_t argc, const Menu::arg *argv);

// Command/function table for the setup menu
static const struct Menu::command setup_menu_commands[] PROGMEM = {
    // command			function called
    // =======          ===============
    {"reset",                       setup_factory},
    {"radio",                       setup_radio},
    {"modes",                       setup_flightmodes},
    {"level",                       setup_level},
#if !defined( __AVR_ATmega1280__ )
    {"accel",                       setup_accel_scale},
#endif
    {"compass",                     setup_compass},
    {"declination",         setup_declination},
    {"battery",                     setup_batt_monitor},
    {"show",                        setup_show},
    {"erase",                       setup_erase},
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
    report_flight_modes();
    report_ins();
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
    int c;

    cliSerial->printf_P(PSTR("\nType 'Y' and hit Enter to perform factory reset, any other key to abort: "));

    do {
        c = cliSerial->read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);
    AP_Param::erase_all();
    cliSerial->printf_P(PSTR("\nFACTORY RESET complete - please reset APM to continue"));

    //default_flight_modes();   // This will not work here.  Replacement code located in init_ardupilot()

    for (;; ) {
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

    for(i = 0; i < 100; i++) {
        delay(20);
        read_radio();
    }


    if(g.channel_roll.radio_in < 500) {
        while(1) {
            cliSerial->printf_P(PSTR("\nNo radio; Check connectors."));
            delay(1000);
            // stop here
        }
    }

    g.channel_roll.radio_min                = g.channel_roll.radio_in;
    g.channel_pitch.radio_min               = g.channel_pitch.radio_in;
    g.channel_throttle.radio_min    = g.channel_throttle.radio_in;
    g.channel_rudder.radio_min              = g.channel_rudder.radio_in;
    g.rc_5.radio_min = g.rc_5.radio_in;
    g.rc_6.radio_min = g.rc_6.radio_in;
    g.rc_7.radio_min = g.rc_7.radio_in;
    g.rc_8.radio_min = g.rc_8.radio_in;

    g.channel_roll.radio_max                = g.channel_roll.radio_in;
    g.channel_pitch.radio_max               = g.channel_pitch.radio_in;
    g.channel_throttle.radio_max    = g.channel_throttle.radio_in;
    g.channel_rudder.radio_max              = g.channel_rudder.radio_in;
    g.rc_5.radio_max = g.rc_5.radio_in;
    g.rc_6.radio_max = g.rc_6.radio_in;
    g.rc_7.radio_max = g.rc_7.radio_in;
    g.rc_8.radio_max = g.rc_8.radio_in;

    g.channel_roll.radio_trim               = g.channel_roll.radio_in;
    g.channel_pitch.radio_trim              = g.channel_pitch.radio_in;
    g.channel_rudder.radio_trim     = g.channel_rudder.radio_in;
    g.rc_5.radio_trim = 1500;
    g.rc_6.radio_trim = 1500;
    g.rc_7.radio_trim = 1500;
    g.rc_8.radio_trim = 1500;

    cliSerial->printf_P(PSTR("\nMove all controls to each extreme. Hit Enter to save: \n"));
    while(1) {

        delay(20);
        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        g.channel_roll.update_min_max();
        g.channel_pitch.update_min_max();
        g.channel_throttle.update_min_max();
        g.channel_rudder.update_min_max();
        g.rc_5.update_min_max();
        g.rc_6.update_min_max();
        g.rc_7.update_min_max();
        g.rc_8.update_min_max();

        if(cliSerial->available() > 0) {
            while (cliSerial->available() > 0) {
                cliSerial->read();
            }
            g.channel_roll.save_eeprom();
            g.channel_pitch.save_eeprom();
            g.channel_throttle.save_eeprom();
            g.channel_rudder.save_eeprom();
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

    while(1) {
        delay(20);
        read_radio();
        switchPosition = readSwitch();


        // look for control switch change
        if (oldSwitchPosition != switchPosition) {
            // force position 5 to MANUAL
            if (switchPosition > 4) {
                flight_modes[switchPosition] = MANUAL;
            }
            // update our current mode
            mode = flight_modes[switchPosition];

            // update the user
            print_switch(switchPosition, mode);

            // Remember switch position
            oldSwitchPosition = switchPosition;
        }

        // look for stick input
        int16_t radioInputSwitch = radio_input_switch();

        if (radioInputSwitch != 0) {

            mode += radioInputSwitch;

            while (
                mode != MANUAL &&
                mode != CIRCLE &&
                mode != STABILIZE &&
                mode != TRAINING &&
                mode != FLY_BY_WIRE_A &&
                mode != FLY_BY_WIRE_B &&
                mode != AUTO &&
                mode != RTL &&
                mode != LOITER)
            {
                if (mode < MANUAL)
                    mode = LOITER;
                else if (mode >LOITER)
                    mode = MANUAL;
                else
                    mode += radioInputSwitch;
            }

            // Override position 5
            if(switchPosition > 4)
                mode = MANUAL;

            // save new mode
            flight_modes[switchPosition] = mode;

            // print new mode
            print_switch(switchPosition, mode);
        }

        // escape hatch
        if(cliSerial->available() > 0) {
            // save changes
            for (mode=0; mode<6; mode++)
                flight_modes[mode].save();
            report_flight_modes();
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
    int c;

    cliSerial->printf_P(PSTR("\nType 'Y' and hit Enter to erase all waypoint and parameter data, any other key to abort: "));

    do {
        c = cliSerial->read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);
    zero_eeprom();
    return 0;
}

static int8_t
setup_level(uint8_t argc, const Menu::arg *argv)
{
    startup_INS_ground(true);
    return 0;
}

#if !defined( __AVR_ATmega1280__ )
/*
  handle full accelerometer calibration via user dialog
 */

static int8_t
setup_accel_scale(uint8_t argc, const Menu::arg *argv)
{
    float trim_roll, trim_pitch;
    cliSerial->println_P(PSTR("Initialising gyros"));

    ahrs.init();
    ahrs.set_fly_forward(true);

    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate,
             flash_leds);
    AP_InertialSensor_UserInteractStream interact(hal.console);
    bool success = ins.calibrate_accel(flash_leds, &interact);
    if (success) {
        // reset ahrs's trim to suggested values from calibration routine
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
        if (g.manual_level == 0) {
            cliSerial->println_P(PSTR("Setting MANUAL_LEVEL to 1"));
            g.manual_level.set_and_save(1);
        }
    }
    report_ins();
    return(0);
}
#endif

static int8_t
setup_compass(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        compass.set_orientation(MAG_ORIENTATION);       // set compass's orientation on aircraft
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
    if(argv[1].i >= 0 && argv[1].i <= 4) {
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
    if(g.battery_monitoring == 0) cliSerial->printf_P(PSTR("Batt monitoring disabled"));
    if(g.battery_monitoring == 3) cliSerial->printf_P(PSTR("Monitoring batt volts"));
    if(g.battery_monitoring == 4) cliSerial->printf_P(PSTR("Monitoring volts and current"));
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

#if APM_CONTROL == DISABLED
	cliSerial->printf_P(PSTR("servo roll:\n"));
	print_PID(&g.pidServoRoll);

    cliSerial->printf_P(PSTR("servo pitch:\n"));
    print_PID(&g.pidServoPitch);

	cliSerial->printf_P(PSTR("servo rudder:\n"));
	print_PID(&g.pidServoRudder);
#endif

    cliSerial->printf_P(PSTR("nav roll:\n"));
    print_PID(&g.pidNavRoll);

    cliSerial->printf_P(PSTR("nav pitch airspeed:\n"));
    print_PID(&g.pidNavPitchAirspeed);

    cliSerial->printf_P(PSTR("energry throttle:\n"));
    print_PID(&g.pidTeThrottle);

    cliSerial->printf_P(PSTR("nav pitch alt:\n"));
    print_PID(&g.pidNavPitchAltitude);

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

    cliSerial->printf_P(PSTR("min: %d\n"
                         "max: %d\n"
                         "cruise: %d\n"
                         "failsafe_enabled: %d\n"
                         "failsafe_value: %d\n"),
                    (int)g.throttle_min,
                    (int)g.throttle_max,
                    (int)g.throttle_cruise,
                    (int)g.throttle_fs_enabled,
                    (int)g.throttle_fs_value);
    print_blanks(2);
}

static void report_ins()
{
    //print_blanks(2);
    cliSerial->printf_P(PSTR("INS\n"));
    print_divider();

    print_gyro_offsets();
    print_accel_offsets_and_scaling();
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

static void report_flight_modes()
{
    //print_blanks(2);
    cliSerial->printf_P(PSTR("Flight modes\n"));
    print_divider();

    for(int16_t i = 0; i < 6; i++ ) {
        print_switch(i, flight_modes[i]);
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
    cliSerial->printf_P(PSTR("CH1: %d | %d | %d\n"), (int)g.channel_roll.radio_min, (int)g.channel_roll.radio_trim, (int)g.channel_roll.radio_max);
    cliSerial->printf_P(PSTR("CH2: %d | %d | %d\n"), (int)g.channel_pitch.radio_min, (int)g.channel_pitch.radio_trim, (int)g.channel_pitch.radio_max);
    cliSerial->printf_P(PSTR("CH3: %d | %d | %d\n"), (int)g.channel_throttle.radio_min, (int)g.channel_throttle.radio_trim, (int)g.channel_throttle.radio_max);
    cliSerial->printf_P(PSTR("CH4: %d | %d | %d\n"), (int)g.channel_rudder.radio_min, (int)g.channel_rudder.radio_trim, (int)g.channel_rudder.radio_max);
    cliSerial->printf_P(PSTR("CH5: %d | %d | %d\n"), (int)g.rc_5.radio_min, (int)g.rc_5.radio_trim, (int)g.rc_5.radio_max);
    cliSerial->printf_P(PSTR("CH6: %d | %d | %d\n"), (int)g.rc_6.radio_min, (int)g.rc_6.radio_trim, (int)g.rc_6.radio_max);
    cliSerial->printf_P(PSTR("CH7: %d | %d | %d\n"), (int)g.rc_7.radio_min, (int)g.rc_7.radio_trim, (int)g.rc_7.radio_max);
    cliSerial->printf_P(PSTR("CH8: %d | %d | %d\n"), (int)g.rc_8.radio_min, (int)g.rc_8.radio_trim, (int)g.rc_8.radio_max);

}

static void
print_switch(uint8_t p, uint8_t m)
{
    cliSerial->printf_P(PSTR("Pos %d: "),p);
    print_flight_mode(m);
}

static void
print_done()
{
    cliSerial->printf_P(PSTR("\nSaved Settings\n\n"));
}

static void
print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        cliSerial->println();
    }
}

static void
print_divider(void)
{
    for (int16_t i = 0; i < 40; i++) {
        cliSerial->printf_P(PSTR("-"));
    }
    cliSerial->println();
}

static int8_t
radio_input_switch(void)
{
    static int8_t bouncer = 0;


    if (int16_t(g.channel_roll.radio_in - g.channel_roll.radio_trim) > 100) {
        bouncer = 10;
    }
    if (int16_t(g.channel_roll.radio_in - g.channel_roll.radio_trim) < -100) {
        bouncer = -10;
    }
    if (bouncer >0) {
        bouncer--;
    }
    if (bouncer <0) {
        bouncer++;
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

static void
print_accel_offsets_and_scaling(void)
{
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f accel_scale = ins.get_accel_scale();
    cliSerial->printf_P(PSTR("Accel offsets: %4.2f, %4.2f, %4.2f\tscale: %4.2f, %4.2f, %4.2f\n"),
                    (float)accel_offsets.x,                           // Pitch
                    (float)accel_offsets.y,                           // Roll
                    (float)accel_offsets.z,                           // YAW
                    (float)accel_scale.x,                             // Pitch
                    (float)accel_scale.y,                             // Roll
                    (float)accel_scale.z);                            // YAW
}

static void
print_gyro_offsets(void)
{
    Vector3f gyro_offsets = ins.get_gyro_offsets();
    cliSerial->printf_P(PSTR("Gyro offsets: %4.2f, %4.2f, %4.2f\n"),
                    (float)gyro_offsets.x,
                    (float)gyro_offsets.y,
                    (float)gyro_offsets.z);
}


#endif // CLI_ENABLED
#line 1 "./Firmware/Arduplane/system.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

#if CLI_ENABLED == ENABLED

// Functions called from the top-level menu
static int8_t   process_logs(uint8_t argc, const Menu::arg *argv);      // in Log.pde
static int8_t   setup_mode(uint8_t argc, const Menu::arg *argv);        // in setup.pde
static int8_t   test_mode(uint8_t argc, const Menu::arg *argv);         // in test.cpp
static int8_t   reboot_board(uint8_t argc, const Menu::arg *argv);

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
static int8_t   main_menu_help(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("Commands:\n"
                         "  logs        log readback/setup mode\n"
                         "  setup       setup mode\n"
                         "  test        test mode\n"
                         "  reboot      reboot to flight mode\n"
                         "\n"));
    return(0);
}

// Command/function table for the top-level menu.
static const struct Menu::command main_menu_commands[] PROGMEM = {
//   command		function called
//   =======        ===============
    {"logs",                process_logs},
    {"setup",               setup_mode},
    {"test",                test_mode},
    {"reboot",              reboot_board},
    {"help",                main_menu_help},
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

static int8_t reboot_board(uint8_t argc, const Menu::arg *argv)
{
    reboot_apm();
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
    // the MAVLink protocol efficiently
    //
    hal.uartA->begin(SERIAL0_BAUD, 128, SERIAL_BUFSIZE);

    // GPS serial port.
    //
    // standard gps running
    hal.uartB->begin(38400, 256, 16);

    cliSerial->printf_P(PSTR("\n\nInit " THISFIRMWARE
                         "\n\nFree RAM: %u\n"),
                    memcheck_available_memory());


    //
    // Check the EEPROM format version before loading any parameters from EEPROM
    //
    load_parameters();

    // reset the uartA baud rate after parameter load
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
    hal.uartC->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD),
            128, SERIAL_BUFSIZE);
    gcs3.init(hal.uartC);
#endif

    mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
    DataFlash.Init();
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash card inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
        do_erase_logs();
        gcs0.reset_cli_timeout();
    }
    if (g.log_bitmask != 0) {
        DataFlash.start_new_log();
    }
#endif

#if HIL_MODE != HIL_MODE_ATTITUDE

 #if CONFIG_ADC == ENABLED
    adc.Init();      // APM ADC library initialization
 #endif

    barometer.init();

    if (g.compass_enabled==true) {
        compass.set_orientation(MAG_ORIENTATION);                                                       // set compass's orientation on aircraft
        if (!compass.init() || !compass.read()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
        }
    }
#endif

    // give AHRS the airspeed sensor
    ahrs.set_airspeed(&airspeed);

#if APM_CONTROL == ENABLED
    // the axis controllers need access to the AHRS system
    g.rollController.set_ahrs(&ahrs);
    g.pitchController.set_ahrs(&ahrs);
    g.yawController.set_ahrs(&ahrs);
#endif

	// Do GPS init
	g_gps = &g_gps_driver;
    // GPS Initialization
    g_gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_4G);

    //mavlink_system.sysid = MAV_SYSTEM_ID;				// Using g.sysid_this_mav
    mavlink_system.compid = 1;          //MAV_COMP_ID_IMU;   // We do not check for comp id
    mavlink_system.type = MAV_TYPE_FIXED_WING;

    // Set initial values for no override
    rc_override_active = hal.rcin->set_overrides(rc_override, 8);

    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up the timer libs

    pinMode(C_LED_PIN, OUTPUT);                         // GPS status LED
    pinMode(A_LED_PIN, OUTPUT);                         // GPS status LED
    pinMode(B_LED_PIN, OUTPUT);                         // GPS status LED
    relay.init();

#if FENCE_TRIGGERED_PIN > 0
    pinMode(FENCE_TRIGGERED_PIN, OUTPUT);
    digitalWrite(FENCE_TRIGGERED_PIN, LOW);
#endif

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
#if USB_MUX_PIN == 0
    hal.uartC->println_P(msg);
#endif

    if (ENABLE_AIR_START == 1) {
        // Perform an air start and get back to flying
        gcs_send_text_P(SEVERITY_LOW,PSTR("<init_ardupilot> AIR START"));

        // Get necessary data from EEPROM
        //----------------
        //read_EEPROM_airstart_critical();
#if HIL_MODE != HIL_MODE_ATTITUDE
        ahrs.init();
        ahrs.set_fly_forward(true);

        ins.init(AP_InertialSensor::WARM_START, 
                 ins_sample_rate,
                 flash_leds);

#endif

        // This delay is important for the APM_RC library to work.
        // We need some time for the comm between the 328 and 1280 to be established.
        int old_pulse = 0;
        while (millis()<=1000 
            && (abs(old_pulse - hal.rcin->read(g.flight_mode_channel)) > 5
               || hal.rcin->read(g.flight_mode_channel) == 1000
               || hal.rcin->read(g.flight_mode_channel) == 1200))
        {
            old_pulse = hal.rcin->read(g.flight_mode_channel);
            delay(25);
        }
        g_gps->update();

        if (g.log_bitmask & MASK_LOG_CMD)
            Log_Write_Startup(TYPE_AIRSTART_MSG);
        reload_commands_airstart();                     // Get set to resume AUTO from where we left off

    }else {
        startup_ground();
        if (g.log_bitmask & MASK_LOG_CMD)
            Log_Write_Startup(TYPE_GROUNDSTART_MSG);
    }

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

#if (GROUND_START_DELAY > 0)
    gcs_send_text_P(SEVERITY_LOW,PSTR("<startup_ground> With Delay"));
    delay(GROUND_START_DELAY * 1000);
#endif

    // Makes the servos wiggle
    // step 1 = 1 wiggle
    // -----------------------
    demo_servos(1);

    //INS ground start
    //------------------------
    //
    startup_INS_ground(false);

    // read the radio to set trims
    // ---------------------------
    trim_radio();               // This was commented out as a HACK.  Why?  I don't find a problem.

    // Save the settings for in-air restart
    // ------------------------------------
    //save_EEPROM_groundstart();

    // initialize commands
    // -------------------
    init_commands();

    // Makes the servos wiggle - 3 times signals ready to fly
    // -----------------------
    demo_servos(3);

    // reset last heartbeat time, so we don't trigger failsafe on slow
    // startup
    last_heartbeat_ms = millis();

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    hal.uartC->set_blocking_writes(false);
    if (gcs3.initialised) {
        hal.uartC->set_blocking_writes(false);
    }

    gcs_send_text_P(SEVERITY_LOW,PSTR("\n\n Ready to FLY."));
}

static void set_mode(enum FlightMode mode)
{
    if(control_mode == mode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }
    if(g.auto_trim > 0 && control_mode == MANUAL)
        trim_control_surfaces();

    control_mode = mode;
    crash_timer = 0;

    switch(control_mode)
    {
    case INITIALISING:
    case MANUAL:
    case STABILIZE:
    case TRAINING:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
        break;

    case CIRCLE:
        // the altitude to circle at is taken from the current altitude
        next_WP.alt = current_loc.alt;
        break;

    case AUTO:
        update_auto();
        break;

    case RTL:
        do_RTL();
        break;

    case LOITER:
        do_loiter_at_location();
        break;

    case GUIDED:
        set_guided_WP();
        break;

    default:
        do_RTL();
        break;
    }

    // if in an auto-throttle mode, start with throttle suppressed for
    // safety. suppress_throttle() will unsupress it when appropriate
    if (control_mode == CIRCLE || control_mode >= FLY_BY_WIRE_B) {
        throttle_suppressed = true;
    }

    if (g.log_bitmask & MASK_LOG_MODE)
        Log_Write_Mode(control_mode);
}

static void check_long_failsafe()
{
    uint32_t tnow = millis();
    // only act on changes
    // -------------------
    if(failsafe != FAILSAFE_LONG  && failsafe != FAILSAFE_GCS) {
        if (rc_override_active && tnow - last_heartbeat_ms > FAILSAFE_LONG_TIME) {
            failsafe_long_on_event(FAILSAFE_LONG);
        }
        if(!rc_override_active && failsafe == FAILSAFE_SHORT && 
           (tnow - ch3_failsafe_timer) > FAILSAFE_LONG_TIME) {
            failsafe_long_on_event(FAILSAFE_LONG);
        }
        if (g.gcs_heartbeat_fs_enabled && 
            (tnow - last_heartbeat_ms) > FAILSAFE_LONG_TIME) {
            failsafe_long_on_event(FAILSAFE_GCS);
        }
    } else {
        // We do not change state but allow for user to change mode
        if (failsafe == FAILSAFE_GCS && 
            (tnow - last_heartbeat_ms) < FAILSAFE_SHORT_TIME) 
            failsafe = FAILSAFE_NONE;
        if (failsafe == FAILSAFE_LONG && rc_override_active && 
            (tnow - last_heartbeat_ms) < FAILSAFE_SHORT_TIME) 
            failsafe = FAILSAFE_NONE;
        if (failsafe == FAILSAFE_LONG && !rc_override_active && !ch3_failsafe) 
            failsafe = FAILSAFE_NONE;
    }
}

static void check_short_failsafe()
{
    // only act on changes
    // -------------------
    if(failsafe == FAILSAFE_NONE) {
        if(ch3_failsafe) {                                              // The condition is checked and the flag ch3_failsafe is set in radio.pde
            failsafe_short_on_event(FAILSAFE_SHORT);
        }
    }

    if(failsafe == FAILSAFE_SHORT) {
        if(!ch3_failsafe) {
            failsafe_short_off_event();
        }
    }
}


static void startup_INS_ground(bool force_accel_level)
{
#if HIL_MODE != HIL_MODE_DISABLED
    while (!barometer.healthy) {
        // the barometer becomes healthy when we get the first
        // HIL_STATE message
        gcs_send_text_P(SEVERITY_LOW, PSTR("Waiting for first HIL_STATE message"));
        delay(1000);
    }
#endif

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
#if HIL_MODE == HIL_MODE_DISABLED
    if (force_accel_level || g.manual_level == 0) {
        // when MANUAL_LEVEL is set to 1 we don't do accelerometer
        // levelling on each boot, and instead rely on the user to do
        // it once via the ground station
        ins.init_accel(flash_leds);
        ahrs.set_trim(Vector3f(0, 0, 0));
    }
#endif
    ahrs.reset();

#if HIL_MODE != HIL_MODE_ATTITUDE
    // read Baro pressure at ground
    //-----------------------------
    init_barometer();

    if (airspeed.enabled()) {
        // initialize airspeed sensor
        // --------------------------
        zero_airspeed();
    } else {
        gcs_send_text_P(SEVERITY_LOW,PSTR("NO airspeed"));
    }

#endif
    digitalWrite(B_LED_PIN, LED_ON);                    // Set LED B high to indicate INS ready
    digitalWrite(A_LED_PIN, LED_OFF);
    digitalWrite(C_LED_PIN, LED_OFF);
}


static void update_GPS_light(void)
{
    // GPS LED on if we have a fix or Blink GPS LED if we are receiving data
    // ---------------------------------------------------------------------
    switch (g_gps->status()) {
    case (2):
        digitalWrite(C_LED_PIN, LED_ON);                  //Turn LED C on when gps has valid fix.
        break;

    case (1):
        if (g_gps->valid_read == true) {
            GPS_light = !GPS_light;                     // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
            if (GPS_light) {
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
    mainLoop_count                  = 0;
    G_Dt_max                                = 0;
    ahrs.renorm_range_count         = 0;
    ahrs.renorm_blowup_count = 0;
    gps_fix_count                   = 0;
    pmTest1                                 = 0;
    perf_mon_timer                  = millis();
}


/*
 *  map from a 8 bit EEPROM baud rate to a real baud rate
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


static void check_usb_mux(void)
{
#if USB_MUX_PIN > 0
    bool usb_check = !digitalRead(USB_MUX_PIN);
    if (usb_check == usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    usb_connected = usb_check;
    if (usb_connected) {
        hal.uartA->begin(SERIAL0_BAUD);
    } else {
        hal.uartA->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD));
    }
#endif
}


/*
 *  called by gyro/accel init to flash LEDs so user
 *  has some mesmerising lights to watch while waiting
 */
void flash_leds(bool on)
{
    digitalWrite(A_LED_PIN, on ? LED_OFF : LED_ON);
    digitalWrite(C_LED_PIN, on ? LED_ON : LED_OFF);
}

/*
 * Read Vcc vs 1.1v internal reference
 */
uint16_t board_voltage(void)
{
    return vcc_pin->read_latest();
}


/*
  force a software reset of the APM
 */
static void reboot_apm(void)
{
    hal.scheduler->reboot();
    while (1);
}


static void
print_flight_mode(uint8_t mode)
{
    switch (mode) {
    case MANUAL:
        cliSerial->println_P(PSTR("Manual"));
        break;
    case CIRCLE:
        cliSerial->println_P(PSTR("Circle"));
        break;
    case STABILIZE:
        cliSerial->println_P(PSTR("Stabilize"));
        break;
    case TRAINING:
        cliSerial->println_P(PSTR("Training"));
        break;
    case FLY_BY_WIRE_A:
        cliSerial->println_P(PSTR("FBW_A"));
        break;
    case FLY_BY_WIRE_B:
        cliSerial->println_P(PSTR("FBW_B"));
        break;
    case AUTO:
        cliSerial->println_P(PSTR("AUTO"));
        break;
    case RTL:
        cliSerial->println_P(PSTR("RTL"));
        break;
    case LOITER:
        cliSerial->println_P(PSTR("Loiter"));
        break;
    default:
        cliSerial->println_P(PSTR("---"));
        break;
    }
}

static void print_comma(void)
{
    cliSerial->print_P(PSTR(","));
}


#line 1 "./Firmware/Arduplane/test.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t   test_radio_pwm(uint8_t argc,    const Menu::arg *argv);
static int8_t   test_radio(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_passthru(uint8_t argc,     const Menu::arg *argv);
static int8_t   test_failsafe(uint8_t argc,     const Menu::arg *argv);
static int8_t   test_gps(uint8_t argc,                  const Menu::arg *argv);
#if CONFIG_ADC == ENABLED
static int8_t   test_adc(uint8_t argc,                  const Menu::arg *argv);
#endif
static int8_t   test_ins(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_battery(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_relay(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_wp(uint8_t argc,                   const Menu::arg *argv);
static int8_t   test_airspeed(uint8_t argc,     const Menu::arg *argv);
static int8_t   test_pressure(uint8_t argc,     const Menu::arg *argv);
static int8_t   test_mag(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_xbee(uint8_t argc,                 const Menu::arg *argv);
static int8_t   test_eedump(uint8_t argc,               const Menu::arg *argv);
static int8_t   test_rawgps(uint8_t argc,                       const Menu::arg *argv);
static int8_t   test_modeswitch(uint8_t argc,           const Menu::arg *argv);
static int8_t   test_logging(uint8_t argc,              const Menu::arg *argv);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
static int8_t   test_shell(uint8_t argc,              const Menu::arg *argv);
#endif

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
static const struct Menu::command test_menu_commands[] PROGMEM = {
    {"pwm",                         test_radio_pwm},
    {"radio",                       test_radio},
    {"passthru",            test_passthru},
    {"failsafe",            test_failsafe},
    {"battery",     test_battery},
    {"relay",                       test_relay},
    {"waypoints",           test_wp},
    {"xbee",                        test_xbee},
    {"eedump",                      test_eedump},
    {"modeswitch",          test_modeswitch},

    // Tests below here are for hardware sensors only present
    // when real sensors are attached or they are emulated
#if HIL_MODE == HIL_MODE_DISABLED
 #if CONFIG_ADC == ENABLED
    {"adc",                 test_adc},
 #endif
    {"gps",                 test_gps},
    {"rawgps",              test_rawgps},
    {"ins",                 test_ins},
    {"airspeed",    test_airspeed},
    {"airpressure", test_pressure},
    {"compass",             test_mag},
#elif HIL_MODE == HIL_MODE_SENSORS
    {"gps",                 test_gps},
    {"ins",                 test_ins},
    {"compass",             test_mag},
#elif HIL_MODE == HIL_MODE_ATTITUDE
#endif
    {"logging",             test_logging},
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
test_eedump(uint8_t argc, const Menu::arg *argv)
{
    uint16_t i, j;

    // hexdump the EEPROM
    for (i = 0; i < EEPROM_MAX_ADDR; i += 16) {
        cliSerial->printf_P(PSTR("%04x:"), i);
        for (j = 0; j < 16; j++)
            cliSerial->printf_P(PSTR(" %02x"), hal.storage->read_byte(i + j));
        cliSerial->println();
    }
    return(0);
}

static int8_t
test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);

        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        cliSerial->printf_P(PSTR("IN:\t1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
                        (int)g.channel_roll.radio_in,
                        (int)g.channel_pitch.radio_in,
                        (int)g.channel_throttle.radio_in,
                        (int)g.channel_rudder.radio_in,
                        (int)g.rc_5.radio_in,
                        (int)g.rc_6.radio_in,
                        (int)g.rc_7.radio_in,
                        (int)g.rc_8.radio_in);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}


static int8_t
test_passthru(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);

        // New radio frame? (we could use also if((millis()- timer) > 20)
        if (hal.rcin->valid() > 0) {
            cliSerial->print_P(PSTR("CH:"));
            for(int16_t i = 0; i < 8; i++) {
                cliSerial->print(hal.rcin->read(i));        // Print channel values
                print_comma();
                hal.rcout->write(i, hal.rcin->read(i)); // Copy input to Servos
            }
            cliSerial->println();
        }
        if (cliSerial->available() > 0) {
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

    while(1) {
        delay(20);
        read_radio();

        g.channel_roll.calc_pwm();
        g.channel_pitch.calc_pwm();
        g.channel_throttle.calc_pwm();
        g.channel_rudder.calc_pwm();

        // write out the servo PWM values
        // ------------------------------
        set_servos();

        cliSerial->printf_P(PSTR("IN 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
                        (int)g.channel_roll.control_in,
                        (int)g.channel_pitch.control_in,
                        (int)g.channel_throttle.control_in,
                        (int)g.channel_rudder.control_in,
                        (int)g.rc_5.control_in,
                        (int)g.rc_6.control_in,
                        (int)g.rc_7.control_in,
                        (int)g.rc_8.control_in);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_failsafe(uint8_t argc, const Menu::arg *argv)
{
    uint8_t fail_test;
    print_hit_enter();
    for(int16_t i = 0; i < 50; i++) {
        delay(20);
        read_radio();
    }

    // read the radio to set trims
    // ---------------------------
    trim_radio();

    oldSwitchPosition = readSwitch();

    cliSerial->printf_P(PSTR("Unplug battery, throttle in neutral, turn off radio.\n"));
    while(g.channel_throttle.control_in > 0) {
        delay(20);
        read_radio();
    }

    while(1) {
        delay(20);
        read_radio();

        if(g.channel_throttle.control_in > 0) {
            cliSerial->printf_P(PSTR("THROTTLE CHANGED %d \n"), (int)g.channel_throttle.control_in);
            fail_test++;
        }

        if(oldSwitchPosition != readSwitch()) {
            cliSerial->printf_P(PSTR("CONTROL MODE CHANGED: "));
            print_flight_mode(readSwitch());
            fail_test++;
        }

        if(g.throttle_fs_enabled && g.channel_throttle.get_failsafe()) {
            cliSerial->printf_P(PSTR("THROTTLE FAILSAFE ACTIVATED: %d, "), (int)g.channel_throttle.radio_in);
            print_flight_mode(readSwitch());
            fail_test++;
        }

        if(fail_test > 0) {
            return (0);
        }
        if(cliSerial->available() > 0) {
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

        while(1) {
            delay(100);
            read_radio();
            read_battery();
            if (g.battery_monitoring == 3) {
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

            if(cliSerial->available() > 0) {
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

    while(1) {
        cliSerial->printf_P(PSTR("Relay on\n"));
        relay.on();
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }

        cliSerial->printf_P(PSTR("Relay off\n"));
        relay.off();
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_wp(uint8_t argc, const Menu::arg *argv)
{
    delay(1000);

    // save the alitude above home option
    if (g.RTL_altitude_cm < 0) {
        cliSerial->printf_P(PSTR("Hold current altitude\n"));
    }else{
        cliSerial->printf_P(PSTR("Hold altitude of %dm\n"), (int)g.RTL_altitude_cm/100);
    }

    cliSerial->printf_P(PSTR("%d waypoints\n"), (int)g.command_total);
    cliSerial->printf_P(PSTR("Hit radius: %d\n"), (int)g.waypoint_radius);
    cliSerial->printf_P(PSTR("Loiter radius: %d\n\n"), (int)g.loiter_radius);

    for(uint8_t i = 0; i <= g.command_total; i++) {
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
                    (long)cmd->alt,
                    (long)cmd->lat,
                    (long)cmd->lng);
}

static int8_t
test_xbee(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);
    cliSerial->printf_P(PSTR("Begin XBee X-CTU Range and RSSI Test:\n"));

    while(1) {

        if (hal.uartC->available())
            hal.uartC->write(hal.uartC->read());

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}


static int8_t
test_modeswitch(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    cliSerial->printf_P(PSTR("Control CH "));

    cliSerial->println(FLIGHT_MODE_CHANNEL, DEC);

    while(1) {
        delay(20);
        uint8_t switchPosition = readSwitch();
        if (oldSwitchPosition != switchPosition) {
            cliSerial->printf_P(PSTR("Position %d\n"),  (int)switchPosition);
            oldSwitchPosition = switchPosition;
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

/*
 *  test the dataflash is working
 */
static int8_t
test_logging(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->println_P(PSTR("Testing dataflash logging"));
    if (!DataFlash.CardInserted()) {
        cliSerial->println_P(PSTR("ERR: No dataflash inserted"));
        return 0;
    }
    DataFlash.ReadManufacturerID();
    cliSerial->printf_P(PSTR("Manufacturer: 0x%02x   Device: 0x%04x\n"),
                    (unsigned)DataFlash.df_manufacturer,
                    (unsigned)DataFlash.df_device);
    cliSerial->printf_P(PSTR("NumPages: %u\n"),
                    (unsigned)DataFlash.df_NumPages+1);
    DataFlash.StartRead(DataFlash.df_NumPages+1);
    cliSerial->printf_P(PSTR("Format version: %lx  Expected format version: %lx\n"),
                    (unsigned long)DataFlash.ReadLong(),
                    (unsigned long)DF_LOGGING_FORMAT);
    return 0;
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

    while(1) {
        for (int8_t i=0; i<9; i++) cliSerial->printf_P(PSTR("%.1f\t"),adc.Ch(i));
        cliSerial->println();
        delay(100);
        if(cliSerial->available() > 0) {
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

    while(1) {
        delay(333);

        // Blink GPS LED if we don't have a fix
        // ------------------------------------
        update_GPS_light();

        g_gps->update();

        if (g_gps->new_data) {
            cliSerial->printf_P(PSTR("Lat: %ld, Lon %ld, Alt: %ldm, #sats: %d\n"),
                            (long)g_gps->latitude,
                            (long)g_gps->longitude,
                            (long)g_gps->altitude/100,
                            (int)g_gps->num_sats);
        }else{
            cliSerial->printf_P(PSTR("."));
        }
        if(cliSerial->available() > 0) {
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

    while(1) {
        delay(20);
        if (millis() - fast_loopTimer_ms > 19) {
            delta_ms_fast_loop      = millis() - fast_loopTimer_ms;
            G_Dt                            = (float)delta_ms_fast_loop / 1000.f;                       // used by DCM integrator
            fast_loopTimer_ms       = millis();

            // INS
            // ---
            ahrs.update();

            if(g.compass_enabled) {
                medium_loopCounter++;
                if(medium_loopCounter == 5) {
                    compass.read();
                    medium_loopCounter = 0;
                }
            }

            // We are using the INS
            // ---------------------
            Vector3f gyros  = ins.get_gyro();
            Vector3f accels = ins.get_accel();
            cliSerial->printf_P(PSTR("r:%4d  p:%4d  y:%3d  g=(%5.1f %5.1f %5.1f)  a=(%5.1f %5.1f %5.1f)\n"),
                            (int)ahrs.roll_sensor / 100,
                            (int)ahrs.pitch_sensor / 100,
                            (uint16_t)ahrs.yaw_sensor / 100,
                            gyros.x, gyros.y, gyros.z,
                            accels.x, accels.y, accels.z);
        }
        if(cliSerial->available() > 0) {
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

    int16_t counter = 0;
    float heading = 0;

    //cliSerial->printf_P(PSTR("MAG_ORIENTATION: %d\n"), MAG_ORIENTATION);

    print_hit_enter();

    while(1) {
        delay(20);
        if (millis() - fast_loopTimer_ms > 19) {
            delta_ms_fast_loop      = millis() - fast_loopTimer_ms;
            G_Dt                            = (float)delta_ms_fast_loop / 1000.f;                       // used by DCM integrator
            fast_loopTimer_ms       = millis();

            // INS
            // ---
            ahrs.update();

            medium_loopCounter++;
            if(medium_loopCounter == 5) {
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
                                    (wrap_360_cd(ToDeg(heading) * 100)) /100,
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

#if HIL_MODE == HIL_MODE_DISABLED

static int8_t
test_airspeed(uint8_t argc, const Menu::arg *argv)
{
    float airspeed_ch = pitot_analog_source->read_average();
    // cliSerial->println(pitot_analog_source.read());
    cliSerial->printf_P(PSTR("airspeed_ch: %.1f\n"), airspeed_ch);

    if (!airspeed.enabled()) {
        cliSerial->printf_P(PSTR("airspeed: "));
        print_enabled(false);
        return (0);

    }else{
        print_hit_enter();
        zero_airspeed();
        cliSerial->printf_P(PSTR("airspeed: "));
        print_enabled(true);

        while(1) {
            delay(20);
            read_airspeed();
            cliSerial->printf_P(PSTR("%.1f m/s\n"), airspeed.get_airspeed());

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    }
}


static int8_t
test_pressure(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("Uncalibrated relative airpressure\n"));
    print_hit_enter();

    home.alt        = 0;
    wp_distance = 0;
    init_barometer();

    while(1) {
        delay(100);
        current_loc.alt = read_barometer() + home.alt;

        if (!barometer.healthy) {
            cliSerial->println_P(PSTR("not healthy"));
        } else {
            cliSerial->printf_P(PSTR("Alt: %0.2fm, Raw: %f Temperature: %.1f\n"),
                            current_loc.alt / 100.0,
                            barometer.get_pressure(), 0.1*barometer.get_temperature());
        }

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_rawgps(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        // Blink Yellow LED if we are sending data to GPS
        if (hal.uartC->available()) {
            digitalWrite(B_LED_PIN, LED_ON);
            hal.uartB->write(hal.uartC->read());
            digitalWrite(B_LED_PIN, LED_OFF);
        }
        // Blink Red LED if we are receiving data from GPS
        if (hal.uartB->available()) {
            digitalWrite(C_LED_PIN, LED_ON);
            hal.uartC->write(hal.uartB->read());
            digitalWrite(C_LED_PIN, LED_OFF);
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}
#endif // HIL_MODE == HIL_MODE_DISABLED

#endif // CLI_ENABLED
