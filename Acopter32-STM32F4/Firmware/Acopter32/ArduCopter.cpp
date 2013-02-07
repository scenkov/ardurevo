#line 1 "./Firmware/ACopter32/ArduCopter.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduCopter V2.9-dev"
/*
 *  ArduCopter Version 2.9
 *  Lead author:	Jason Short
 *  Based on code and ideas from the Arducopter team: Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen, Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni
 *  Thanks to:	Chris Anderson, Mike Smith, Jordi Munoz, Doug Weibel, James Goppert, Benjamin Pelletier, Robert Lefebvre, Marco Robustini
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  Special Thanks for Contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera		:Auto Compass Declination
 *  Amilcar Lucas		:Camera mount library
 *  Andrew Tridgell		:General development, Mavlink Support
 *  Angel Fernandez		:Alpha testing
 *  Doug Weibel			:Libraries
 *  Christof Schmid		:Alpha testing
 *  Dani Saez           :V Octo Support
 *  Gregory Fletcher	:Camera mount orientation math
 *  Guntars				:Arming safety suggestion
 *  HappyKillmore		:Mavlink GCS
 *  Hein Hollander      :Octo Support
 *  Igor van Airde      :Control Law optimization
 *  Leonard Hall 		:Flight Dynamics, INAV throttle
 *  Jonathan Challinger :Inertial Navigation
 *  Jean-Louis Naudin   :Auto Landing
 *  Max Levine			:Tri Support, Graphics
 *  Jack Dunkle			:Alpha testing
 *  James Goppert		:Mavlink Support
 *  Jani Hiriven		:Testing feedback
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio			:Stabilization Control laws
 *  Randy Mackay		:General development and release
 *  Marco Robustini		:Lead tester
 *  Michael Oborne		:Mission Planner GCS
 *  Mike Smith			:Libraries, Coding support
 *  Oliver				:Piezo support
 *  Olivier Adler       :PPM Encoder
 *  Robert Lefebvre		:Heli Support & LEDs
 *  Sandro Benigno      :Camera support
 *
 *  And much more so PLEASE PM me on DIYDRONES to add your contribution to the List
 *
 *  Requires modified "mrelax" version of Arduino, which can be found here:
 *  http://code.google.com/p/ardupilot-mega/downloads/list
 *
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// Common dependencies
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
// AP_HAL
#include <AP_HAL.h>
//#include <AP_HAL_AVR.h>
//#include <AP_HAL_AVR_SITL.h>
//#include <AP_HAL_SMACCM.h>
//#include <AP_HAL_PX4.h>
//#include <AP_HAL_SMACCM.h>
#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_Empty.h>

// Application dependencies
#include <GCS_MAVLink.h>        // MAVLink GCS definitions
#include <AP_GPS.h>             // ArduPilot GPS library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <APM_PI.h>             // PI library
#include <AC_PID.h>             // PID library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>          // AP Motors library
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow.h>     // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <AP_LeadFilter.h>      // GPS Lead filter
#include <AP_Relay.h>           // APM relay
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AP_Limits.h>
#include <memcheck.h>           // memory limit checker
#include <SITL.h>               // software in the loop support
#include <AP_Scheduler.h>       // main loop scheduler

// AP_HAL to Arduino compatibility layer
#include "compat.h"
// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"

// Local modules
#include "Parameters.h"
#include "GCS.h"

////////////////////////////////////////////////////////////////////////////////
// cliSerial
////////////////////////////////////////////////////////////////////////////////
// cliSerial isn't strictly necessary - it is an alias for hal.console. It may
// be deprecated in favor of hal.console in later releases.
 static void compass_accumulate(void) ;
 static void barometer_accumulate(void) ;
  static void perf_update(void) ;
  void loop() ;
 static void fast_loop() ;
  static void medium_loop() ;
 static void fifty_hz_loop() ;
   static void slow_loop() ;
 static void super_slow_loop() ;
 static void update_optical_flow(void) ;
 static void update_GPS(void) ;
 bool set_yaw_mode(uint8_t new_yaw_mode) ;
 void update_yaw_mode(void) ;
 bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode) ;
 void update_roll_pitch_mode(void) ;
 void update_simple_mode(void) ;
 void update_super_simple_beading() ;
 bool set_throttle_mode( uint8_t new_throttle_mode ) ;
 void update_throttle_mode(void) ;
  static void read_AHRS(void) ;
  static void update_trig(void);
 static void update_altitude() ;
  static void update_altitude_est() ;
  static void tuning();
   void set_home_is_set(bool b) ;
 void set_armed(bool b) ;
 void set_auto_armed(bool b) ;
 void set_simple_mode(bool b) ;
 static void set_failsafe(bool mode) ;
 void set_low_battery(bool b) ;
 void set_takeoff_complete(bool b) ;
 void set_land_complete(bool b) ;
  void set_alt_change(uint8_t flag);
  void set_compass_healthy(bool b) ;
  void set_gps_healthy(bool b) ;
  void dump_state() ;
  static void get_stabilize_roll(int32_t target_angle) ;
  static void get_stabilize_pitch(int32_t target_angle) ;
  static void get_stabilize_yaw(int32_t target_angle) ;
  static void get_acro_roll(int32_t target_rate) ;
  static void get_acro_pitch(int32_t target_rate) ;
  static void get_acro_yaw(int32_t target_rate) ;
 static void get_roll_rate_stabilized_ef(int32_t stick_angle) ;
 static void get_pitch_rate_stabilized_ef(int32_t stick_angle) ;
 static void get_yaw_rate_stabilized_ef(int32_t stick_angle) ;
 void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) ;
 void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) ;
 void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) ;
 void update_rate_contoller_targets() ;
 void run_rate_controllers() ;
 void init_rate_controllers() ;
  static int16_t get_heli_rate_roll(int32_t target_rate) ;
  static int16_t get_heli_rate_pitch(int32_t target_rate) ;
  static int16_t get_heli_rate_yaw(int32_t target_rate) ;
 static int16_t get_rate_roll(int32_t target_rate) ;
  static int16_t get_rate_pitch(int32_t target_rate) ;
  static int16_t get_rate_yaw(int32_t target_rate) ;
 static int32_t get_of_roll(int32_t input_roll) ;
  static int32_t get_of_pitch(int32_t input_pitch) ;
  static void get_look_ahead_yaw(int16_t pilot_yaw) ;
 static void update_throttle_cruise(int16_t throttle) ;
 static int16_t get_angle_boost(int16_t throttle) ;
 static int16_t get_angle_boost(int16_t throttle) ;
 void set_throttle_out( int16_t throttle_out, bool apply_angle_boost ) ;
 void set_throttle_accel_target( int16_t desired_acceleration ) ;
 void throttle_accel_deactivate() ;
 static int16_t get_throttle_accel(int16_t z_target_accel) ;
 static int16_t get_pilot_desired_throttle(int16_t throttle_control) ;
 static int16_t get_pilot_desired_climb_rate(int16_t throttle_control) ;
 static int16_t get_pilot_desired_acceleration(int16_t throttle_control) ;
 static int32_t get_pilot_desired_direct_alt(int16_t throttle_control) ;
 static void get_throttle_rate(int16_t z_target_speed) ;
 static void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate) ;
 static void get_throttle_althold_with_slew(int16_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate) ;
 static void get_throttle_rate_stabilized(int16_t target_rate) ;
 static void get_throttle_land() ;
 static void get_throttle_surface_tracking(int16_t target_rate) ;
 static void reset_I_all(void) ;
  static void reset_rate_I() ;
  static void reset_optflow_I(void) ;
  static void reset_wind_I(void) ;
  static void reset_throttle_I(void) ;
  static void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle) ;
  static void reset_stability_I(void) ;
 static void gcs_check_delay() ;
  static void gcs_send_heartbeat(void) ;
  static void gcs_send_deferred(void) ;
  static NOINLINE void send_heartbeat(mavlink_channel_t chan) ;
  static NOINLINE void send_attitude(mavlink_channel_t chan) ;
 static NOINLINE void send_limits_status(mavlink_channel_t chan) ;
   static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops) ;
  static void NOINLINE send_meminfo(mavlink_channel_t chan) ;
  static void NOINLINE send_location(mavlink_channel_t chan) ;
  static void NOINLINE send_nav_controller_output(mavlink_channel_t chan) ;
  static void NOINLINE send_ahrs(mavlink_channel_t chan) ;
 static void NOINLINE send_simstate(mavlink_channel_t chan) ;
  static void NOINLINE send_hwstatus(mavlink_channel_t chan) ;
  static void NOINLINE send_gps_raw(mavlink_channel_t chan) ;
  static void NOINLINE send_servo_out(mavlink_channel_t chan) ;
  static void NOINLINE send_radio_in(mavlink_channel_t chan) ;
  static void NOINLINE send_radio_out(mavlink_channel_t chan) ;
  static void NOINLINE send_vfr_hud(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu1(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu2(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu3(mavlink_channel_t chan) ;
  static void NOINLINE send_current_waypoint(mavlink_channel_t chan) ;
  static void NOINLINE send_statustext(mavlink_channel_t chan) ;
 static bool telemetry_delayed(mavlink_channel_t chan) ;
 static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops) ;
 static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops) ;
  void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str) ;
 static void mavlink_delay_cb() ;
 static void gcs_send_message(enum ap_message id) ;
 static void gcs_data_stream_send(void) ;
 static void gcs_check_input(void) ;
  static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str) ;
  static bool print_log_menu(void) ;
  static void do_erase_logs(void) ;
 static void Log_Write_GPS() ;
 static void Log_Read_GPS() ;
 static void Log_Write_IMU() ;
 static void Log_Read_IMU() ;
 static void Log_Write_Current() ;
 static void Log_Read_Current() ;
 static void Log_Write_Motors() ;
 static void Log_Read_Motors() ;
 static void Log_Write_Optflow() ;
 static void Log_Read_Optflow() ;
 static void Log_Write_Nav_Tuning() ;
 static void Log_Read_Nav_Tuning() ;
 static void Log_Write_Control_Tuning() ;
 static void Log_Read_Control_Tuning() ;
  static void Log_Write_Iterm() ;
 static void Log_Read_Iterm() ;
 static void Log_Write_Performance() ;
 static void Log_Read_Performance() ;
 static void Log_Write_Cmd(uint8_t num, struct Location *wp) ;
 static void Log_Read_Cmd() ;
 static void Log_Write_Attitude() ;
 static void Log_Read_Attitude() ;
 static void Log_Write_INAV() ;
 static void Log_Read_INAV() ;
 static void Log_Write_Mode(uint8_t mode) ;
 static void Log_Read_Mode() ;
 static void Log_Write_Startup() ;
 static void Log_Read_Startup() ;
 static void Log_Write_Event(uint8_t id) ;
 static void Log_Read_Event() ;
 static void Log_Write_Data(uint8_t id, int8_t value) ;
 static void Log_Read_Int8t() ;
 static void Log_Write_Data(uint8_t id, int16_t value) ;
 static void Log_Read_Int16t() ;
 static void Log_Write_Data(uint8_t id, uint16_t value) ;
 static void Log_Read_UInt16t() ;
 static void Log_Write_Data(uint8_t id, int32_t value) ;
 static void Log_Read_Int32t() ;
 static void Log_Write_Data(uint8_t id, uint32_t value) ;
 static void Log_Read_UInt32t() ;
 static void Log_Write_Data(uint8_t id, float value) ;
 static void Log_Read_Float() ;
 static void Log_Write_PID(uint8_t pid_id, int32_t error, int32_t p, int32_t i, int32_t d, int32_t output, float gain) ;
 static void Log_Read_PID() ;
 static void Log_Write_DMP() ;
 static void Log_Read_DMP() ;
 static void Log_Write_Camera() ;
 static void Log_Read_Camera() ;
 static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) ;
 static void Log_Read_Error() ;
 static void Log_Read(int16_t start_page, int16_t end_page) ;
 static void log_callback(uint8_t msgid) ;
 static void Log_Write_Startup() ;
 static void Log_Write_Cmd(uint8_t num, struct Location *wp) ;
 static void Log_Write_Mode(uint8_t mode) ;
 static void Log_Write_IMU() ;
 static void Log_Write_GPS() ;
 static void Log_Write_Current() ;
 static void Log_Write_Iterm() ;
 static void Log_Write_Attitude() ;
 static void Log_Write_INAV() ;
 static void Log_Write_Data(uint8_t id, int8_t value);
 static void Log_Write_Data(uint8_t id, int16_t value);
 static void Log_Write_Data(uint8_t id, uint16_t value);
 static void Log_Write_Data(uint8_t id, int32_t value);
 static void Log_Write_Data(uint8_t id, uint32_t value);
 static void Log_Write_Data(uint8_t id, float value);
 static void Log_Write_Event(uint8_t id);
 static void Log_Write_Optflow() ;
 static void Log_Write_Nav_Tuning() ;
 static void Log_Write_Control_Tuning() ;
 static void Log_Write_Motors() ;
 static void Log_Write_Performance() ;
 static void Log_Write_PID(uint8_t pid_id, int32_t error, int32_t p, int32_t i, int32_t d, int32_t output, float gain) ;
 static void Log_Write_DMP() ;
 static void Log_Write_Camera() ;
 static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) ;
   static void load_parameters(void) ;
void userhook_init() ;
  void userhook_50Hz() ;
  static void init_commands() ;
 static struct Location get_cmd_with_index(int i) ;
 static void set_cmd_with_index(struct Location temp, int i) ;
  static int32_t get_RTL_alt() ;
  static void set_next_WP(struct Location *wp) ;
 static void init_home() ;
 static void process_nav_command() ;
  static void process_cond_command() ;
  static void process_now_command() ;
 static bool verify_must() ;
 static bool verify_may() ;
 static void do_RTL(void) ;
 static void do_takeoff() ;
 static void do_nav_wp() ;
 static void do_land() ;
  static void do_loiter_unlimited() ;
 static void do_loiter_turns() ;
 static void do_loiter_time() ;
 static bool verify_takeoff() ;
 static bool verify_land() ;
 static bool verify_nav_wp() ;
  static bool verify_loiter_unlimited() ;
 static bool verify_loiter_time() ;
 static bool verify_loiter_turns() ;
 static bool verify_RTL() ;
  static void do_wait_delay() ;
  static void do_change_alt() ;
  static void do_within_distance() ;
  static void do_yaw() ;
  static bool verify_wait_delay() ;
  static bool verify_change_alt() ;
  static bool verify_within_distance() ;
 static bool verify_yaw() ;
 static bool verify_nav_roi() ;
  static void do_change_speed() ;
  static void do_jump() ;
  static void do_set_home() ;
  static void do_set_servo() ;
  static void do_set_relay() ;
  static void do_repeat_servo() ;
  static void do_repeat_relay() ;
 static void do_nav_roi() ;
 static void do_take_picture() ;
 static void change_command(uint8_t cmd_index) ;
 static void update_commands() ;
 static void execute_nav_command(void) ;
 static void verify_commands(void) ;
 static int16_t find_next_nav_index(int16_t search_index) ;
  static void exit_mission() ;
  void delay(uint32_t ms) ;
  void mavlink_delay(uint32_t ms) ;
  uint32_t millis() ;
  uint32_t micros() ;
  void pinMode(uint8_t pin, uint8_t output) ;
  void digitalWriteFast(uint8_t pin, uint8_t out) ;
  void digitalWrite(uint8_t pin, uint8_t out) ;
  uint8_t digitalReadFast(uint8_t pin) ;
  uint8_t digitalRead(uint8_t pin) ;
 static void read_control_switch() ;
  static uint8_t readSwitch(void);
  static void reset_control_switch() ;
 static void read_trim_switch() ;
 static void save_trim() ;
 static void auto_trim() ;
 static void failsafe_on_event() ;
 static void failsafe_off_event() ;
  static void low_battery_event(void) ;
 void failsafe_enable() ;
 void failsafe_disable() ;
 void failsafe_check(uint32_t tnow) ;
  void init_flip() ;
  void roll_flip() ;
 static void read_inertia() ;
  static void update_lights() ;
  static void update_GPS_light(void) ;
  static void update_motor_light(void) ;
  static void dancing_light() ;
 static void clear_leds() ;
 static void update_copter_leds(void) ;
  static void copter_leds_reset(void) ;
  static void copter_leds_on(void) ;
  static void copter_leds_off(void) ;
  static void copter_leds_slow_blink(void) ;
  static void copter_leds_fast_blink(void) ;
   static void copter_leds_oscillate(void) ;
    static void copter_leds_GPS_on(void) ;
  static void copter_leds_GPS_off(void) ;
  static void copter_leds_GPS_slow_blink(void) ;
  static void copter_leds_GPS_fast_blink(void) ;
  static void copter_leds_aux_off(void);
  static void copter_leds_aux_on(void);
  void piezo_on();
  void piezo_off();
  void piezo_beep();
  void set_recovery_home_alt() ;
  static void limits_loop() ;
  void limits_send_mavlink_status(mavlink_channel_t chan) ;
 static void arm_motors() ;
   static void init_arm_motors() ;
   static void init_disarm_motors() ;
 static void set_servos_4() ;
 static void update_navigation() ;
 static void run_nav_updates(void) ;
 static void calc_velocity_and_position();
 static void calc_distance_and_bearing() ;
  static void calc_location_error(struct Location *next_loc) ;
 static void run_autopilot() ;
 static bool set_nav_mode(uint8_t new_nav_mode) ;
 static void update_nav_mode() ;
  static bool check_missed_wp() ;
 static void calc_loiter(int16_t x_error, int16_t y_error) ;
 static void calc_nav_rate(int16_t max_speed) ;
 static void calc_nav_pitch_roll() ;
  static int16_t get_desired_speed(int16_t max_speed) ;
  static void reset_desired_speed() ;
  static void update_crosstrack(void) ;
  static void force_new_altitude(int32_t new_alt) ;
  static void set_new_altitude(int32_t new_alt) ;
  static void verify_altitude() ;
 static void reset_nav_params(void) ;
  static int32_t wrap_360(int32_t error) ;
  static int32_t wrap_180(int32_t error) ;
 static int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec) ;
 static void get_loiter_accel(int16_t accel_req_forward, int16_t accel_req_right) ;
 static void get_loiter_accel_lat_lon(int16_t accel_lat, int16_t accel_lon) ;
 static void get_loiter_vel_lat_lon(int16_t vel_lat, int16_t vel_lon, float dt) ;
 static void get_loiter_pos_lat_lon(int32_t target_lat, int32_t target_lon, float dt) ;
 static void loiter_set_pos_from_velocity(int16_t vel_forward_cms, int16_t vel_right_cms, float dt) ;
 static void loiter_set_target(float lat_from_home_cm, float lon_from_home_cm) ;
 void perf_info_reset() ;
 void perf_info_check_loop_time(uint32_t time_in_micros) ;
 uint16_t perf_info_get_num_loops() ;
 uint32_t perf_info_get_max_time() ;
 uint16_t perf_info_get_num_long_running() ;
  static void default_dead_zones() ;
  static void init_rc_in() ;
  static void init_rc_out() ;
  void output_min() ;
 static void read_radio() ;
 static void set_throttle_and_failsafe(uint16_t throttle_pwm) ;
  static void trim_radio() ;
  static void ReadSCP1000(void) ;
 static void init_sonar(void) ;
  static void init_barometer(void) ;
 static int32_t read_barometer(void) ;
 static int16_t read_sonar(void) ;
  static void init_compass() ;
  static void init_optflow() ;
 static void read_battery(void) ;
 void read_receiver_rssi(void) ;
  static void setup_wait_key(void) ;
  static void clear_offsets() ;
  static void report_batt_monitor() ;
  static void report_sonar() ;
  static void report_frame() ;
  static void report_radio() ;
  static void report_ins() ;
  static void report_compass() ;
  static void report_flight_modes() ;
  void report_optflow() ;
 static void report_heli() ;
  static void report_gyro() ;
  static void print_radio_values() ;
  static void print_switch(uint8_t p, uint8_t m, bool b) ;
  static void print_done() ;
   static void zero_eeprom(void) ;
  static void print_accel_offsets_and_scaling(void) ;
  static void print_gyro_offsets(void) ;
  static RC_Channel * heli_get_servo(int16_t servo_num);
 static int16_t read_num_from_serial() ;
  static void print_blanks(int16_t num) ;
  static void print_divider(void) ;
  static void print_enabled(bool b) ;
   static void init_esc() ;
  static void print_wp(struct Location *cmd, uint8_t index) ;
  static void report_version() ;
   static void report_tuning() ;
  static void init_ardupilot() ;
 static void init_ap_limits() ;
 static void startup_ground(void) ;
 static void set_mode(uint8_t mode) ;
  static void init_simple_bearing() ;
 static bool check_startup_for_CLI() ;
 static uint32_t map_baudrate(int8_t rate, uint32_t default_baud) ;
 static void check_usb_mux(void) ;
 void flash_leds(bool on) ;
 uint16_t board_voltage(void) ;
 static void reboot_apm(void) ;
 static void print_flight_mode(uint8_t mode) ;
  static void print_hit_enter() ;
  static void print_test_disabled() ;
 void update_toy_throttle() ;
 void update_toy_altitude() ;
 void edf_toy() ;
 void roll_pitch_toy() ;
#line 125 "./Firmware/ACopter32/ArduCopter.pde"
AP_HAL::BetterStream* cliSerial;

// N.B. we need to keep a static declaration which isn't guarded by macros
// at the top to cooperate with the prototype mangler. 

////////////////////////////////////////////////////////////////////////////////
// AP_HAL instance
////////////////////////////////////////////////////////////////////////////////

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters g;

// main loop scheduler
AP_Scheduler scheduler;

////////////////////////////////////////////////////////////////////////////////
// prototypes
////////////////////////////////////////////////////////////////////////////////
static void update_events(void);

////////////////////////////////////////////////////////////////////////////////
// Dataflash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
DataFlash_MP32 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
DataFlash_SITL DataFlash;
#else
DataFlash_Empty DataFlash;
#endif


////////////////////////////////////////////////////////////////////////////////
// the rate we run the main loop at
////////////////////////////////////////////////////////////////////////////////
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_200HZ;

////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal flight mode. Real sensors are used.
// - HIL Attitude mode. Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode. Synthetic sensors are configured that
//   supply data from the simulation.
//

// All GPS access should be through this pointer.
static GPS         *g_gps;

// flight modes convenience array
static AP_Int8 *flight_modes = &g.flight_mode1;

#if HIL_MODE == HIL_MODE_DISABLED

 #if CONFIG_ADC == ENABLED
AP_ADC_ADS7844 adc;
 #endif

 #if CONFIG_IMU_TYPE == CONFIG_IMU_MPU6000
AP_InertialSensor_MPU6000 ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_OILPAN
AP_InertialSensor_Oilpan ins(&adc);
#elif CONFIG_IMU_TYPE == CONFIG_IMU_SITL
AP_InertialSensor_Stub ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_PX4
AP_InertialSensor_PX4 ins;
 #endif

 #if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 // When building for SITL we use the HIL barometer and compass drivers
AP_Baro_BMP085_HIL barometer;
AP_Compass_HIL compass;
SITL sitl;
 #else
// Otherwise, instantiate a real barometer and compass driver
  #if CONFIG_BARO == AP_BARO_BMP085
AP_Baro_BMP085 barometer;
  #elif CONFIG_BARO == AP_BARO_PX4
AP_Baro_PX4 barometer;
  #elif CONFIG_BARO == AP_BARO_MS5611
   #if CONFIG_MS5611_SERIAL == AP_BARO_MS5611_SPI
AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
   #elif CONFIG_MS5611_SERIAL == AP_BARO_MS5611_I2C
AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
   #else
    #error Unrecognized CONFIG_MS5611_SERIAL setting.
   #endif
  #endif

 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
AP_Compass_PX4 compass;
 #else
AP_Compass_HMC5843 compass;
 #endif
 #endif

 #if OPTFLOW == ENABLED
AP_OpticalFlow_ADNS3080 optflow;
 #else
AP_OpticalFlow optflow;
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

 #if DMP_ENABLED == ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_APM2
AP_AHRS_MPU6000  ahrs(&ins, g_gps);               // only works with APM2
 #else
AP_AHRS_DCM ahrs(&ins, g_gps);
 #endif

// ahrs2 object is the secondary ahrs to allow running DMP in parallel with DCM
  #if SECONDARY_DMP_ENABLED == ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_APM2
AP_AHRS_MPU6000  ahrs2(&ins, g_gps);               // only works with APM2
  #endif

#elif HIL_MODE == HIL_MODE_SENSORS
// sensor emulators
AP_ADC_HIL              adc;
AP_Baro_BMP085_HIL      barometer;
AP_Compass_HIL          compass;
AP_GPS_HIL              g_gps_driver;
AP_InertialSensor_Stub  ins;
AP_AHRS_DCM             ahrs(&ins, g_gps);


static int32_t gps_base_alt;

#elif HIL_MODE == HIL_MODE_ATTITUDE
AP_ADC_HIL              adc;
AP_InertialSensor_Stub  ins;
AP_AHRS_HIL             ahrs(&ins, g_gps);
AP_GPS_HIL              g_gps_driver;
AP_Compass_HIL          compass;                  // never used
AP_Baro_BMP085_HIL      barometer;

 #if OPTFLOW == ENABLED
  #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
AP_OpticalFlow_ADNS3080 optflow;
  #else
AP_OpticalFlow_ADNS3080 optflow;
  #endif    // CONFIG_HAL_BOARD == HAL_BOARD_APM2
 #endif     // OPTFLOW == ENABLED

static int32_t gps_base_alt;
#else
 #error Unrecognised HIL_MODE setting.
#endif // HIL MODE

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
GCS_MAVLINK gcs0;
GCS_MAVLINK gcs3;

////////////////////////////////////////////////////////////////////////////////
// SONAR selection
////////////////////////////////////////////////////////////////////////////////
//
ModeFilterInt16_Size3 sonar_mode_filter(1);
#if CONFIG_SONAR == ENABLED
AP_HAL::AnalogSource *sonar_analog_source;
AP_RangeFinder_MaxsonarXL *sonar;
#endif

////////////////////////////////////////////////////////////////////////////////
// User variables
////////////////////////////////////////////////////////////////////////////////
#ifdef USERHOOK_VARIABLES
 #include USERHOOK_VARIABLES
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

/* Radio values
 *               Channel assignments
 *                       1	Ailerons (rudder if no ailerons)
 *                       2	Elevator
 *                       3	Throttle
 *                       4	Rudder (if we have ailerons)
 *                       5	Mode - 3 position switch
 *                       6  User assignable
 *                       7	trainer switch - sets throttle nominal (toggle switch), sets accels to Level (hold > 1 second)
 *                       8	TBD
 *               Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
 *               See libraries/RC_Channel/RC_Channel_aux.h for more information
 */

//Documentation of GLobals:
static union {
    struct {
        uint8_t home_is_set        : 1; // 1
        uint8_t simple_mode        : 1; // 2    // This is the state of simple mode
        uint8_t manual_attitude    : 1; // 3
        uint8_t manual_throttle    : 1; // 4

        uint8_t low_battery        : 1; // 5    // Used to track if the battery is low - LED output flashes when the batt is low
        uint8_t loiter_override    : 1; // 6    // Are we navigating while holding a positon? This is set to false once the speed drops below 1m/s
        uint8_t armed              : 1; // 7
        uint8_t auto_armed         : 1; // 8

        uint8_t failsafe           : 1; // 9    // A status flag for the failsafe state
        uint8_t do_flip            : 1; // 10   // Used to enable flip code
        uint8_t takeoff_complete   : 1; // 11
        uint8_t land_complete      : 1; // 12
        uint8_t compass_status     : 1; // 13
        uint8_t gps_status         : 1; // 14
        uint8_t fast_corner        : 1; // 15   // should we take the waypoint quickly or slow down?
    };
    uint16_t value;
} ap;


static struct AP_System{
    uint8_t GPS_light               : 1; // 1   // Solid indicates we have full 3D lock and can navigate, flash = read
    uint8_t motor_light             : 1; // 2   // Solid indicates Armed state
    uint8_t new_radio_frame         : 1; // 3   // Set true if we have new PWM data to act on from the Radio
    uint8_t nav_ok                  : 1; // 4   // deprecated
    uint8_t CH7_flag                : 1; // 5   // manages state of the ch7 toggle switch
    uint8_t usb_connected           : 1; // 6   // true if APM is powered from USB connection
    uint8_t alt_sensor_flag         : 1; // 7   // used to track when to read sensors vs estimate alt
    uint8_t yaw_stopped             : 1; // 8   // Used to manage the Yaw hold capabilities

} ap_system;

/*
  what navigation updated are needed
 */
static struct nav_updates {
    uint8_t need_velpos             : 1;
    uint8_t need_dist_bearing       : 1;
    uint8_t need_nav_controllers    : 1;
    uint8_t need_nav_pitch_roll     : 1;
} nav_updates;


////////////////////////////////////////////////////////////////////////////////
// velocity in lon and lat directions calculated from GPS position and accelerometer data
// updated after GPS read - 5-10hz
static int16_t lon_speed;       // expressed in cm/s.  positive numbers mean moving east
static int16_t lat_speed;       // expressed in cm/s.  positive numbers when moving north

// The difference between the desired rate of travel and the actual rate of travel
// updated after GPS read - 5-10hz
static int16_t x_rate_error;
static int16_t y_rate_error;

////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as STABILIZE, ACRO,
static int8_t control_mode = STABILIZE;
// Used to maintain the state of the previous control switch position
// This is set to -1 when we need to re-read the switch
static uint8_t oldSwitchPosition;

// receiver RSSI
static uint8_t receiver_rssi;


////////////////////////////////////////////////////////////////////////////////
// Motor Output
////////////////////////////////////////////////////////////////////////////////
#if FRAME_CONFIG == QUAD_FRAME
 #define MOTOR_CLASS AP_MotorsQuad
#endif
#if FRAME_CONFIG == TRI_FRAME
 #define MOTOR_CLASS AP_MotorsTri
#endif
#if FRAME_CONFIG == HEXA_FRAME
 #define MOTOR_CLASS AP_MotorsHexa
#endif
#if FRAME_CONFIG == Y6_FRAME
 #define MOTOR_CLASS AP_MotorsY6
#endif
#if FRAME_CONFIG == OCTA_FRAME
 #define MOTOR_CLASS AP_MotorsOcta
#endif
#if FRAME_CONFIG == OCTA_QUAD_FRAME
 #define MOTOR_CLASS AP_MotorsOctaQuad
#endif
#if FRAME_CONFIG == HELI_FRAME
 #define MOTOR_CLASS AP_MotorsHeli
#endif

#if FRAME_CONFIG == HELI_FRAME  // helicopter constructor requires more arguments
MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4, &g.rc_8, &g.heli_servo_1, &g.heli_servo_2, &g.heli_servo_3, &g.heli_servo_4);
#elif FRAME_CONFIG == TRI_FRAME  // tri constructor requires additional rc_7 argument to allow tail servo reversing
MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4, &g.rc_7);
#else
MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4);
#endif

////////////////////////////////////////////////////////////////////////////////
// PIDs
////////////////////////////////////////////////////////////////////////////////
// This is a convienience accessor for the IMU roll rates. It's currently the raw IMU rates
// and not the adjusted omega rates, but the name is stuck
static Vector3f omega;
// This is used to hold radio tuning values for in-flight CH6 tuning
float tuning_value;
// used to limit the rate that the pid controller output is logged so that it doesn't negatively affect performance
static uint8_t pid_log_counter;

////////////////////////////////////////////////////////////////////////////////
// LED output
////////////////////////////////////////////////////////////////////////////////
// This is current status for the LED lights state machine
// setting this value changes the output of the LEDs
static uint8_t led_mode = NORMAL_LEDS;
// Blinking indicates GPS status
static uint8_t copter_leds_GPS_blink;
// Blinking indicates battery status
static uint8_t copter_leds_motor_blink;
// Navigation confirmation blinks
static int8_t copter_leds_nav_blink;

////////////////////////////////////////////////////////////////////////////////
// GPS variables
////////////////////////////////////////////////////////////////////////////////
// This is used to scale GPS values for EEPROM storage
// 10^7 times Decimal GPS means 1 == 1cm
// This approximation makes calculations integer and it's easy to read
static const float t7 = 10000000.0;
// We use atan2 and other trig techniques to calaculate angles
// We need to scale the longitude up to make these calcs work
// to account for decreasing distance between lines of longitude away from the equator
static float scaleLongUp = 1;
// Sometimes we need to remove the scaling for distance calcs
static float scaleLongDown = 1;


////////////////////////////////////////////////////////////////////////////////
// Mavlink specific
////////////////////////////////////////////////////////////////////////////////
// Used by Mavlink for unknow reasons
static const float radius_of_earth = 6378100;   // meters

// Unions for getting byte values
union float_int {
    int32_t int_value;
    float float_value;
} float_int;


////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the "next_WP" location in degrees * 100
static int32_t wp_bearing;
// navigation mode - options include NAV_NONE, NAV_LOITER, NAV_CIRCLE, NAV_WP
static uint8_t nav_mode;
// Register containing the index of the current navigation command in the mission script
static int16_t command_nav_index;
// Register containing the index of the previous navigation command in the mission script
// Used to manage the execution of conditional commands
static uint8_t prev_nav_index;
// Register containing the index of the current conditional command in the mission script
static uint8_t command_cond_index;
// Used to track the required WP navigation information
// options include
// NAV_ALTITUDE - have we reached the desired altitude?
// NAV_LOCATION - have we reached the desired location?
// NAV_DELAY    - have we waited at the waypoint the desired time?
static uint8_t wp_verify_byte;                                                  // used for tracking state of navigating waypoints
// used to limit the speed ramp up of WP navigation
// Acceleration is limited to 1m/s/s
static int16_t max_speed_old;
// Used to track how many cm we are from the "next_WP" location
static int32_t long_error, lat_error;
static int16_t control_roll;
static int16_t control_pitch;
static uint8_t rtl_state;

////////////////////////////////////////////////////////////////////////////////
// Orientation
////////////////////////////////////////////////////////////////////////////////
// Convienience accessors for commonly used trig functions. These values are generated
// by the DCM through a few simple equations. They are used throughout the code where cos and sin
// would normally be used.
// The cos values are defaulted to 1 to get a decent initial value for a level state
static float cos_roll_x         = 1;
static float cos_pitch_x        = 1;
static float cos_yaw_x          = 1;
static float sin_yaw_y          = 1;
static float cos_yaw            = 1;
static float sin_yaw            = 1;
static float sin_roll           = 1;
static float sin_pitch          = 1;

////////////////////////////////////////////////////////////////////////////////
// SIMPLE Mode
////////////////////////////////////////////////////////////////////////////////
// Used to track the orientation of the copter for Simple mode. This value is reset at each arming
// or in SuperSimple mode when the copter leaves a 20m radius from home.
static int32_t initial_simple_bearing;

////////////////////////////////////////////////////////////////////////////////
// Rate contoller targets
////////////////////////////////////////////////////////////////////////////////
static uint8_t rate_targets_frame = EARTH_FRAME;    // indicates whether rate targets provided in earth or body frame
static int32_t roll_rate_target_ef = 0;
static int32_t pitch_rate_target_ef = 0;
static int32_t yaw_rate_target_ef = 0;
static int32_t roll_rate_target_bf = 0;     // body frame roll rate target
static int32_t pitch_rate_target_bf = 0;    // body frame pitch rate target
static int32_t yaw_rate_target_bf = 0;      // body frame yaw rate target

////////////////////////////////////////////////////////////////////////////////
// Throttle variables
////////////////////////////////////////////////////////////////////////////////
static int16_t throttle_accel_target_ef;    // earth frame throttle acceleration target
static bool throttle_accel_controller_active;   // true when accel based throttle controller is active, false when higher level throttle controllers are providing throttle output directly
static float throttle_avg;                  // g.throttle_cruise as a float
static int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only


////////////////////////////////////////////////////////////////////////////////
// ACRO Mode
////////////////////////////////////////////////////////////////////////////////
// Used to control Axis lock
int32_t roll_axis;
int32_t pitch_axis;

// Filters
AP_LeadFilter xLeadFilter;      // Long GPS lag filter
AP_LeadFilter yLeadFilter;      // Lat  GPS lag filter
#if FRAME_CONFIG == HELI_FRAME
LowPassFilterFloat rate_roll_filter;    // Rate Roll filter
LowPassFilterFloat rate_pitch_filter;   // Rate Pitch filter
// LowPassFilterFloat rate_yaw_filter;     // Rate Yaw filter
#endif // HELI_FRAME

// Barometer filter
AverageFilterInt32_Size5 baro_filter;

////////////////////////////////////////////////////////////////////////////////
// Circle Mode / Loiter control
////////////////////////////////////////////////////////////////////////////////
// used to determin the desired location in Circle mode
// increments at circle_rate / second
static float circle_angle;
// used to control the speed of Circle mode
// units are in radians, default is 5° per second
static const float circle_rate = 0.0872664625;
// used to track the delat in Circle Mode
static int32_t old_wp_bearing;
// deg : how many times to circle * 360 for Loiter/Circle Mission command
static int16_t loiter_total;
// deg : how far we have turned around a waypoint
static int16_t loiter_sum;
// How long we should stay in Loiter Mode for mission scripting
static uint16_t loiter_time_max;
// How long have we been loitering - The start time in millis
static uint32_t loiter_time;
// The synthetic location created to make the copter do circles around a WP
static struct   Location circle_WP;
// inertial nav loiter variables
static float loiter_lat_from_home_cm;
static float loiter_lon_from_home_cm;


////////////////////////////////////////////////////////////////////////////////
// CH7 control
////////////////////////////////////////////////////////////////////////////////
// This register tracks the current Mission Command index when writing
// a mission using CH7 in flight
static int8_t CH7_wp_index;


////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
// Battery Voltage of battery, initialized above threshold for filter
static float battery_voltage1 = LOW_VOLTAGE * 1.05f;
// refers to the instant amp draw – based on an Attopilot Current sensor
static float current_amps1;
// refers to the total amps drawn – based on an Attopilot Current sensor
static float current_total1;


////////////////////////////////////////////////////////////////////////////////
// Altitude
////////////////////////////////////////////////////////////////////////////////
// The (throttle) controller desired altitude in cm
static float controller_desired_alt;
// The cm we are off in altitude from next_WP.alt – Positive value means we are below the WP
static int32_t altitude_error;
// The cm/s we are moving up or down based on sensor data - Positive = UP
static int16_t climb_rate_actual;
// Used to dither our climb_rate over 50hz
static int16_t climb_rate_error;
// The cm/s we are moving up or down based on filtered data - Positive = UP
static int16_t climb_rate;
// The altitude as reported by Sonar in cm – Values are 20 to 700 generally.
static int16_t sonar_alt;
static uint8_t sonar_alt_health;   // true if we can trust the altitude from the sonar
// The climb_rate as reported by sonar in cm/s
static int16_t sonar_rate;
// The altitude as reported by Baro in cm – Values can be quite high
static int32_t baro_alt;
// The climb_rate as reported by Baro in cm/s
static int16_t baro_rate;

static int16_t saved_toy_throttle;


////////////////////////////////////////////////////////////////////////////////
// flight modes
////////////////////////////////////////////////////////////////////////////////
// Flight modes are combinations of Roll/Pitch, Yaw and Throttle control modes
// Each Flight mode is a unique combination of these modes
//
// The current desired control scheme for Yaw
static uint8_t yaw_mode;
// The current desired control scheme for roll and pitch / navigation
static uint8_t roll_pitch_mode;
// The current desired control scheme for altitude hold
static uint8_t throttle_mode;


////////////////////////////////////////////////////////////////////////////////
// flight specific
////////////////////////////////////////////////////////////////////////////////
// An additional throttle added to keep the copter at the same altitude when banking
static int16_t angle_boost;
// counter to verify landings
static uint16_t land_detector;


////////////////////////////////////////////////////////////////////////////////
// Navigation general
////////////////////////////////////////////////////////////////////////////////
// The location of home in relation to the copter, updated every GPS read
static int32_t home_bearing;
// distance between plane and home in cm
static int32_t home_distance;
// distance between plane and next_WP in cm
// is not static because AP_Camera uses it
uint32_t wp_distance;

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
////////////////////////////////////////////////////////////////////////////////
// home location is stored when we have a good GPS lock and arm the copter
// Can be reset each the copter is re-armed
static struct   Location home;
// Current location of the copter
static struct   Location current_loc;
// Next WP is the desired location of the copter - the next waypoint or loiter location
static struct   Location next_WP;
// Prev WP is used to get the optimum path from one WP to the next
static struct   Location prev_WP;
// Holds the current loaded command from the EEPROM for navigation
static struct   Location command_nav_queue;
// Holds the current loaded command from the EEPROM for conditional scripts
static struct   Location command_cond_queue;
// Holds the current loaded command from the EEPROM for guided mode
static struct   Location guided_WP;


////////////////////////////////////////////////////////////////////////////////
// Crosstrack
////////////////////////////////////////////////////////////////////////////////
// deg * 100, The original angle to the next_WP when the next_WP was set
// Also used to check when we pass a WP
static int32_t original_wp_bearing;
// The amount of angle correction applied to wp_bearing to bring the copter back on its optimum path
static int16_t crosstrack_error;


////////////////////////////////////////////////////////////////////////////////
// Navigation Roll/Pitch functions
////////////////////////////////////////////////////////////////////////////////
// all angles are deg * 100 : target yaw angle
// The Commanded ROll from the autopilot.
static int32_t nav_roll;
// The Commanded pitch from the autopilot. negative Pitch means go forward.
static int32_t nav_pitch;
// The desired bank towards North (Positive) or South (Negative)
static int32_t auto_roll;
static int32_t auto_pitch;

// Don't be fooled by the fact that Pitch is reversed from Roll in its sign!
static int16_t nav_lat;
// The desired bank towards East (Positive) or West (Negative)
static int16_t nav_lon;
// The Commanded ROll from the autopilot based on optical flow sensor.
static int32_t of_roll;
// The Commanded pitch from the autopilot based on optical flow sensor. negative Pitch means go forward.
static int32_t of_pitch;


////////////////////////////////////////////////////////////////////////////////
// Navigation Throttle control
////////////////////////////////////////////////////////////////////////////////
// The Commanded Throttle from the autopilot.
static int16_t nav_throttle;    // 0-1000 for throttle control
// This is a simple counter to track the amount of throttle used during flight
// This could be useful later in determining and debuging current usage and predicting battery life
static uint32_t throttle_integrator;

////////////////////////////////////////////////////////////////////////////////
// Climb rate control
////////////////////////////////////////////////////////////////////////////////
// Time when we intiated command in millis - used for controlling decent rate
// Used to track the altitude offset for climbrate control
static int8_t alt_change_flag;

////////////////////////////////////////////////////////////////////////////////
// Navigation Yaw control
////////////////////////////////////////////////////////////////////////////////
// The Commanded Yaw from the autopilot.
static int32_t nav_yaw;
static uint8_t yaw_timer;
// Yaw will point at this location if yaw_mode is set to YAW_LOOK_AT_LOCATION
static struct Location yaw_look_at_WP;
// bearing from current location to the yaw_look_at_WP
static int32_t yaw_look_at_WP_bearing;
// yaw used for YAW_LOOK_AT_HEADING yaw_mode
static int32_t yaw_look_at_heading;
// Deg/s we should turn
static int16_t yaw_look_at_heading_slew;



////////////////////////////////////////////////////////////////////////////////
// Repeat Mission Scripting Command
////////////////////////////////////////////////////////////////////////////////
// The type of repeating event - Toggle a servo channel, Toggle the APM1 relay, etc
static uint8_t event_id;
// Used to manage the timimng of repeating events
static uint32_t event_timer;
// How long to delay the next firing of event in millis
static uint16_t event_delay;
// how many times to fire : 0 = forever, 1 = do once, 2 = do twice
static int16_t event_repeat;
// per command value, such as PWM for servos
static int16_t event_value;
// the stored value used to undo commands - such as original PWM command
static int16_t event_undo_value;

////////////////////////////////////////////////////////////////////////////////
// Delay Mission Scripting Command
////////////////////////////////////////////////////////////////////////////////
static int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
static uint32_t condition_start;


////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// Integration time for the gyros (DCM algorithm)
// Updated with the fast loop
static float G_Dt = 0.02;

////////////////////////////////////////////////////////////////////////////////
// Inertial Navigation
////////////////////////////////////////////////////////////////////////////////
#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
AP_InertialNav inertial_nav(&ahrs, &ins, &barometer, &g_gps);
#endif

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
// The number of GPS fixes we have had
static uint8_t gps_fix_count;

// System Timers
// --------------
// Time in microseconds of main control loop
static uint32_t fast_loopTimer;
// Counters for branching from 10 hz control loop
static uint8_t medium_loopCounter;
// Counters for branching from 3 1/3hz control loop
static uint8_t slow_loopCounter;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;
// Delta Time in milliseconds for navigation computations, updated with every good GPS read
static float dTnav;
// Counters for branching from 4 minute control loop used to save Compass offsets
static int16_t superslow_loopCounter;
// Loiter timer - Records how long we have been in loiter
static uint32_t rtl_loiter_start_time;
// disarms the copter while in Acro or Stabilize mode after 30 seconds of no flight
static uint8_t auto_disarming_counter;
// prevents duplicate GPS messages from entering system
static uint32_t last_gps_time;

// Used to exit the roll and pitch auto trim function
static uint8_t auto_trim_counter;

// Reference to the relay object (APM1 -> PORTL 2) (APM2 -> PORTB 7)
AP_Relay relay;

//Reference to the camera object (it uses the relay object inside it)
#if CAMERA == ENABLED
  AP_Camera camera(&relay);
#endif

// a pin for reading the receiver RSSI voltage. The scaling by 0.25
// is to take the 0 to 1024 range down to an 8 bit range for MAVLink
AP_HAL::AnalogSource* rssi_analog_source;


// Input sources for battery voltage, battery current, board vcc
AP_HAL::AnalogSource* batt_volt_analog_source;
AP_HAL::AnalogSource* batt_curr_analog_source;
AP_HAL::AnalogSource* board_vcc_analog_source;


#if CLI_ENABLED == ENABLED
    static int8_t   setup_show (uint8_t argc, const Menu::arg *argv);
#endif

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


////////////////////////////////////////////////////////////////////////////////
// Experimental AP_Limits library - set constraints, limits, fences, minima, maxima on various parameters
////////////////////////////////////////////////////////////////////////////////
#if AP_LIMITS == ENABLED
AP_Limits               limits;
AP_Limit_GPSLock        gpslock_limit(g_gps);
AP_Limit_Geofence       geofence_limit(FENCE_START_BYTE, FENCE_WP_SIZE, MAX_FENCEPOINTS, g_gps, &home, &current_loc);
AP_Limit_Altitude       altitude_limit(&current_loc);
#endif

////////////////////////////////////////////////////////////////////////////////
// function definitions to keep compiler from complaining about undeclared functions
////////////////////////////////////////////////////////////////////////////////
void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

// setup the var_info table
AP_Param param_loader(var_info, WP_START_BYTE);

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { update_GPS,            2,     900 },
    { update_navigation,     2,     500 },
    { medium_loop,           2,     700 },
    { update_altitude_est,   2,    1000 },
    { fifty_hz_loop,         2,     950 },
    { run_nav_updates,       2,     500 },
    { slow_loop,            10,     500 },
    { gcs_check_input,	     2,     700 },
    { gcs_send_heartbeat,  100,     700 },
    { gcs_data_stream_send,  2,    1500 },
    { gcs_send_deferred,     2,    1200 },
    { compass_accumulate,    2,     700 },
    { barometer_accumulate,  2,     900 },
    { super_slow_loop,     100,    1100 },
    { perf_update,        1000,     500 }
};

static bool CH7_toy_flag;

#if TOY_MIXER == TOY_LOOKUP_TABLE
static const int16_t toy_lookup[] = {
    186,    373,    558,    745,
    372,    745,    1117,   1490,
    558,    1118,   1675,   2235,
    743,    1490,   2233,   2980,
    929,    1863,   2792,   3725,
    1115,   2235,   3350,   4470,
    1301,   2608,   3908,   4500,
    1487,   2980,   4467,   4500,
    1673,   3353,   4500,   4500
};
#endif

void setup() {
    // this needs to be the first call, as it fills memory with
    // sentinel values
    memcheck_init();
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

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

    rssi_analog_source      = hal.analogin->channel(g.rssi_pin, 0.25);
    batt_volt_analog_source = hal.analogin->channel(g.battery_volt_pin);
    batt_curr_analog_source = hal.analogin->channel(g.battery_curr_pin);
    board_vcc_analog_source = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
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
  try to accumulate a baro reading
 */
static void barometer_accumulate(void)
{
    barometer.accumulate();
}

// enable this to get console logging of scheduler performance
#define SCHEDULER_DEBUG 0

static void perf_update(void)
{
    if (g.log_bitmask & MASK_LOG_PM)
        Log_Write_Performance();
    if (scheduler.debug()) {
        cliSerial->printf_P(PSTR("PERF: %u/%u %lu\n"), 
                            (unsigned)perf_info_get_num_long_running(),
                            (unsigned)perf_info_get_num_loops(),
                            (unsigned long)perf_info_get_max_time());
    }
    perf_info_reset();
    gps_fix_count = 0;
}

void loop()
{
    uint32_t timer = micros();

    // We want this to execute fast
    // ----------------------------
    if (ins.num_samples_available() >= 2) {

        // check loop time
        perf_info_check_loop_time(timer - fast_loopTimer);

        G_Dt                            = (float)(timer - fast_loopTimer) / 1000000.f;                  // used by PI Loops
        fast_loopTimer          = timer;

        // for mainloop failure monitoring
        mainLoop_count++;

        // Execute the fast loop
        // ---------------------
        fast_loop();

        // tell the scheduler one tick has passed
        scheduler.tick();
    } else {
        uint16_t dt = timer - fast_loopTimer;
        if (dt < 10000) {
            uint16_t time_to_next_loop = 10000 - dt;
            scheduler.run(time_to_next_loop);
        }
    }
}


// Main loop - 100hz
static void fast_loop()
{
    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

    // reads all of the necessary trig functions for cameras, throttle, etc.
    // --------------------------------------------------------------------
    update_trig();

    // run low level rate controllers that only require IMU data
    run_rate_controllers();

    // write out the servo PWM values
    // ------------------------------
    set_servos_4();

    // Inertial Nav
    // --------------------
    read_inertia();

    // optical flow
    // --------------------
#if OPTFLOW == ENABLED
    if(g.optflow_enabled) {
        update_optical_flow();
    }
#endif  // OPTFLOW == ENABLED

    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();

    // custom code/exceptions for flight modes
    // ---------------------------------------
    update_yaw_mode();
    update_roll_pitch_mode();

    // update targets to rate controllers
    update_rate_contoller_targets();

    // agmatthews - USERHOOKS
#ifdef USERHOOK_FASTLOOP
    USERHOOK_FASTLOOP
#endif

}

static void medium_loop()
{
    // This is the start of the medium (10 Hz) loop pieces
    // -----------------------------------------
    switch(medium_loopCounter) {

    // This case deals with the GPS and Compass
    //-----------------------------------------
    case 0:
        medium_loopCounter++;

#if HIL_MODE != HIL_MODE_ATTITUDE                                                               // don't execute in HIL mode
        if(g.compass_enabled) {
            if (compass.read()) {
                compass.null_offsets();
            }
        }
#endif

        // auto_trim - stores roll and pitch radio inputs to ahrs
        auto_trim();

        // record throttle output
        // ------------------------------
        throttle_integrator += g.rc_3.servo_out;
        break;

    // This case performs some navigation computations
    //------------------------------------------------
    case 1:
        medium_loopCounter++;
        read_receiver_rssi();
        break;

    // command processing
    //-------------------
    case 2:
        medium_loopCounter++;

        if(control_mode == TOY_A) {
            update_toy_throttle();

            if(throttle_mode == THROTTLE_AUTO) {
                update_toy_altitude();
            }
        }

        ap_system.alt_sensor_flag = true;
        break;

    // This case deals with sending high rate telemetry
    //-------------------------------------------------
    case 3:
        medium_loopCounter++;

        // perform next command
        // --------------------
        if(control_mode == AUTO) {
            if(ap.home_is_set && g.command_total > 1) {
                update_commands();
            }
        }

        if(motors.armed()) {
            if (g.log_bitmask & MASK_LOG_ATTITUDE_MED) {
                Log_Write_Attitude();
#if SECONDARY_DMP_ENABLED == ENABLED
                Log_Write_DMP();
#endif
            }

            if (g.log_bitmask & MASK_LOG_MOTORS)
                Log_Write_Motors();
        }
        break;

    // This case controls the slow loop
    //---------------------------------
    case 4:
        medium_loopCounter = 0;

        if (g.battery_monitoring != 0) {
            read_battery();
        }

        // Accel trims      = hold > 2 seconds
        // Throttle cruise  = switch less than 1 second
        // --------------------------------------------
        read_trim_switch();

        // Check for engine arming
        // -----------------------
        arm_motors();

        // agmatthews - USERHOOKS
#ifdef USERHOOK_MEDIUMLOOP
        USERHOOK_MEDIUMLOOP
#endif

#if COPTER_LEDS == ENABLED
        update_copter_leds();
#endif
        break;

    default:
        // this is just a catch all
        // ------------------------
        medium_loopCounter = 0;
        break;
    }
}

// stuff that happens at 50 hz
// ---------------------------
static void fifty_hz_loop()
{
    // Update the throttle ouput
    // -------------------------
    update_throttle_mode();

#if TOY_EDF == ENABLED
    edf_toy();
#endif

#ifdef USERHOOK_50HZLOOP
    USERHOOK_50HZLOOP
#endif


#if HIL_MODE != HIL_MODE_DISABLED && FRAME_CONFIG != HELI_FRAME
    // HIL for a copter needs very fast update of the servo values
    gcs_send_message(MSG_RADIO_OUT);
#endif

#if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update_mount_position();
#endif

#if MOUNT2 == ENABLED
    // update camera mount's position
    camera_mount2.update_mount_position();
#endif

#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
#endif

# if HIL_MODE == HIL_MODE_DISABLED
    if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST && motors.armed()) {
        Log_Write_Attitude();
#if SECONDARY_DMP_ENABLED == ENABLED
        Log_Write_DMP();
#endif
    }

    if (g.log_bitmask & MASK_LOG_IMU && motors.armed())
        Log_Write_IMU();
#endif

}


static void slow_loop()
{

#if AP_LIMITS == ENABLED

    // Run the AP_Limits main loop
    limits_loop();

#endif // AP_LIMITS_ENABLED

    // This is the slow (3 1/3 Hz) loop pieces
    //----------------------------------------
    switch (slow_loopCounter) {
    case 0:
        slow_loopCounter++;
        superslow_loopCounter++;

        // record if the compass is healthy
        set_compass_healthy(compass.healthy);

        if(superslow_loopCounter > 1200) {
#if HIL_MODE != HIL_MODE_ATTITUDE
            if(g.rc_3.control_in == 0 && control_mode == STABILIZE && g.compass_enabled) {
                compass.save_offsets();
                superslow_loopCounter = 0;
            }
#endif
        }


        if(motors.armed()) {
            if (g.log_bitmask & MASK_LOG_ITERM)
                Log_Write_Iterm();
        }else{
            // check the user hasn't updated the frame orientation
            motors.set_frame_orientation(g.frame_orientation);
        }

        break;

    case 1:
        slow_loopCounter++;

#if MOUNT == ENABLED
        update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_10, &g.rc_11);
#endif
        enable_aux_servos();

#if MOUNT == ENABLED
        camera_mount.update_mount_type();
#endif

#if MOUNT2 == ENABLED
        camera_mount2.update_mount_type();
#endif

        // agmatthews - USERHOOKS
#ifdef USERHOOK_SLOWLOOP
        USERHOOK_SLOWLOOP
#endif

        break;

    case 2:
        slow_loopCounter = 0;
        update_events();

        // blink if we are armed
        update_lights();

        if(g.radio_tuning > 0)
            tuning();

#if USB_MUX_PIN > 0
        check_usb_mux();
#endif
        break;

    default:
        slow_loopCounter = 0;
        break;
    }
}

#define AUTO_DISARMING_DELAY 25
// 1Hz loop
static void super_slow_loop()
{
    if (g.log_bitmask != 0) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    if (g.log_bitmask & MASK_LOG_CURRENT && motors.armed())
        Log_Write_Current();

    // this function disarms the copter if it has been sitting on the ground for any moment of time greater than 25 seconds
    // but only of the control mode is manual
    if((control_mode <= ACRO) && (g.rc_3.control_in == 0)) {
        auto_disarming_counter++;

        if(auto_disarming_counter == AUTO_DISARMING_DELAY) {
            init_disarm_motors();
        }else if (auto_disarming_counter > AUTO_DISARMING_DELAY) {
            auto_disarming_counter = AUTO_DISARMING_DELAY + 1;
        }
    }else{
        auto_disarming_counter = 0;
    }

    // agmatthews - USERHOOKS
#ifdef USERHOOK_SUPERSLOWLOOP
    USERHOOK_SUPERSLOWLOOP
#endif
}

// called at 100hz but data from sensor only arrives at 20 Hz
#if OPTFLOW == ENABLED
static void update_optical_flow(void)
{
    static uint32_t last_of_update = 0;
    static uint8_t of_log_counter = 0;

    // if new data has arrived, process it
    if( optflow.last_update != last_of_update ) {
        last_of_update = optflow.last_update;
        optflow.update_position(ahrs.roll, ahrs.pitch, cos_yaw_x, sin_yaw_y, current_loc.alt);      // updates internal lon and lat with estimation based on optical flow

        // write to log at 5hz
        of_log_counter++;
        if( of_log_counter >= 4 ) {
            of_log_counter = 0;
            if (g.log_bitmask & MASK_LOG_OPTFLOW) {
                Log_Write_Optflow();
            }
        }
    }
}
#endif  // OPTFLOW == ENABLED

// called at 50hz
static void update_GPS(void)
{
    // A counter that is used to grab at least 10 reads before commiting the Home location
    static uint8_t ground_start_count  = 10;

    g_gps->update();
    update_GPS_light();

    set_gps_healthy(g_gps->status() == g_gps->GPS_OK);

    if (g_gps->new_data && g_gps->fix) {
        // clear new data flag
        g_gps->new_data = false;

        // check for duiplicate GPS messages
        if(last_gps_time != g_gps->time) {

            // for performance monitoring
            // --------------------------
            gps_fix_count++;

            if(ground_start_count > 1) {
                ground_start_count--;

            } else if (ground_start_count == 1) {

                // We countdown N number of good GPS fixes
                // so that the altitude is more accurate
                // -------------------------------------
                if (g_gps->latitude == 0) {
                    ground_start_count = 5;

                }else{
                    if (g.compass_enabled) {
                        // Set compass declination automatically
                        compass.set_initial_location(g_gps->latitude, g_gps->longitude);
                    }
                    // save home to eeprom (we must have a good fix to have reached this point)
                    init_home();
                    ground_start_count = 0;
                }
            }

            if (g.log_bitmask & MASK_LOG_GPS && motors.armed()) {
                Log_Write_GPS();
            }

#if HIL_MODE == HIL_MODE_ATTITUDE                                                               // only execute in HIL mode
            ap_system.alt_sensor_flag = true;
#endif
        }

        // save GPS time so we don't get duplicate reads
        last_gps_time = g_gps->time;
    }
}

// set_yaw_mode - update yaw mode and initialise any variables required
bool set_yaw_mode(uint8_t new_yaw_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool yaw_initialised = false;

    // return immediately if no change
    if( new_yaw_mode == yaw_mode ) {
        return true;
    }

    switch( new_yaw_mode ) {
        case YAW_HOLD:
        case YAW_ACRO:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_NEXT_WP:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_LOCATION:
            if( ap.home_is_set ) {
                // update bearing - assumes yaw_look_at_WP has been intialised before set_yaw_mode was called
                yaw_look_at_WP_bearing = get_bearing_cd(&current_loc, &yaw_look_at_WP);
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_HEADING:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_HOME:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_TOY:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AHEAD:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( yaw_initialised ) {
        yaw_mode = new_yaw_mode;
    }

    // return success or failure
    return yaw_initialised;
}

// update_yaw_mode - run high level yaw controllers
// 100hz update rate
void update_yaw_mode(void)
{
    switch(yaw_mode) {

    case YAW_HOLD:
        // heading hold at heading held in nav_yaw but allow input from pilot
        get_yaw_rate_stabilized_ef(g.rc_4.control_in);
        break;

    case YAW_ACRO:
        // pilot controlled yaw using rate controller
        if(g.axis_enabled) {
            get_yaw_rate_stabilized_ef(g.rc_4.control_in);
        }else{
            get_acro_yaw(g.rc_4.control_in);
        }
        break;

    case YAW_LOOK_AT_NEXT_WP:
        // point towards next waypoint (no pilot input accepted)
        // we don't use wp_bearing because we don't want the copter to turn too much during flight
        nav_yaw = get_yaw_slew(nav_yaw, original_wp_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_LOCATION:
        // point towards a location held in yaw_look_at_WP (no pilot input accepted)
        nav_yaw = get_yaw_slew(nav_yaw, yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HOME:
        // keep heading always pointing at home with no pilot input allowed
        nav_yaw = get_yaw_slew(nav_yaw, home_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
        nav_yaw = get_yaw_slew(nav_yaw, yaw_look_at_heading, yaw_look_at_heading_slew);
        get_stabilize_yaw(nav_yaw);
        break;

	case YAW_LOOK_AHEAD:
		// Commanded Yaw to automatically look ahead.
        get_look_ahead_yaw(g.rc_4.control_in);
        break;

#if TOY_LOOKUP == TOY_EXTERNAL_MIXER
    case YAW_TOY:
        // update to allow external roll/yaw mixing
        // keep heading always pointing at home with no pilot input allowed
        nav_yaw = get_yaw_slew(nav_yaw, home_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);
        break;
#endif
    }
}

// set_roll_pitch_mode - update roll/pitch mode and initialise any variables as required
bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool roll_pitch_initialised = false;

    // return immediately if no change
    if( new_roll_pitch_mode == roll_pitch_mode ) {
        return true;
    }

    switch( new_roll_pitch_mode ) {
        case ROLL_PITCH_STABLE:
        case ROLL_PITCH_ACRO:
        case ROLL_PITCH_AUTO:
        case ROLL_PITCH_STABLE_OF:
        case ROLL_PITCH_TOY:
            roll_pitch_initialised = true;
            break;

        case ROLL_PITCH_LOITER_INAV:
            // require gps lock
            if( ap.home_is_set ) {
                roll_pitch_initialised = true;
            }
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( roll_pitch_initialised ) {
        roll_pitch_mode = new_roll_pitch_mode;
    }

    // return success or failure
    return roll_pitch_initialised;
}

// update_roll_pitch_mode - run high level roll and pitch controllers
// 100hz update rate
void update_roll_pitch_mode(void)
{
    if (ap.do_flip) {
        if(abs(g.rc_1.control_in) < 4000) {
            roll_flip();
            return;
        }else{
            // force an exit from the loop if we are not hands off sticks.
            ap.do_flip = false;
            Log_Write_Event(DATA_EXIT_FLIP);
        }
    }

    switch(roll_pitch_mode) {
    case ROLL_PITCH_ACRO:

#if FRAME_CONFIG == HELI_FRAME
		if(g.axis_enabled) {
            get_roll_rate_stabilized_ef(g.rc_1.control_in);
            get_pitch_rate_stabilized_ef(g.rc_2.control_in);
        }else{
            // ACRO does not get SIMPLE mode ability
            if (motors.flybar_mode == 1) {
                g.rc_1.servo_out = g.rc_1.control_in;
                g.rc_2.servo_out = g.rc_2.control_in;
            } else {
                get_acro_roll(g.rc_1.control_in);
                get_acro_pitch(g.rc_2.control_in);
            }
		}
#else  // !HELI_FRAME
		if(g.axis_enabled) {
            get_roll_rate_stabilized_ef(g.rc_1.control_in);
            get_pitch_rate_stabilized_ef(g.rc_2.control_in);
        }else{
            // ACRO does not get SIMPLE mode ability
            get_acro_roll(g.rc_1.control_in);
            get_acro_pitch(g.rc_2.control_in);
		}
#endif  // HELI_FRAME
        break;

    case ROLL_PITCH_STABLE:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap_system.new_radio_frame) {
            update_simple_mode();
        }

        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);

        break;

    case ROLL_PITCH_AUTO:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap_system.new_radio_frame) {
            update_simple_mode();
        }
        // mix in user control with Nav control
        nav_roll                += constrain_int32(wrap_180(auto_roll  - nav_roll),  -g.auto_slew_rate.get(), g.auto_slew_rate.get());                 // 40 deg a second
        nav_pitch               += constrain_int32(wrap_180(auto_pitch - nav_pitch), -g.auto_slew_rate.get(), g.auto_slew_rate.get());                 // 40 deg a second

        control_roll            = g.rc_1.control_mix(nav_roll);
        control_pitch           = g.rc_2.control_mix(nav_pitch);

        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);
        break;

    case ROLL_PITCH_STABLE_OF:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap_system.new_radio_frame) {
            update_simple_mode();
        }

        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

        // mix in user control with optical flow
        get_stabilize_roll(get_of_roll(control_roll));
        get_stabilize_pitch(get_of_pitch(control_pitch));
        break;

    // THOR
    // a call out to the main toy logic
    case ROLL_PITCH_TOY:
        roll_pitch_toy();
        break;

    case ROLL_PITCH_LOITER_INAV:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap_system.new_radio_frame) {
            update_simple_mode();
        }
        // copy user input for logging purposes
        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

        // update loiter target from user controls - max velocity is 5.0 m/s
        if( control_roll != 0 || control_pitch != 0 ) {
            loiter_set_pos_from_velocity(-control_pitch/(2*4.5), control_roll/(2*4.5),0.01f);
        }

        // copy latest output from nav controller to stabilize controller
        nav_roll    = auto_roll;
        nav_pitch   = auto_pitch;
        get_stabilize_roll(nav_roll);
        get_stabilize_pitch(nav_pitch);
        break;
    }

	#if FRAME_CONFIG != HELI_FRAME
    if(g.rc_3.control_in == 0 && control_mode <= ACRO) {
        reset_rate_I();
        reset_stability_I();
    }
	#endif //HELI_FRAME

    if(ap_system.new_radio_frame) {
        // clear new radio frame info
        ap_system.new_radio_frame = false;
    }
}

// new radio frame is used to make sure we only call this at 50hz
void update_simple_mode(void)
{
    static uint8_t simple_counter = 0;             // State machine counter for Simple Mode
    static float simple_sin_y=0, simple_cos_x=0;

    // used to manage state machine
    // which improves speed of function
    simple_counter++;

    int16_t delta = wrap_360(ahrs.yaw_sensor - initial_simple_bearing)/100;

    if (simple_counter == 1) {
        // roll
        simple_cos_x = sinf(radians(90 - delta));

    }else if (simple_counter > 2) {
        // pitch
        simple_sin_y = cosf(radians(90 - delta));
        simple_counter = 0;
    }

    // Rotate input by the initial bearing
    int16_t _roll   = g.rc_1.control_in * simple_cos_x + g.rc_2.control_in * simple_sin_y;
    int16_t _pitch  = -(g.rc_1.control_in * simple_sin_y - g.rc_2.control_in * simple_cos_x);

    g.rc_1.control_in = _roll;
    g.rc_2.control_in = _pitch;
}

// update_super_simple_beading - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void update_super_simple_beading()
{
    // are we in SIMPLE mode?
    if(ap.simple_mode && g.super_simple) {
        // get distance to home
        if(home_distance > SUPER_SIMPLE_RADIUS) {        // 10m from home
            // we reset the angular offset to be a vector from home to the quad
            initial_simple_bearing = wrap_360(home_bearing+18000);
        }
    }
}

// set_throttle_mode - sets the throttle mode and initialises any variables as required
bool set_throttle_mode( uint8_t new_throttle_mode )
{
    // boolean to ensure proper initialisation of throttle modes
    bool throttle_initialised = false;

    // return immediately if no change
    if( new_throttle_mode == throttle_mode ) {
        return true;
    }

    // initialise any variables required for the new throttle mode
    switch(new_throttle_mode) {
        case THROTTLE_MANUAL:
        case THROTTLE_MANUAL_TILT_COMPENSATED:
            throttle_accel_deactivate();                // this controller does not use accel based throttle controller
            altitude_error = 0;                         // clear altitude error reported to GCS
            throttle_initialised = true;
            break;

        case THROTTLE_ACCELERATION:                     // pilot inputs the desired acceleration
            if( g.throttle_accel_enabled ) {            // this throttle mode requires use of the accel based throttle controller
                altitude_error = 0;                     // clear altitude error reported to GCS
                throttle_initialised = true;
            }
            break;

        case THROTTLE_RATE:
            altitude_error = 0;                         // clear altitude error reported to GCS
            throttle_initialised = true;
            break;

        case THROTTLE_STABILIZED_RATE:
        case THROTTLE_DIRECT_ALT:
            controller_desired_alt = current_loc.alt;   // reset controller desired altitude to current altitude
            throttle_initialised = true;
            break;

        case THROTTLE_HOLD:
        case THROTTLE_AUTO:
            controller_desired_alt = current_loc.alt;   // reset controller desired altitude to current altitude
            set_new_altitude(current_loc.alt);          // by default hold the current altitude
            if ( throttle_mode <= THROTTLE_MANUAL_TILT_COMPENSATED ) {      // reset the alt hold I terms if previous throttle mode was manual
                reset_throttle_I();
                set_accel_throttle_I_from_pilot_throttle(get_pilot_desired_throttle(g.rc_3.control_in));
            }
            throttle_initialised = true;
            break;

        case THROTTLE_LAND:
            set_land_complete(false);   // mark landing as incomplete
            land_detector = 0;          // A counter that goes up if our climb rate stalls out.
            controller_desired_alt = current_loc.alt;   // reset controller desired altitude to current altitude
            // Set target altitude to LAND_START_ALT if we are high, below this altitude the get_throttle_rate_stabilized will take care of setting the next_WP.alt
            if (current_loc.alt >= LAND_START_ALT) {
                set_new_altitude(LAND_START_ALT);
            }
            throttle_initialised = true;
            break;

        default:
            // To-Do: log an error message to the dataflash or tlogs instead of printing to the serial port
            cliSerial->printf_P(PSTR("Unsupported throttle mode: %d!!"),new_throttle_mode);
            break;
    }

    // update the throttle mode
    if( throttle_initialised ) {
        throttle_mode = new_throttle_mode;

        // reset some variables used for logging
        desired_climb_rate = 0;
        nav_throttle = 0;
    }

    // return success or failure
    return throttle_initialised;
}

// update_throttle_mode - run high level throttle controllers
// 50 hz update rate
void update_throttle_mode(void)
{
    int16_t pilot_climb_rate;
    int16_t pilot_throttle_scaled;

    if(ap.do_flip)     // this is pretty bad but needed to flip in AP modes.
        return;

    // do not run throttle controllers if motors disarmed
    if( !motors.armed() ) {
        set_throttle_out(0, false);
        throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
        return;
    }

#if FRAME_CONFIG == HELI_FRAME
	if (control_mode == STABILIZE){
		motors.stab_throttle = true;
	} else {
		motors.stab_throttle = false;
	}
#endif // HELI_FRAME

    switch(throttle_mode) {

    case THROTTLE_MANUAL:
        // completely manual throttle
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
        }else{
            // send pilot's output directly to motors
            pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
            set_throttle_out(pilot_throttle_scaled, false);

            // update estimate of throttle cruise
			#if FRAME_CONFIG == HELI_FRAME
            update_throttle_cruise(motors.coll_out);
			#else
			update_throttle_cruise(pilot_throttle_scaled);
			#endif  //HELI_FRAME


            // check if we've taken off yet
            if (!ap.takeoff_complete && motors.armed()) {
                if (pilot_throttle_scaled > g.throttle_cruise) {
                    // we must be in the air by now
                    set_takeoff_complete(true);
                }
            }
        }
        break;

    case THROTTLE_MANUAL_TILT_COMPENSATED:
        // manual throttle but with angle boost
        if (g.rc_3.control_in <= 0) {
            set_throttle_out(0, false); // no need for angle boost with zero throttle
        }else{
            pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
            set_throttle_out(pilot_throttle_scaled, true);

            // update estimate of throttle cruise
            #if FRAME_CONFIG == HELI_FRAME
            update_throttle_cruise(motors.coll_out);
			#else
			update_throttle_cruise(pilot_throttle_scaled);
			#endif  //HELI_FRAME

            if (!ap.takeoff_complete && motors.armed()) {
                if (pilot_throttle_scaled > g.throttle_cruise) {
                    // we must be in the air by now
                    set_takeoff_complete(true);
                }
            }
        }
        break;

    case THROTTLE_ACCELERATION:
        // pilot inputs the desired acceleration
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
            throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
        }else{
            int16_t desired_acceleration = get_pilot_desired_acceleration(g.rc_3.control_in);
            set_throttle_accel_target(desired_acceleration);
        }
        break;

    case THROTTLE_RATE:
        // pilot inputs the desired climb rate.  Note this is the unstabilized rate controller
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
            throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
        }else{
            pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
            get_throttle_rate(pilot_climb_rate);
        }
        break;

    case THROTTLE_STABILIZED_RATE:
        // pilot inputs the desired climb rate.  Note this is the stabilized rate controller
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
            throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
            altitude_error = 0;             // clear altitude error reported to GCS - normally underlying alt hold controller updates altitude error reported to GCS
        }else{
            pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
            get_throttle_rate_stabilized(pilot_climb_rate);
        }
        break;

    case THROTTLE_DIRECT_ALT:
        // pilot inputs a desired altitude from 0 ~ 10 meters
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
            throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
            altitude_error = 0;             // clear altitude error reported to GCS - normally underlying alt hold controller updates altitude error reported to GCS
        }else{
            int32_t desired_alt = get_pilot_desired_direct_alt(g.rc_3.control_in);
            get_throttle_althold_with_slew(desired_alt, g.auto_velocity_z_min, g.auto_velocity_z_max);
        }
        break;

    case THROTTLE_HOLD:
        // alt hold plus pilot input of climb rate
        pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
        if( sonar_alt_health >= SONAR_ALT_HEALTH_MAX ) {
            // if sonar is ok, use surface tracking
            get_throttle_surface_tracking(pilot_climb_rate);
        }else{
            // if no sonar fall back stabilize rate controller
            get_throttle_rate_stabilized(pilot_climb_rate);
        }
        break;

    case THROTTLE_AUTO:
        // auto pilot altitude controller with target altitude held in next_WP.alt
        if(motors.auto_armed() == true) {
            get_throttle_althold_with_slew(next_WP.alt, g.auto_velocity_z_min, g.auto_velocity_z_max);
        }
        break;

    case THROTTLE_LAND:
        // landing throttle controller
        get_throttle_land();
        break;
    }
}

static void read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    ahrs.update();
    omega = ins.get_gyro();

#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.update();
#endif
}

static void update_trig(void){
    Vector2f yawvector;
    Matrix3f temp   = ahrs.get_dcm_matrix();

    yawvector.x     = temp.a.x;     // sin
    yawvector.y     = temp.b.x;         // cos
    yawvector.normalize();

    cos_pitch_x     = safe_sqrt(1 - (temp.c.x * temp.c.x));     // level = 1
    cos_roll_x      = temp.c.z / cos_pitch_x;                       // level = 1

    cos_pitch_x     = constrain(cos_pitch_x, 0, 1.0);
    // this relies on constrain() of infinity doing the right thing,
    // which it does do in avr-libc
    cos_roll_x      = constrain(cos_roll_x, -1.0, 1.0);

    sin_yaw_y       = yawvector.x;                                              // 1y = north
    cos_yaw_x       = yawvector.y;                                              // 0x = north

    // added to convert earth frame to body frame for rate controllers
    sin_pitch       = -temp.c.x;
    sin_roll        = temp.c.y / cos_pitch_x;

    sin_yaw               = constrain(temp.b.x/cos_pitch_x, -1.0, 1.0);
    cos_yaw               = constrain(temp.a.x/cos_pitch_x, -1.0, 1.0);

    //flat:
    // 0 ° = cos_yaw:  0.00, sin_yaw:  1.00,
    // 90° = cos_yaw:  1.00, sin_yaw:  0.00,
    // 180 = cos_yaw:  0.00, sin_yaw: -1.00,
    // 270 = cos_yaw: -1.00, sin_yaw:  0.00,
}

// updated at 10hz
static void update_altitude()
{
    int32_t old_baro_alt    = baro_alt;
    int16_t old_sonar_alt   = sonar_alt;

#if HIL_MODE == HIL_MODE_ATTITUDE
    // we are in the SIM, fake out the baro and Sonar
    int16_t fake_relative_alt = g_gps->altitude - gps_base_alt;
    baro_alt                = fake_relative_alt;
    baro_rate               = (baro_alt - old_baro_alt) * 5;             // 5hz
    if(g.sonar_enabled) {
        sonar_alt           = fake_relative_alt;
        sonar_rate          = baro_rate;
    }
    current_loc.alt = baro_alt;
    climb_rate_actual = baro_rate;
#else
    // read in actual baro altitude
    baro_alt            = read_barometer();

    // calc baro based vertical velocity
    int16_t temp		= (baro_alt - old_baro_alt) * 10;
    baro_rate           = (temp + baro_rate) >> 1;
    baro_rate			= constrain(baro_rate, -500, 500);

    // read in sonar altitude and calculate sonar rate
    sonar_alt           = read_sonar();
    // start calculating the sonar_rate as soon as valid sonar readings start coming in so that we are ready when the sonar_alt_health becomes 3
    // Note: post 2.9.1 release we will remove the sonar_rate variable completely
    if(sonar_alt_health > 1) {
        sonar_rate      = (sonar_alt - old_sonar_alt) * 10;
        sonar_rate      = constrain(sonar_rate, -150, 150);
    }

    // Note: with inertial nav, alt and rate are pulled from the inav lib at 50hz in update_altitude_est function
    // so none of the below is required
# if INERTIAL_NAV_Z != ENABLED
    // if no sonar set current alt to baro alt
    if(!g.sonar_enabled) {
        // NO Sonar case
        current_loc.alt = baro_alt;
        climb_rate_actual = baro_rate;
    }else{
        // Blend barometer and sonar data together
        float scale;
        if(baro_alt < 800) {
            scale = (float)(sonar_alt - 400) / 200.0;
            scale = constrain(scale, 0.0, 1.0);
            // solve for a blended altitude
            current_loc.alt = ((float)sonar_alt * (1.0 - scale)) + ((float)baro_alt * scale);

            // solve for a blended climb_rate
            climb_rate_actual = ((float)sonar_rate * (1.0 - scale)) + (float)baro_rate * scale;

        }else{
            // we must be higher than sonar (>800), don't get tricked by bad sonar reads
            current_loc.alt = baro_alt;
            // dont blend, go straight baro
            climb_rate_actual       = baro_rate;
        }
    }
    // climb_rate_error is used to spread the change in climb rate across the next 5 samples
    climb_rate_error = (climb_rate_actual - climb_rate) / 5;
# endif // INERTIAL_NAV_Z != ENABLED
#endif  // HIL_MODE == HIL_MODE_ATTITUDE

    // update the target altitude
    verify_altitude();
}

static void update_altitude_est()
{
#if INERTIAL_NAV_Z == ENABLED
    // with inertial nav we can update the altitude and climb rate at 50hz
    current_loc.alt = inertial_nav.get_altitude();
    climb_rate = inertial_nav.get_velocity_z();

    // update baro and sonar alt and climb rate just for logging purposes
    // To-Do: remove alt_sensor_flag and move update_altitude to be called from 10hz loop
    if(ap_system.alt_sensor_flag) {
        ap_system.alt_sensor_flag = false;
        update_altitude();
        if ((g.log_bitmask & MASK_LOG_CTUN) && motors.armed()) {
            Log_Write_Control_Tuning();
        }
    }
#else
    if(ap_system.alt_sensor_flag) {
        update_altitude();
        ap_system.alt_sensor_flag = false;

        if ((g.log_bitmask & MASK_LOG_CTUN) && motors.armed()) {
            Log_Write_Control_Tuning();
        }
    }else{
        // simple dithering of climb rate
        climb_rate += climb_rate_error;
        current_loc.alt += (climb_rate / 50);
    }
#endif
}

static void tuning(){
    tuning_value = (float)g.rc_6.control_in / 1000.0f;
    g.rc_6.set_range(g.radio_tuning_low,g.radio_tuning_high);                   // 0 to 1

    switch(g.radio_tuning) {

    case CH6_RATE_KD:
        g.pid_rate_roll.kD(tuning_value);
        g.pid_rate_pitch.kD(tuning_value);
        break;

    case CH6_STABILIZE_KP:
        g.pi_stabilize_roll.kP(tuning_value);
        g.pi_stabilize_pitch.kP(tuning_value);
        break;

    case CH6_STABILIZE_KI:
        g.pi_stabilize_roll.kI(tuning_value);
        g.pi_stabilize_pitch.kI(tuning_value);
        break;

    case CH6_ACRO_KP:
        g.acro_p = tuning_value;
        break;

    case CH6_RATE_KP:
        g.pid_rate_roll.kP(tuning_value);
        g.pid_rate_pitch.kP(tuning_value);
        break;

    case CH6_RATE_KI:
        g.pid_rate_roll.kI(tuning_value);
        g.pid_rate_pitch.kI(tuning_value);
        break;

    case CH6_YAW_KP:
        g.pi_stabilize_yaw.kP(tuning_value);
        break;

    case CH6_YAW_KI:
        g.pi_stabilize_yaw.kI(tuning_value);
        break;

    case CH6_YAW_RATE_KP:
        g.pid_rate_yaw.kP(tuning_value);
        break;

    case CH6_YAW_RATE_KD:
        g.pid_rate_yaw.kD(tuning_value);
        break;

    case CH6_THROTTLE_KP:
        g.pid_throttle.kP(tuning_value);
        break;

    case CH6_THROTTLE_KI:
        g.pid_throttle.kI(tuning_value);
        break;

    case CH6_THROTTLE_KD:
        g.pid_throttle.kD(tuning_value);
        break;

    case CH6_TOP_BOTTOM_RATIO:
        motors.top_bottom_ratio = tuning_value;
        break;

    case CH6_RELAY:
        if (g.rc_6.control_in > 525) relay.on();
        if (g.rc_6.control_in < 475) relay.off();
        break;

    case CH6_TRAVERSE_SPEED:
        g.waypoint_speed_max = g.rc_6.control_in;
        break;

    case CH6_LOITER_KP:
        g.pi_loiter_lat.kP(tuning_value);
        g.pi_loiter_lon.kP(tuning_value);
        break;

    case CH6_LOITER_KI:
        g.pi_loiter_lat.kI(tuning_value);
        g.pi_loiter_lon.kI(tuning_value);
        break;

    case CH6_NAV_KP:
        g.pid_nav_lat.kP(tuning_value);
        g.pid_nav_lon.kP(tuning_value);
        break;

    case CH6_LOITER_RATE_KP:
        g.pid_loiter_rate_lon.kP(tuning_value);
        g.pid_loiter_rate_lat.kP(tuning_value);
        break;

    case CH6_LOITER_RATE_KI:
        g.pid_loiter_rate_lon.kI(tuning_value);
        g.pid_loiter_rate_lat.kI(tuning_value);
        break;

    case CH6_LOITER_RATE_KD:
        g.pid_loiter_rate_lon.kD(tuning_value);
        g.pid_loiter_rate_lat.kD(tuning_value);
        break;

    case CH6_NAV_KI:
        g.pid_nav_lat.kI(tuning_value);
        g.pid_nav_lon.kI(tuning_value);
        break;

#if FRAME_CONFIG == HELI_FRAME
    case CH6_HELI_EXTERNAL_GYRO:
        motors.ext_gyro_gain = tuning_value;
        break;
#endif

    case CH6_THR_HOLD_KP:
        g.pi_alt_hold.kP(tuning_value);
        break;

    case CH6_OPTFLOW_KP:
        g.pid_optflow_roll.kP(tuning_value);
        g.pid_optflow_pitch.kP(tuning_value);
        break;

    case CH6_OPTFLOW_KI:
        g.pid_optflow_roll.kI(tuning_value);
        g.pid_optflow_pitch.kI(tuning_value);
        break;

    case CH6_OPTFLOW_KD:
        g.pid_optflow_roll.kD(tuning_value);
        g.pid_optflow_pitch.kD(tuning_value);
        break;

#if HIL_MODE != HIL_MODE_ATTITUDE                                       // do not allow modifying _kp or _kp_yaw gains in HIL mode
    case CH6_AHRS_YAW_KP:
        ahrs._kp_yaw.set(tuning_value);
        break;

    case CH6_AHRS_KP:
        ahrs._kp.set(tuning_value);
        break;
#endif

    case CH6_INAV_TC:
#if INERTIAL_NAV_XY == ENABLED
        inertial_nav.set_time_constant_xy(tuning_value);
#endif
#if INERTIAL_NAV_Z == ENABLED
        inertial_nav.set_time_constant_z(tuning_value);
#endif
        break;

    case CH6_THR_ACCEL_KP:
        g.pid_throttle_accel.kP(tuning_value);
        break;

    case CH6_THR_ACCEL_KI:
        g.pid_throttle_accel.kI(tuning_value);
        break;

    case CH6_THR_ACCEL_KD:
        g.pid_throttle_accel.kD(tuning_value);
        break;
    }
}

AP_HAL_MAIN();

#line 1 "./Firmware/ACopter32/AP_State.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


void set_home_is_set(bool b)
{
    // if no change, exit immediately
    if( ap.home_is_set == b )
        return;

    ap.home_is_set 	= b;
    if(b) {
        Log_Write_Event(DATA_SET_HOME);
    }
}

// ---------------------------------------------
void set_armed(bool b)
{
    // if no change, exit immediately
    if( ap.armed == b )
        return;

    ap.armed = b;
    if(b){
        Log_Write_Event(DATA_ARMED);
    }else{
        Log_Write_Event(DATA_DISARMED);
    }
}

// ---------------------------------------------
void set_auto_armed(bool b)
{
    // if no change, exit immediately
    if( ap.auto_armed == b )
        return;

    ap.auto_armed = b;
    if(b){
        Log_Write_Event(DATA_AUTO_ARMED);
    }
}

// ---------------------------------------------
void set_simple_mode(bool b)
{
    if(ap.simple_mode != b){
        if(b){
            Log_Write_Event(DATA_SET_SIMPLE_ON);
        }else{
            Log_Write_Event(DATA_SET_SIMPLE_OFF);
        }
        ap.simple_mode = b;
    }
}

// ---------------------------------------------
static void set_failsafe(bool mode)
{
    // only act on changes
    // -------------------
    if(ap.failsafe != mode) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        ap.failsafe = mode;

        if (ap.failsafe == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_off_event();
        }else{
            // We've lost radio contact
            // ------------------------
            failsafe_on_event();
        }
    }
}


// ---------------------------------------------
void set_low_battery(bool b)
{
    ap.low_battery = b;
}

// ---------------------------------------------
void set_takeoff_complete(bool b)
{
    // if no change, exit immediately
    if( ap.takeoff_complete == b )
        return;

    if(b){
        Log_Write_Event(DATA_TAKEOFF);
    }
    ap.takeoff_complete = b;
}

// ---------------------------------------------
void set_land_complete(bool b)
{
    // if no change, exit immediately
    if( ap.land_complete == b )
        return;

    if(b){
        Log_Write_Event(DATA_LAND_COMPLETE);
    }
    ap.land_complete = b;
}

// ---------------------------------------------

void set_alt_change(uint8_t flag){

    // if no change, exit immediately
    if( alt_change_flag == flag ) {
        return;
    }

    // update flag
    alt_change_flag = flag;

    if(flag == REACHED_ALT){
        Log_Write_Event(DATA_REACHED_ALT);

    }else if(flag == ASCENDING){
        Log_Write_Event(DATA_ASCENDING);

    }else if(flag == DESCENDING){
        Log_Write_Event(DATA_DESCENDING);
    }
}

void set_compass_healthy(bool b)
{
    if(ap.compass_status != b){
        if(false == b){
            Log_Write_Event(DATA_LOST_COMPASS);
        }
    }
    ap.compass_status = b;
}

void set_gps_healthy(bool b)
{
    if(ap.gps_status != b){
        if(false == b){
            Log_Write_Event(DATA_LOST_GPS);
        }
    }
    ap.gps_status = b;
}

void dump_state()
{
    cliSerial->printf("st: %u\n",ap.value);
}
#line 1 "./Firmware/ACopter32/Attitude.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void
get_stabilize_roll(int32_t target_angle)
{
    // angle error
    target_angle            = wrap_180(target_angle - ahrs.roll_sensor);

    // limit the error we're feeding to the PID
    target_angle            = constrain(target_angle, -4500, 4500);

        // convert to desired Rate:
    int32_t target_rate = g.pi_stabilize_roll.get_p(target_angle);

    int16_t i_stab;
    if(labs(ahrs.roll_sensor) < 500) {
        target_angle            = constrain(target_angle, -500, 500);
        i_stab                          = g.pi_stabilize_roll.get_i(target_angle, G_Dt);
    }else{
        i_stab                          = g.pi_stabilize_roll.get_integrator();
    }

    // set targets for rate controller
    set_roll_rate_target(target_rate+i_stab, EARTH_FRAME);
}

static void
get_stabilize_pitch(int32_t target_angle)
{
    // angle error
    target_angle            = wrap_180(target_angle - ahrs.pitch_sensor);

    // limit the error we're feeding to the PID
    target_angle            = constrain(target_angle, -4500, 4500);

    // convert to desired Rate:
    int32_t target_rate = g.pi_stabilize_pitch.get_p(target_angle);

    int16_t i_stab;
    if(labs(ahrs.pitch_sensor) < 500) {
        target_angle            = constrain(target_angle, -500, 500);
        i_stab                          = g.pi_stabilize_pitch.get_i(target_angle, G_Dt);
    }else{
        i_stab                          = g.pi_stabilize_pitch.get_integrator();
    }

    // set targets for rate controller
    set_pitch_rate_target(target_rate + i_stab, EARTH_FRAME);
}

static void
get_stabilize_yaw(int32_t target_angle)
{
    int32_t target_rate,i_term;
    int32_t angle_error;
    int32_t output = 0;

    // angle error
    angle_error             = wrap_180(target_angle - ahrs.yaw_sensor);

    // limit the error we're feeding to the PID

    angle_error             = constrain(angle_error, -4500, 4500);

    // convert angle error to desired Rate:
    target_rate = g.pi_stabilize_yaw.get_p(angle_error);
    i_term = g.pi_stabilize_yaw.get_i(angle_error, G_Dt);

    // do not use rate controllers for helicotpers with external gyros
#if FRAME_CONFIG == HELI_FRAME
    if(motors.ext_gyro_enabled) {
        g.rc_4.servo_out = constrain((target_rate + i_term), -4500, 4500);
    }
#endif

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && g.radio_tuning == CH6_YAW_KP ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_YAW_KP, angle_error, target_rate, i_term, 0, output, tuning_value);
        }
    }
#endif

    // set targets for rate controller
    set_yaw_rate_target(target_rate+i_term, EARTH_FRAME);
}

static void
get_acro_roll(int32_t target_rate)
{
    target_rate = target_rate * g.acro_p;

    // set targets for rate controller
    set_roll_rate_target(target_rate, BODY_FRAME);
}

static void
get_acro_pitch(int32_t target_rate)
{
    target_rate = target_rate * g.acro_p;

    // set targets for rate controller
    set_pitch_rate_target(target_rate, BODY_FRAME);
}

static void
get_acro_yaw(int32_t target_rate)
{
    target_rate = target_rate * g.acro_p;

    // set targets for rate controller
    set_yaw_rate_target(target_rate, BODY_FRAME);
}

// Roll with rate input and stabilized in the earth frame
static void
get_roll_rate_stabilized_ef(int32_t stick_angle)
{
    int32_t angle_error = 0;

    // convert the input to the desired roll rate
    int32_t target_rate = stick_angle * g.acro_p - (roll_axis * g.acro_balance_roll)/100;

    // convert the input to the desired roll rate
    roll_axis += target_rate * G_Dt;
    roll_axis = wrap_180(roll_axis);

    // ensure that we don't reach gimbal lock
    if (labs(roll_axis > 4500) && g.acro_trainer_enabled) {
        roll_axis	= constrain(roll_axis, -4500, 4500);
        angle_error = wrap_180(roll_axis - ahrs.roll_sensor);
    } else {
        // angle error with maximum of +- max_angle_overshoot
        angle_error = wrap_180(roll_axis - ahrs.roll_sensor);
        angle_error	= constrain(angle_error, -MAX_ROLL_OVERSHOOT, MAX_ROLL_OVERSHOOT);
    }

    if (motors.armed() == false || ((g.rc_3.control_in == 0) && !ap.failsafe)) {
        angle_error = 0;
    }

    // update roll_axis to be within max_angle_overshoot of our current heading
    roll_axis = wrap_180(angle_error + ahrs.roll_sensor);

    // set earth frame targets for rate controller

    // set earth frame targets for rate controller
	set_roll_rate_target(g.pi_stabilize_roll.get_p(angle_error) + target_rate, EARTH_FRAME);
}

// Pitch with rate input and stabilized in the earth frame
static void
get_pitch_rate_stabilized_ef(int32_t stick_angle)
{
    int32_t angle_error = 0;

    // convert the input to the desired pitch rate
    int32_t target_rate = stick_angle * g.acro_p - (pitch_axis * g.acro_balance_pitch)/100;

    // convert the input to the desired pitch rate
    pitch_axis += target_rate * G_Dt;
    pitch_axis = wrap_180(pitch_axis);

    // ensure that we don't reach gimbal lock
    if (labs(pitch_axis) > 4500) {
        pitch_axis	= constrain(pitch_axis, -4500, 4500);
        angle_error = wrap_180(pitch_axis - ahrs.pitch_sensor);
    } else {
        // angle error with maximum of +- max_angle_overshoot
        angle_error = wrap_180(pitch_axis - ahrs.pitch_sensor);
        angle_error	= constrain(angle_error, -MAX_PITCH_OVERSHOOT, MAX_PITCH_OVERSHOOT);
    }

    if (motors.armed() == false || ((g.rc_3.control_in == 0) && !ap.failsafe)) {
        angle_error = 0;
    }

    // update pitch_axis to be within max_angle_overshoot of our current heading
    pitch_axis = wrap_180(angle_error + ahrs.pitch_sensor);

    // set earth frame targets for rate controller
	set_pitch_rate_target(g.pi_stabilize_pitch.get_p(angle_error) + target_rate, EARTH_FRAME);
}

// Yaw with rate input and stabilized in the earth frame
static void
get_yaw_rate_stabilized_ef(int32_t stick_angle)
{

    int32_t angle_error = 0;

    // convert the input to the desired yaw rate
    int32_t target_rate = stick_angle * g.acro_p;

    // convert the input to the desired yaw rate
    nav_yaw += target_rate * G_Dt;
    nav_yaw = wrap_360(nav_yaw);

    // calculate difference between desired heading and current heading
    angle_error = wrap_180(nav_yaw - ahrs.yaw_sensor);

    // limit the maximum overshoot
    angle_error	= constrain(angle_error, -MAX_YAW_OVERSHOOT, MAX_YAW_OVERSHOOT);

    if (motors.armed() == false || ((g.rc_3.control_in == 0) && !ap.failsafe)) {
    	angle_error = 0;
    }

    // update nav_yaw to be within max_angle_overshoot of our current heading
    nav_yaw = wrap_360(angle_error + ahrs.yaw_sensor);

    // set earth frame targets for rate controller
	set_yaw_rate_target(g.pi_stabilize_yaw.get_p(angle_error)+target_rate, EARTH_FRAME);
}

// set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        roll_rate_target_bf = desired_rate;
    }else{
        roll_rate_target_ef = desired_rate;
    }
}

// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        pitch_rate_target_bf = desired_rate;
    }else{
        pitch_rate_target_ef = desired_rate;
    }
}

// set_yaw_rate_target - to be called by upper controllers to set yaw rate targets in the earth frame
void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        yaw_rate_target_bf = desired_rate;
    }else{
        yaw_rate_target_ef = desired_rate;
    }
}

// update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
void
update_rate_contoller_targets()
{
    if( rate_targets_frame == EARTH_FRAME ) {
        // convert earth frame rates to body frame rates
        roll_rate_target_bf 	= roll_rate_target_ef - sin_pitch * yaw_rate_target_ef;
        pitch_rate_target_bf 	= cos_roll_x  * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef;
        yaw_rate_target_bf 		= cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef;
    }
}

// run roll, pitch and yaw rate controllers and send output to motors
// targets for these controllers comes from stabilize controllers
void
run_rate_controllers()
{
#if FRAME_CONFIG == HELI_FRAME          // helicopters only use rate controllers for yaw and only when not using an external gyro
    if(!motors.ext_gyro_enabled) {
		g.rc_1.servo_out = get_heli_rate_roll(roll_rate_target_bf);
		g.rc_2.servo_out = get_heli_rate_pitch(pitch_rate_target_bf);
        g.rc_4.servo_out = get_heli_rate_yaw(yaw_rate_target_bf);
    }
#else
    // call rate controllers
    g.rc_1.servo_out = get_rate_roll(roll_rate_target_bf);
    g.rc_2.servo_out = get_rate_pitch(pitch_rate_target_bf);
    g.rc_4.servo_out = get_rate_yaw(yaw_rate_target_bf);
#endif

    // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
    if( g.throttle_accel_enabled && throttle_accel_controller_active ) {
        set_throttle_out(get_throttle_accel(throttle_accel_target_ef), true);
    }
}

#if FRAME_CONFIG == HELI_FRAME
// init_rate_controllers - set-up filters for rate controller inputs
void init_rate_controllers()
{
   // initalise low pass filters on rate controller inputs
   // 1st parameter is time_step, 2nd parameter is time_constant
   rate_roll_filter.set_cutoff_frequency(0.01f, 2.0f);
   rate_pitch_filter.set_cutoff_frequency(0.01f, 2.0f);
   // rate_yaw_filter.set_cutoff_frequency(0.01f, 2.0f);
   // other option for initialisation is rate_roll_filter.set_cutoff_frequency(<time_step>,<cutoff_freq>);
}

static int16_t
get_heli_rate_roll(int32_t target_rate)
{
    int32_t p,i,d,ff;                // used to capture pid values for logging
	int32_t current_rate;			// this iteration's rate
    int32_t rate_error;             // simply target_rate - current_rate
    int32_t output;                 // output from pid controller

	// get current rate
	current_rate    = (omega.x * DEGX100);

    // filter input
    current_rate = rate_roll_filter.apply(current_rate);
	
    // call pid controller
    rate_error  = target_rate - current_rate;
    p   = g.pid_rate_roll.get_p(rate_error);

    if (motors.flybar_mode == 1) {												// Mechanical Flybars get regular integral for rate auto trim
		if (target_rate > -50 && target_rate < 50){								// Frozen at high rates
			i		= g.pid_rate_roll.get_i(rate_error, G_Dt);
		} else {
			i		= g.pid_rate_roll.get_integrator();
		}
	} else {
		i			= g.pid_rate_roll.get_leaky_i(rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);		// Flybarless Helis get huge I-terms. I-term controls much of the rate
	}
	
	d = g.pid_rate_roll.get_d(rate_error, G_Dt);
	
	ff = g.heli_roll_ff * target_rate;

    output = p + i + d + ff;

    // constrain output
    output = constrain(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_heli_rate_pitch(int32_t target_rate)
{
    int32_t p,i,d,ff;                                                                   // used to capture pid values for logging
	int32_t current_rate;                                                     			// this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t output;                                                                     // output from pid controller

	// get current rate
	current_rate    = (omega.y * DEGX100);
	
	// filter input
	current_rate = rate_pitch_filter.apply(current_rate);
	
	// call pid controller
    rate_error      = target_rate - current_rate;
    p               = g.pid_rate_pitch.get_p(rate_error);						// Helicopters get huge feed-forward
	
	if (motors.flybar_mode == 1) {												// Mechanical Flybars get regular integral for rate auto trim
		if (target_rate > -50 && target_rate < 50){								// Frozen at high rates
			i		= g.pid_rate_pitch.get_i(rate_error, G_Dt);
		} else {
			i		= g.pid_rate_pitch.get_integrator();
		}
	} else {
		i			= g.pid_rate_pitch.get_leaky_i(rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);	// Flybarless Helis get huge I-terms. I-term controls much of the rate
	}
	
	d = g.pid_rate_pitch.get_d(rate_error, G_Dt);
	
	ff = g.heli_pitch_ff*target_rate;
    
    output = p + i + d + ff;

    // constrain output
    output = constrain(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        if( pid_log_counter == 0 ) {               // relies on get_heli_rate_roll to update pid_log_counter
            Log_Write_PID(CH6_RATE_KP+100, rate_error, p, i, 0, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_heli_rate_yaw(int32_t target_rate)
{
    int32_t p,i,d,ff;                                                                   // used to capture pid values for logging
	int32_t current_rate;                                                               // this iteration's rate
    int32_t rate_error;
    int32_t output;

	// get current rate
    current_rate    = (omega.z * DEGX100);
	
	// filter input
	// current_rate = rate_yaw_filter.apply(current_rate);
	
    // rate control
    rate_error              = target_rate - current_rate;

    // separately calculate p, i, d values for logging
    p = g.pid_rate_yaw.get_p(rate_error);
    
    i = g.pid_rate_yaw.get_i(rate_error, G_Dt);

    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);
	
	ff = g.heli_yaw_ff*target_rate;

    output  = p + i + d + ff;
    output = constrain(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_YAW_RATE_KP || g.radio_tuning == CH6_YAW_RATE_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_YAW_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
	
#endif

	// output control
	return output;
}
#endif // HELI_FRAME

#if FRAME_CONFIG != HELI_FRAME
static int16_t
get_rate_roll(int32_t target_rate)
{
    int32_t p,i,d;                  // used to capture pid values for logging
    int32_t current_rate;           // this iteration's rate
    int32_t rate_error;             // simply target_rate - current_rate
    int32_t output;                 // output from pid controller

    // get current rate
    current_rate    = (omega.x * DEGX100);

    // call pid controller
    rate_error  = target_rate - current_rate;
    p           = g.pid_rate_roll.get_p(rate_error);

    // freeze I term if we've breached roll-pitch limits
    if( motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) ) {
        i	= g.pid_rate_roll.get_integrator();
    }else{
        i   = g.pid_rate_roll.get_i(rate_error, G_Dt);
    }

    d = g.pid_rate_roll.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain(output, -5000, 5000);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        pid_log_counter++;                          // Note: get_rate_pitch pid logging relies on this function to update pid_log_counter so if you change the line above you must change the equivalent line in get_rate_pitch
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_rate_pitch(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t current_rate;                                                       // this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t output;                                                                     // output from pid controller

    // get current rate
    current_rate    = (omega.y * DEGX100);

    // call pid controller
    rate_error      = target_rate - current_rate;
    p               = g.pid_rate_pitch.get_p(rate_error);
    // freeze I term if we've breached roll-pitch limits
    if( motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) ) {
        i = g.pid_rate_pitch.get_integrator();
    }else{
        i = g.pid_rate_pitch.get_i(rate_error, G_Dt);
    }
    d = g.pid_rate_pitch.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain(output, -5000, 5000);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        if( pid_log_counter == 0 ) {               // relies on get_rate_roll having updated pid_log_counter
            Log_Write_PID(CH6_RATE_KP+100, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_rate_yaw(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t rate_error;
    int32_t output;

    // rate control
    rate_error              = target_rate - (omega.z * DEGX100);

    // separately calculate p, i, d values for logging
    p = g.pid_rate_yaw.get_p(rate_error);
    // freeze I term if we've breached yaw limits
    if( motors.reached_limit(AP_MOTOR_YAW_LIMIT) ) {
        i = g.pid_rate_yaw.get_integrator();
    }else{
        i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
    }
    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);

    output  = p+i+d;
    output = constrain(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && g.radio_tuning == CH6_YAW_RATE_KP ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_YAW_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

#if FRAME_CONFIG == TRI_FRAME
    // constrain output
    return output;
#else // !TRI_FRAME
    // output control:
    int16_t yaw_limit = 2200 + abs(g.rc_4.control_in);

    // smoother Yaw control:
    return constrain(output, -yaw_limit, yaw_limit);
#endif // TRI_FRAME
}
#endif // !HELI_FRAME

// calculate modified roll/pitch depending upon optical flow calculated position
static int32_t
get_of_roll(int32_t input_roll)
{
#if OPTFLOW == ENABLED
    static float tot_x_cm = 0;      // total distance from target
    static uint32_t last_of_roll_update = 0;
    int32_t new_roll = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( optflow.last_update != last_of_roll_update) {
        last_of_roll_update = optflow.last_update;

        // add new distance moved
        tot_x_cm += optflow.x_cm;

        // only stop roll if caller isn't modifying roll
        if( input_roll == 0 && current_loc.alt < 1500) {
            p = g.pid_optflow_roll.get_p(-tot_x_cm);
            i = g.pid_optflow_roll.get_i(-tot_x_cm,1.0f);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_roll.get_d(-tot_x_cm,1.0f);
            new_roll = p+i+d;
        }else{
            g.pid_optflow_roll.reset_I();
            tot_x_cm = 0;
            p = 0;              // for logging
            i = 0;
            d = 0;
        }
        // limit amount of change and maximum angle
        of_roll = constrain(new_roll, (of_roll-20), (of_roll+20));

 #if LOGGING_ENABLED == ENABLED
        // log output if PID logging is on and we are tuning the rate P, I or D gains
        if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_OPTFLOW_KP || g.radio_tuning == CH6_OPTFLOW_KI || g.radio_tuning == CH6_OPTFLOW_KD) ) {
            pid_log_counter++;              // Note: get_of_pitch pid logging relies on this function updating pid_log_counter so if you change the line above you must change the equivalent line in get_of_pitch
            if( pid_log_counter >= 5 ) {    // (update rate / desired output rate) = (100hz / 10hz) = 10
                pid_log_counter = 0;
                Log_Write_PID(CH6_OPTFLOW_KP, tot_x_cm, p, i, d, of_roll, tuning_value);
            }
        }
 #endif // LOGGING_ENABLED == ENABLED
    }

    // limit max angle
    of_roll = constrain(of_roll, -1000, 1000);

    return input_roll+of_roll;
#else
    return input_roll;
#endif
}

static int32_t
get_of_pitch(int32_t input_pitch)
{
#if OPTFLOW == ENABLED
    static float tot_y_cm = 0;  // total distance from target
    static uint32_t last_of_pitch_update = 0;
    int32_t new_pitch = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( optflow.last_update != last_of_pitch_update ) {
        last_of_pitch_update = optflow.last_update;

        // add new distance moved
        tot_y_cm += optflow.y_cm;

        // only stop roll if caller isn't modifying pitch
        if( input_pitch == 0 && current_loc.alt < 1500 ) {
            p = g.pid_optflow_pitch.get_p(tot_y_cm);
            i = g.pid_optflow_pitch.get_i(tot_y_cm, 1.0f);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_pitch.get_d(tot_y_cm, 1.0f);
            new_pitch = p + i + d;
        }else{
            tot_y_cm = 0;
            g.pid_optflow_pitch.reset_I();
            p = 0;              // for logging
            i = 0;
            d = 0;
        }

        // limit amount of change
        of_pitch = constrain(new_pitch, (of_pitch-20), (of_pitch+20));

 #if LOGGING_ENABLED == ENABLED
        // log output if PID logging is on and we are tuning the rate P, I or D gains
        if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_OPTFLOW_KP || g.radio_tuning == CH6_OPTFLOW_KI || g.radio_tuning == CH6_OPTFLOW_KD) ) {
            if( pid_log_counter == 0 ) {        // relies on get_of_roll having updated the pid_log_counter
                Log_Write_PID(CH6_OPTFLOW_KP+100, tot_y_cm, p, i, d, of_pitch, tuning_value);
            }
        }
 #endif // LOGGING_ENABLED == ENABLED
    }

    // limit max angle
    of_pitch = constrain(of_pitch, -1000, 1000);

    return input_pitch+of_pitch;
#else
    return input_pitch;
#endif
}

/*************************************************************
 * yaw controllers
 *************************************************************/

static void get_look_ahead_yaw(int16_t pilot_yaw)
{
    // Commanded Yaw to automatically look ahead.
    if (g_gps->fix && g_gps->ground_course > YAW_LOOK_AHEAD_MIN_SPEED) {
        nav_yaw = get_yaw_slew(nav_yaw, g_gps->ground_course, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(wrap_360(nav_yaw + pilot_yaw));   // Allow pilot to "skid" around corners up to 45 degrees
    }else{
        nav_yaw += pilot_yaw * g.acro_p * G_Dt;
        nav_yaw = wrap_360(nav_yaw);
        get_stabilize_yaw(nav_yaw);
    }
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update_throttle_cruise - update throttle cruise if necessary
static void update_throttle_cruise(int16_t throttle)
{
    // ensure throttle_avg has been initialised
    if( throttle_avg == 0 ) {
        throttle_avg = g.throttle_cruise;
    }
    // calc average throttle if we are in a level hover
    if (throttle > g.throttle_min && abs(climb_rate) < 60 && labs(ahrs.roll_sensor) < 500 && labs(ahrs.pitch_sensor) < 500) {
        throttle_avg = throttle_avg * 0.99f + (float)throttle * 0.01f;
        g.throttle_cruise = throttle_avg;
    }
}

#if FRAME_CONFIG == HELI_FRAME
// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
// for traditional helicopters
static int16_t get_angle_boost(int16_t throttle)
{
    float angle_boost_factor = cos_pitch_x * cos_roll_x;
    angle_boost_factor = 1.0f - constrain(angle_boost_factor, .5f, 1.0f);
    int16_t throttle_above_mid = max(throttle - motors.throttle_mid,0);

    // to allow logging of angle boost
    angle_boost = throttle_above_mid*angle_boost_factor;

    return throttle + angle_boost;
}
#else   // all multicopters
// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
static int16_t get_angle_boost(int16_t throttle)
{
    float temp = cos_pitch_x * cos_roll_x;
    int16_t throttle_out;

    temp = constrain(temp, 0.5f, 1.0f);
    temp = constrain(9000-max(labs(roll_axis),labs(pitch_axis)), 0, 3000) / (3000 * temp);
    throttle_out = constrain((float)(throttle-g.throttle_min) * temp + g.throttle_min, g.throttle_min, 1000);

    // to allow logging of angle boost
    angle_boost = throttle_out - throttle;

    return throttle_out;
}
#endif // FRAME_CONFIG == HELI_FRAME

 // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
 // provide 0 to cut motors
void set_throttle_out( int16_t throttle_out, bool apply_angle_boost )
{
    if( apply_angle_boost ) {
        g.rc_3.servo_out = get_angle_boost(throttle_out);
    }else{
        g.rc_3.servo_out = throttle_out;
        // clear angle_boost for logging purposes
        angle_boost = 0;
    }
}

// set_throttle_accel_target - to be called by upper throttle controllers to set desired vertical acceleration in earth frame
void set_throttle_accel_target( int16_t desired_acceleration )
{
    if( g.throttle_accel_enabled ) {
        throttle_accel_target_ef = desired_acceleration;
        throttle_accel_controller_active = true;
    }else{
        // To-Do log dataflash or tlog error
        cliSerial->print_P(PSTR("Err: target sent to inactive acc thr controller!\n"));
    }
}

// disable_throttle_accel - disables the accel based throttle controller
// it will be re-enasbled on the next set_throttle_accel_target
// required when we wish to set motors to zero when pilot inputs zero throttle
void throttle_accel_deactivate()
{
    throttle_accel_controller_active = false;
}

// get_throttle_accel - accelerometer based throttle controller
// returns an actual throttle output (0 ~ 1000) to be sent to the motors
static int16_t
get_throttle_accel(int16_t z_target_accel)
{
    static float z_accel_error = 0;     // The acceleration error in cm.
    static uint32_t last_call_ms = 0;   // the last time this controller was called
    int32_t p,i,d;                      // used to capture pid values for logging
    int16_t output;
    float z_accel_meas;
    uint32_t now = millis();

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(ahrs.get_accel_ef().z + GRAVITY_MSS) * 100;

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_accel_error = 0;
    } else {
        // calculate accel error and Filter with fc = 2 Hz
        z_accel_error = z_accel_error + 0.11164f * (constrain(z_target_accel - z_accel_meas, -32000, 32000) - z_accel_error);
    }
    last_call_ms = now;

    // separately calculate p, i, d values for logging
    p = g.pid_throttle_accel.get_p(z_accel_error);
    // freeze I term if we've breached throttle limits
    if( motors.reached_limit(AP_MOTOR_THROTTLE_LIMIT) ) {
        i = g.pid_throttle_accel.get_integrator();
    }else{
        i = g.pid_throttle_accel.get_i(z_accel_error, .01f);
    }
    d = g.pid_throttle_accel.get_d(z_accel_error, .01f);

    //
    // limit the rate
    output =  constrain(p+i+d+g.throttle_cruise, g.throttle_min, g.throttle_max);

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_THR_ACCEL_KP || g.radio_tuning == CH6_THR_ACCEL_KI || g.radio_tuning == CH6_THR_ACCEL_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (50hz / 10hz) = 5hz
            pid_log_counter = 0;
            Log_Write_PID(CH6_THR_ACCEL_KP, z_accel_error, p, i, d, output, tuning_value);
        }
    }
#endif

    return output;
}

// get_pilot_desired_throttle - transform pilot's throttle input to make cruise throttle mid stick
// used only for manual throttle modes
// returns throttle output 0 to 1000
#define THROTTLE_IN_MIDDLE 500          // the throttle mid point
static int16_t get_pilot_desired_throttle(int16_t throttle_control)
{
    int16_t throttle_out;

    // exit immediately in the simple cases
    if( throttle_control == 0 || g.throttle_mid == 500) {
        return throttle_control;
    }

    // ensure reasonable throttle values
    throttle_control = constrain(throttle_control,0,1000);
    g.throttle_mid = constrain(g.throttle_mid,300,700);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_MIDDLE) {
        // below the deadband
        throttle_out = g.throttle_min + ((float)(throttle_control-g.throttle_min))*((float)(g.throttle_mid - g.throttle_min))/((float)(500-g.throttle_min));
    }else if(throttle_control > THROTTLE_IN_MIDDLE) {
        // above the deadband
        throttle_out = g.throttle_mid + ((float)(throttle_control-500))*(float)(1000-g.throttle_mid)/500.0f;
    }else{
        // must be in the deadband
        throttle_out = g.throttle_mid;
    }

    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
#define THROTTLE_IN_DEADBAND 100        // the throttle input channel's deadband in PWM
#define THROTTLE_IN_DEADBAND_TOP (THROTTLE_IN_MIDDLE+THROTTLE_IN_DEADBAND)  // top of the deadband
#define THROTTLE_IN_DEADBAND_BOTTOM (THROTTLE_IN_MIDDLE-THROTTLE_IN_DEADBAND)  // bottom of the deadband
static int16_t get_pilot_desired_climb_rate(int16_t throttle_control)
{
    int16_t desired_rate = 0;

    // throttle failsafe check
    if( ap.failsafe ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain(throttle_control,0,1000);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
        // below the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else if (throttle_control > THROTTLE_IN_DEADBAND_TOP) {
        // above the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else{
        // must be in the deadband
        desired_rate = 0;
    }

    // desired climb rate for logging
    desired_climb_rate = desired_rate;

    return desired_rate;
}

// get_pilot_desired_acceleration - transform pilot's throttle input to a desired acceleration
// default upper and lower bounds are 500 cm/s/s (roughly 1/2 a G)
// returns acceleration in cm/s/s
static int16_t get_pilot_desired_acceleration(int16_t throttle_control)
{
    int32_t desired_accel = 0;

    // throttle failsafe check
    if( ap.failsafe ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain(throttle_control,0,1000);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
        // below the deadband
        desired_accel = (int32_t)ACCELERATION_MAX_Z * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else if(throttle_control > THROTTLE_IN_DEADBAND_TOP) {
        // above the deadband
        desired_accel = (int32_t)ACCELERATION_MAX_Z * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else{
        // must be in the deadband
        desired_accel = 0;
    }

    return desired_accel;
}

// get_pilot_desired_direct_alt - transform pilot's throttle input to a desired altitude
// return altitude in cm between 0 to 10m
static int32_t get_pilot_desired_direct_alt(int16_t throttle_control)
{
    int32_t desired_alt = 0;

    // throttle failsafe check
    if( ap.failsafe ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain(throttle_control,0,1000);

    desired_alt = throttle_control;

    return desired_alt;
}

// get_throttle_rate - calculates desired accel required to achieve desired z_target_speed
// sets accel based throttle controller target
static void
get_throttle_rate(int16_t z_target_speed)
{
    static uint32_t last_call_ms = 0;
    static float z_rate_error = 0;   // The velocity error in cm.
    int32_t p,i,d;      // used to capture pid values for logging
    int16_t output;     // the target acceleration if the accel based throttle is enabled, otherwise the output to be sent to the motors
    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_rate_error    = 0;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        z_rate_error    = z_rate_error + 0.20085f * ((z_target_speed - climb_rate) - z_rate_error);
    }
    last_call_ms = now;

    // separately calculate p, i, d values for logging
    p = g.pid_throttle.get_p(z_rate_error);

    // freeze I term if we've breached throttle limits
    if(motors.reached_limit(AP_MOTOR_THROTTLE_LIMIT)) {
        i = g.pid_throttle.get_integrator();
    }else{
        i = g.pid_throttle.get_i(z_rate_error, .02);
    }
    d = g.pid_throttle.get_d(z_rate_error, .02);

    // consolidate target acceleration
    output =  p+i+d;

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_THROTTLE_KP || g.radio_tuning == CH6_THROTTLE_KI || g.radio_tuning == CH6_THROTTLE_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (50hz / 10hz) = 5hz
            pid_log_counter = 0;
            Log_Write_PID(CH6_THROTTLE_KP, z_rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // send output to accelerometer based throttle controller if enabled otherwise send directly to motors
    if( g.throttle_accel_enabled ) {
        // set target for accel based throttle controller
        set_throttle_accel_target(output);
    }else{
        set_throttle_out(g.throttle_cruise+output, true);
    }

    // update throttle cruise
    // TO-DO: this may not be correct because g.rc_3.servo_out has not been updated for this iteration
    if( z_target_speed == 0 ) {
        update_throttle_cruise(g.rc_3.servo_out);
    }
}

// get_throttle_althold - hold at the desired altitude in cm
// updates accel based throttle controller targets
// Note: max_climb_rate is an optional parameter to allow reuse of this function by landing controller
static void
get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    int32_t alt_error;
    int16_t desired_rate;
    int32_t linear_distance;      // the distace we swap between linear and sqrt.

    // calculate altitude error
    alt_error    = target_alt - current_loc.alt;

    // check kP to avoid division by zero
    if( g.pi_alt_hold.kP() != 0 ) {
        linear_distance = 250/(2*g.pi_alt_hold.kP()*g.pi_alt_hold.kP());
        if( alt_error > 2*linear_distance ) {
            desired_rate = safe_sqrt(2*250*(alt_error-linear_distance));
        }else if( alt_error < -2*linear_distance ) {
            desired_rate = -safe_sqrt(2*250*(-alt_error-linear_distance));
        }else{
            desired_rate = g.pi_alt_hold.get_p(alt_error);
        }
    }else{
        desired_rate = 0;
    }

    desired_rate = constrain(desired_rate, min_climb_rate, max_climb_rate);

    // call rate based throttle controller which will update accel based throttle controller targets
    get_throttle_rate(desired_rate);

    // update altitude error reported to GCS
    altitude_error = alt_error;

    // TO-DO: enabled PID logging for this controller
}

// get_throttle_althold_with_slew - altitude controller with slew to avoid step changes in altitude target
// calls normal althold controller which updates accel based throttle controller targets
static void
get_throttle_althold_with_slew(int16_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    // limit target altitude change
    controller_desired_alt += constrain(target_alt-controller_desired_alt, min_climb_rate*0.02f, max_climb_rate*0.02f);

    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain(controller_desired_alt,current_loc.alt-750,current_loc.alt+750);

    get_throttle_althold(controller_desired_alt, min_climb_rate-250, max_climb_rate+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_rate_stabilized - rate controller with additional 'stabilizer'
// 'stabilizer' ensure desired rate is being met
// calls normal throttle rate controller which updates accel based throttle controller targets
static void
get_throttle_rate_stabilized(int16_t target_rate)
{
    controller_desired_alt += target_rate * 0.02f;

    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain(controller_desired_alt,current_loc.alt-750,current_loc.alt+750);

    set_new_altitude(controller_desired_alt);

    get_throttle_althold(controller_desired_alt, -g.pilot_velocity_z_max-250, g.pilot_velocity_z_max+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_land - high level landing logic
// sends the desired acceleration in the accel based throttle controller
// called at 50hz
static void
get_throttle_land()
{
    // if we are above 10m and the sonar does not sense anything perform regular alt hold descent
    if (current_loc.alt >= LAND_START_ALT && !(g.sonar_enabled && sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        get_throttle_althold_with_slew(LAND_START_ALT, g.auto_velocity_z_min, -abs(g.land_speed));
    }else{
        get_throttle_rate_stabilized(-abs(g.land_speed));

        // detect whether we have landed by watching for minimum throttle and now movement
        if (abs(climb_rate) < 20 && (g.rc_3.servo_out <= get_angle_boost(g.throttle_min) || g.pid_throttle_accel.get_integrator() <= -150)) {
            if( land_detector < LAND_DETECTOR_TRIGGER ) {
                land_detector++;
            }else{
                set_land_complete(true);
                if( g.rc_3.control_in == 0 || ap.failsafe ) {
                    init_disarm_motors();
                }
            }
        }else{
            // we've sensed movement up or down so decrease land_detector
            if (land_detector > 0 ) {
                land_detector--;
            }
        }
    }
}

// get_throttle_surface_tracking - hold copter at the desired distance above the ground
// updates accel based throttle controller targets
static void
get_throttle_surface_tracking(int16_t target_rate)
{
    static float target_sonar_alt = 0;   // The desired altitude in cm above the ground
    static uint32_t last_call_ms = 0;
    float distance_error;
    float sonar_induced_slew_rate;

    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 200 ) {
        target_sonar_alt = sonar_alt + controller_desired_alt - current_loc.alt;
    }
    last_call_ms = now;

    target_sonar_alt += target_rate * 0.02f;

    distance_error = (target_sonar_alt-sonar_alt);
    sonar_induced_slew_rate = constrain(fabs(THR_SURFACE_TRACKING_P * distance_error),0,THR_SURFACE_TRACKING_VELZ_MAX);

    // do not let target altitude get too far from current altitude above ground
    // Note: the 750cm limit is perhaps too wide but is consistent with the regular althold limits and helps ensure a smooth transition
    target_sonar_alt = constrain(target_sonar_alt,sonar_alt-750,sonar_alt+750);
    controller_desired_alt = current_loc.alt+(target_sonar_alt-sonar_alt);
    set_new_altitude(controller_desired_alt);

    get_throttle_althold_with_slew(controller_desired_alt, target_rate-sonar_induced_slew_rate, target_rate+sonar_induced_slew_rate);   // VELZ_MAX limits how quickly we react
}

/*
 *  reset all I integrators
 */
static void reset_I_all(void)
{
    reset_rate_I();
    reset_stability_I();
    reset_wind_I();
    reset_throttle_I();
    reset_optflow_I();

    // This is the only place we reset Yaw
    g.pi_stabilize_yaw.reset_I();
}

static void reset_rate_I()
{
    g.pid_rate_roll.reset_I();
    g.pid_rate_pitch.reset_I();
    g.pid_rate_yaw.reset_I();
}

static void reset_optflow_I(void)
{
    g.pid_optflow_roll.reset_I();
    g.pid_optflow_pitch.reset_I();
    of_roll = 0;
    of_pitch = 0;
}

static void reset_wind_I(void)
{
    // Wind Compensation
    // this i is not currently being used, but we reset it anyway
    // because someone may modify it and not realize it, causing a bug
    g.pi_loiter_lat.reset_I();
    g.pi_loiter_lon.reset_I();

    g.pid_loiter_rate_lat.reset_I();
    g.pid_loiter_rate_lon.reset_I();

    g.pid_nav_lat.reset_I();
    g.pid_nav_lon.reset_I();
}

static void reset_throttle_I(void)
{
    // For Altitude Hold
    g.pi_alt_hold.reset_I();
    g.pid_throttle.reset_I();
    g.pid_throttle_accel.reset_I();
}

static void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle)
{
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    g.pid_throttle_accel.set_integrator(pilot_throttle-g.throttle_cruise);
}

static void reset_stability_I(void)
{
    // Used to balance a quad
    // This only needs to be reset during Auto-leveling in flight
    g.pi_stabilize_roll.reset_I();
    g.pi_stabilize_pitch.reset_I();
}
#line 1 "./Firmware/ACopter32/GCS_Mavlink.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;


// this costs us 51 bytes, but means that low priority
// messages don't block the CPU
static mavlink_statustext_t pending_status;

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;

// true if we are out of time in our event timeslice
static bool	gcs_out_of_time;


// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_ ## id ## _LEN) return false

// prototype this for use inside the GCS class
static void gcs_send_text_fmt(const prog_char_t *fmt, ...);



// gcs_check_delay - keep GCS communications going
// in our delay callback
static void gcs_check_delay()
{
    static uint32_t last_1hz, last_50hz;

    uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs_send_heartbeat();
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_check_input();
        gcs_data_stream_send();
        gcs_send_deferred();
    }
    }

static void gcs_send_heartbeat(void)
{
    gcs_send_message(MSG_HEARTBEAT);
}

static void gcs_send_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
}

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

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    switch (control_mode) {
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    }

    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (motors.armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
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

#if AP_LIMITS == ENABLED
static NOINLINE void send_limits_status(mavlink_channel_t chan)
{
    limits_send_mavlink_status(chan);
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

    control_sensors_enabled |= (1<<10); // 3D angular rate control
    control_sensors_enabled |= (1<<11); // attitude stabilisation
    control_sensors_enabled |= (1<<13); // altitude control
    control_sensors_enabled |= (1<<15); // motor control

    switch (control_mode) {
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
    case POSITION:
        control_sensors_enabled |= (1<<12); // yaw position
        control_sensors_enabled |= (1<<14); // X/Y position control
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
        battery_remaining = (100.0f * (g.pack_capacity - current_total1) / g.pack_capacity);
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
        0, // CPU Load not supported in AC yet
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
        g_gps->ground_course);          // course in 1/100 degree
}

static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)
{
    mavlink_msg_nav_controller_output_send(
        chan,
        nav_roll / 1.0e2f,
        nav_pitch / 1.0e2f,
        wp_bearing / 1.0e2f,
        wp_bearing / 1.0e2f,
        wp_distance / 1.0e2f,
        altitude_error / 1.0e2f,
        0,
        crosstrack_error);      // was 0
}

static void NOINLINE send_ahrs(mavlink_channel_t chan)
{
    Vector3f omega_I = ahrs.get_gyro_drift();
    mavlink_msg_ahrs_send(
        chan,
        omega_I.x,
        omega_I.y,
        omega_I.z,
        1,
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
    // This is used for HIL.  Do not change without discussing with HIL maintainers

#if FRAME_CONFIG == HELI_FRAME

    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        g.rc_1.servo_out,
        g.rc_2.servo_out,
        g.rc_3.radio_out,
        g.rc_4.servo_out,
        0,
        0,
        0,
        0,
        receiver_rssi);
#else
 #if X_PLANE == ENABLED
    /* update by JLN for X-Plane HIL */
    if(motors.armed() && motors.auto_armed()) {
        mavlink_msg_rc_channels_scaled_send(
            chan,
            millis(),
            0,         // port 0
            g.rc_1.servo_out,
            g.rc_2.servo_out,
            10000 * g.rc_3.norm_output(),
            g.rc_4.servo_out,
            10000 * g.rc_1.norm_output(),
            10000 * g.rc_2.norm_output(),
            10000 * g.rc_3.norm_output(),
            10000 * g.rc_4.norm_output(),
            receiver_rssi);
    }else{
        mavlink_msg_rc_channels_scaled_send(
            chan,
            millis(),
            0,         // port 0
            0,
            0,
            -10000,
            0,
            10000 * g.rc_1.norm_output(),
            10000 * g.rc_2.norm_output(),
            10000 * g.rc_3.norm_output(),
            10000 * g.rc_4.norm_output(),
            receiver_rssi);
    }

 #else
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0,         // port 0
        g.rc_1.servo_out,
        g.rc_2.servo_out,
        g.rc_3.radio_out,
        g.rc_4.servo_out,
        10000 * g.rc_1.norm_output(),
        10000 * g.rc_2.norm_output(),
        10000 * g.rc_3.norm_output(),
        10000 * g.rc_4.norm_output(),
        receiver_rssi);
 #endif
#endif
}

static void NOINLINE send_radio_in(mavlink_channel_t chan)
{
    mavlink_msg_rc_channels_raw_send(
        chan,
        millis(),
        0, // port
        g.rc_1.radio_in,
        g.rc_2.radio_in,
        g.rc_3.radio_in,
        g.rc_4.radio_in,
        g.rc_5.radio_in,
        g.rc_6.radio_in,
        g.rc_7.radio_in,
        g.rc_8.radio_in,
        receiver_rssi);
}

static void NOINLINE send_radio_out(mavlink_channel_t chan)
{
    mavlink_msg_servo_output_raw_send(
        chan,
        micros(),
        0, // port
        motors.motor_out[AP_MOTORS_MOT_1],
        motors.motor_out[AP_MOTORS_MOT_2],
        motors.motor_out[AP_MOTORS_MOT_3],
        motors.motor_out[AP_MOTORS_MOT_4],
        motors.motor_out[AP_MOTORS_MOT_5],
        motors.motor_out[AP_MOTORS_MOT_6],
        motors.motor_out[AP_MOTORS_MOT_7],
        motors.motor_out[AP_MOTORS_MOT_8]);
}

static void NOINLINE send_vfr_hud(mavlink_channel_t chan)
{
    mavlink_msg_vfr_hud_send(
        chan,
        (float)g_gps->ground_speed / 100.0f,
        (float)g_gps->ground_speed / 100.0f,
        (ahrs.yaw_sensor / 100) % 360,
        g.rc_3.servo_out/10,
        current_loc.alt / 100.0f,
        climb_rate / 100.0f);
}

static void NOINLINE send_raw_imu1(mavlink_channel_t chan)
{
    Vector3f accel = ins.get_accel();
    Vector3f gyro = ins.get_gyro();
    mavlink_msg_raw_imu_send(
        chan,
        micros(),
        accel.x * 1000.0f / GRAVITY_MSS,
        accel.y * 1000.0f / GRAVITY_MSS,
        accel.z * 1000.0f / GRAVITY_MSS,
        gyro.x * 1000.0f,
        gyro.y * 1000.0f,
        gyro.z * 1000.0f,
        compass.mag_x,
        compass.mag_y,
        compass.mag_z);
}

static void NOINLINE send_raw_imu2(mavlink_channel_t chan)
{
    mavlink_msg_scaled_pressure_send(
        chan,
        millis(),
        (float)barometer.get_pressure()/100.0f,
        (float)(barometer.get_pressure() - barometer.get_ground_pressure())/100.0f,
        (int)(barometer.get_temperature()*10));
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

static void NOINLINE send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_mission_current_send(
        chan,
        (uint16_t)g.command_index);
}

static void NOINLINE send_statustext(mavlink_channel_t chan)
{
    mavlink_msg_statustext_send(
        chan,
        pending_status.severity,
        pending_status.text);
}

// are we still delaying telemetry to try to avoid Xbee bricking?
static bool telemetry_delayed(mavlink_channel_t chan)
{
    uint32_t tnow = millis() >> 10;
    if (tnow > (uint8_t)g.telem_delay) {
        return false;
    }
#if USB_MUX_PIN > 0
    if (chan == MAVLINK_COMM_0 && ap_system.usb_connected) {
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

    // if we don't have at least 1ms remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (scheduler.time_available_usec() < 800) {
        gcs_out_of_time = true;
        return false;
    }

    switch(id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        send_heartbeat(chan);
        break;

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
        CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
        send_nav_controller_output(chan);
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
        } else {
            gcs3.queued_waypoint_send();
        }
        break;

    case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

#if AP_LIMITS == ENABLED

    case MSG_LIMITS_STATUS:
        CHECK_PAYLOAD_SIZE(LIMITS_STATUS);
        send_limits_status(chan);
        break;

#endif

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
        pending_status.severity = (uint8_t)severity;
        strncpy((char *)pending_status.text, str, sizeof(pending_status.text));
        mavlink_send_message(chan, MSG_STATUSTEXT, 0);
    } else {
        // send immediately
        mavlink_msg_statustext_send(
            chan,
            severity,
            str);
    }
}

const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Raw sensor stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRateRawSensors,      0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Extended status stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRateExtendedStatus,  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: RC Channel stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRateRCChannels,      0),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Raw Control stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRateRawController,   0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Position stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRatePosition,        0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Extra data type 1 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRateExtra1,          0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Extra data type 2 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRateExtra2,          0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Extra data type 3 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRateExtra3,          0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Parameter stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
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
GCS_MAVLINK::init(AP_HAL::UARTDriver* port)
{
    GCS_Class::init(port);
    if (port == hal.uartA) {
        mavlink_comm_0_port = port;
        chan = MAVLINK_COMM_0;
    }else{
        mavlink_comm_1_port = port;
        chan = MAVLINK_COMM_1;
    }
    _queued_parameter = NULL;
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
    for (uint16_t i=0; i<nbytes; i++) {
        uint8_t c = comm_receive_ch(chan);

#if CLI_ENABLED == ENABLED
        /* allow CLI to be started by hitting enter 3 times, if no
         *  heartbeat packets have been received */
        if (mavlink_active == false) {
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

    if (!waypoint_receiving && !waypoint_sending) {
        return;
    }

    uint32_t tnow = millis();

    if (waypoint_receiving &&
        waypoint_request_i <= (unsigned)g.command_total &&
        tnow > waypoint_timelast_request + 500 + (stream_slowdown*20)) {
        waypoint_timelast_request = tnow;
        send_message(MSG_NEXT_WAYPOINT);
    }

    // stop waypoint sending if timeout
    if (waypoint_sending && (tnow - waypoint_timelast_send) > waypoint_send_timeout) {
        waypoint_sending = false;
    }

    // stop waypoint receiving if timeout
    if (waypoint_receiving && (tnow - waypoint_timelast_receive) > waypoint_receive_timeout) {
        waypoint_receiving = false;
    }
}

// see if we should send a stream now. Called at 50Hz
bool GCS_MAVLINK::stream_trigger(enum streams stream_num)
{
    uint8_t rate;
    switch (stream_num) {
        case STREAM_RAW_SENSORS:
            rate = streamRateRawSensors.get();
            break;
        case STREAM_EXTENDED_STATUS:
            rate = streamRateExtendedStatus.get();
            break;
        case STREAM_RC_CHANNELS:
            rate = streamRateRCChannels.get();
            break;
        case STREAM_RAW_CONTROLLER:
            rate = streamRateRawController.get();
            break;
        case STREAM_POSITION:
            rate = streamRatePosition.get();
            break;
        case STREAM_EXTRA1:
            rate = streamRateExtra1.get();
            break;
        case STREAM_EXTRA2:
            rate = streamRateExtra2.get();
            break;
        case STREAM_EXTRA3:
            rate = streamRateExtra3.get();
            break;
        case STREAM_PARAMS:
            rate = streamRateParams.get();
            break;
        default:
            rate = 0;
    }

    if (rate == 0) {
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
    if (waypoint_receiving || waypoint_sending) {
        // don't interfere with mission transfer
        return;
    }

    gcs_out_of_time = false;

    if (_queued_parameter != NULL) {
        if (streamRateParams.get() <= 0) {
            streamRateParams.set(50);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
        // don't send anything else at the same time as parameters
        return;
    }

    if (gcs_out_of_time) return;

    if (in_mavlink_delay) {
        // don't send any other stream types while in the delay callback
        return;
    }

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU2);
        send_message(MSG_RAW_IMU3);
        //cliSerial->printf("mav1 %d\n", (int)streamRateRawSensors.get());
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
        send_message(MSG_LIMITS_STATUS);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_POSITION)) {
        send_message(MSG_LOCATION);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
        //cliSerial->printf("mav4 %d\n", (int)streamRateRawController.get());
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_RADIO_OUT);
        send_message(MSG_RADIO_IN);
        //cliSerial->printf("mav5 %d\n", (int)streamRateRCChannels.get());
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
        //cliSerial->printf("mav6 %d\n", (int)streamRateExtra1.get());
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
        //cliSerial->printf("mav7 %d\n", (int)streamRateExtra2.get());
    }

    if (gcs_out_of_time) return;

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
    struct Location tell_command = {};                                  // command for telemetry
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:     //66
    {
        // decode
        mavlink_request_data_stream_t packet;
        mavlink_msg_request_data_stream_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        int16_t freq = 0;                 // packet frequency

        if (packet.start_stop == 0)
            freq = 0;                     // stop sending
        else if (packet.start_stop == 1)
            freq = packet.req_message_rate;                     // start sending
        else
            break;

        switch(packet.req_stream_id) {

        case MAV_DATA_STREAM_ALL:
            streamRateRawSensors            = freq;
            streamRateExtendedStatus        = freq;
            streamRateRCChannels            = freq;
            streamRateRawController         = freq;
            streamRatePosition                      = freq;
            streamRateExtra1                        = freq;
            streamRateExtra2                        = freq;
            //streamRateExtra3.set_and_save(freq);	// We just do set and save on the last as it takes care of the whole group.
            streamRateExtra3                        = freq;                             // Don't save!!
            break;

        case MAV_DATA_STREAM_RAW_SENSORS:
            streamRateRawSensors = freq;                                        // We do not set and save this one so that if HIL is shut down incorrectly
            // we will not continue to broadcast raw sensor data at 50Hz.
            break;
        case MAV_DATA_STREAM_EXTENDED_STATUS:
            //streamRateExtendedStatus.set_and_save(freq);
            streamRateExtendedStatus = freq;
            break;

        case MAV_DATA_STREAM_RC_CHANNELS:
            streamRateRCChannels  = freq;
            break;

        case MAV_DATA_STREAM_RAW_CONTROLLER:
            streamRateRawController = freq;
            break;

        //case MAV_DATA_STREAM_RAW_SENSOR_FUSION:
        //	streamRateRawSensorFusion.set_and_save(freq);
        //	break;

        case MAV_DATA_STREAM_POSITION:
            streamRatePosition = freq;
            break;

        case MAV_DATA_STREAM_EXTRA1:
            streamRateExtra1 = freq;
            break;

        case MAV_DATA_STREAM_EXTRA2:
            streamRateExtra2 = freq;
            break;

        case MAV_DATA_STREAM_EXTRA3:
            streamRateExtra3 = freq;
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

        case MAV_CMD_NAV_LOITER_UNLIM:
            set_mode(LOITER);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            set_mode(RTL);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_NAV_LAND:
            set_mode(LAND);
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
                ins.init_accel(flash_leds);
                ahrs.set_trim(Vector3f(0,0,0));             // clear out saved trim
            }
            if (packet.param4 == 1) {
                trim_radio();
            }
            if (packet.param5 == 1) {
                // this blocks
                AP_InertialSensor_UserInteractStream interact(hal.console);
                ins.calibrate_accel(flash_leds, &interact);
            }
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (packet.target_component == MAV_COMP_ID_SYSTEM_CONTROL) {
                if (packet.param1 == 1.0f) {
                    init_arm_motors();
                    result = MAV_RESULT_ACCEPTED;
                } else if (packet.param1 == 0.0f)  {
                    init_disarm_motors();
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_UNSUPPORTED;
                }
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (packet.param1 == 1) {
                reboot_apm();
                result = MAV_RESULT_ACCEPTED;
            }
            break;


        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
        }

        mavlink_msg_command_ack_send(
            chan,
            packet.command,
            result);

        break;
    }

    case MAVLINK_MSG_ID_SET_MODE:      //11
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
        case STABILIZE:
        case ACRO:
        case ALT_HOLD:
        case AUTO:
        case GUIDED:
        case LOITER:
        case RTL:
        case CIRCLE:
        case POSITION:
        case LAND:
        case OF_LOITER:
            set_mode(packet.custom_mode);
            break;
        }

        break;
    }

    /*case MAVLINK_MSG_ID_SET_NAV_MODE:
     *       {
     *               // decode
     *               mavlink_set_nav_mode_t packet;
     *               mavlink_msg_set_nav_mode_decode(msg, &packet);
     *               // To set some flight modes we must first receive a "set nav mode" message and then a "set mode" message
     *               mav_nav = packet.nav_mode;
     *               break;
     *       }
     */
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:     //43
    {
        //send_text_P(SEVERITY_LOW,PSTR("waypoint request list"));

        // decode
        mavlink_mission_request_list_t packet;
        mavlink_msg_mission_request_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        // Start sending waypoints
        mavlink_msg_mission_count_send(
            chan,msg->sysid,
            msg->compid,
            g.command_total);                     // includes home

        waypoint_timelast_send          = millis();
        waypoint_sending                        = true;
        waypoint_receiving                      = false;
        waypoint_dest_sysid                     = msg->sysid;
        waypoint_dest_compid            = msg->compid;
        break;
    }

    // XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:     // 40
    {
        //send_text_P(SEVERITY_LOW,PSTR("waypoint request"));

        // Check if sending waypiont
        //if (!waypoint_sending) break;
        // 5/10/11 - We are trying out relaxing the requirement that we be in waypoint sending mode to respond to a waypoint request.  DEW

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
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;                     // reference frame
        } else {
            frame = MAV_FRAME_GLOBAL;                     // reference frame
        }

        float param1 = 0, param2 = 0, param3 = 0, param4 = 0;

        // time that the mav should loiter in milliseconds
        uint8_t current = 0;                 // 1 (true), 0 (false)

        if (packet.seq == (uint16_t)g.command_index)
            current = 1;

        uint8_t autocontinue = 1;                 // 1 (true), 0 (false)

        float x = 0, y = 0, z = 0;

        if (tell_command.id < MAV_CMD_NAV_LAST) {
            // command needs scaling
            x = tell_command.lat/1.0e7f;                     // local (x), global (latitude)
            y = tell_command.lng/1.0e7f;                     // local (y), global (longitude)
            // ACM is processing alt inside each command. so we save and load raw values. - this is diffrent to APM
            z = tell_command.alt/1.0e2f;                     // local (z), global/relative (altitude)
        }

        // Switch to map APM command fields into MAVLink command fields
        switch (tell_command.id) {

        case MAV_CMD_NAV_LOITER_TURNS:
        case MAV_CMD_CONDITION_CHANGE_ALT:
        case MAV_CMD_DO_SET_HOME:
            param1 = tell_command.p1;
            break;

        case MAV_CMD_NAV_ROI:
            param1 = tell_command.p1;                                   // MAV_ROI (aka roi mode) is held in wp's parameter but we actually do nothing with it because we only support pointing at a specific location provided by x,y and z parameters
            break;

        case MAV_CMD_CONDITION_YAW:
            param3 = tell_command.p1;
            param1 = tell_command.alt;
            param2 = tell_command.lat;
            param4 = tell_command.lng;
            break;

        case MAV_CMD_NAV_TAKEOFF:
            param1 = 0;
            break;

        case MAV_CMD_NAV_LOITER_TIME:
            param1 = tell_command.p1;                                   // ACM loiter time is in 1 second increments
            break;

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

        case MAV_CMD_NAV_WAYPOINT:
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

    case MAVLINK_MSG_ID_MISSION_ACK:     //47
    {
        //send_text_P(SEVERITY_LOW,PSTR("waypoint ack"));

        // decode
        mavlink_mission_ack_t packet;
        mavlink_msg_mission_ack_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // turn off waypoint send
        waypoint_sending = false;
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:     // 21
    {
        // gcs_send_text_P(SEVERITY_LOW,PSTR("param request list"));

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

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:     // 45
    {
        //send_text_P(SEVERITY_LOW,PSTR("waypoint clear all"));

        // decode
        mavlink_mission_clear_all_t packet;
        mavlink_msg_mission_clear_all_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component)) break;

        // clear all waypoints
        uint8_t type = 0;                 // ok (0), error(1)
        g.command_total.set_and_save(1);

        // send acknowledgement 3 times to makes sure it is received
        for (int16_t i=0; i<3; i++)
            mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, type);

        break;
    }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:     // 41
    {
        //send_text_P(SEVERITY_LOW,PSTR("waypoint set current"));

        // decode
        mavlink_mission_set_current_t packet;
        mavlink_msg_mission_set_current_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // set current command
        change_command(packet.seq);

        mavlink_msg_mission_current_send(chan, g.command_index);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_COUNT:     // 44
    {
        //send_text_P(SEVERITY_LOW,PSTR("waypoint count"));

        // decode
        mavlink_mission_count_t packet;
        mavlink_msg_mission_count_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // start waypoint receiving
        if (packet.count > MAX_WAYPOINTS) {
            packet.count = MAX_WAYPOINTS;
        }
        g.command_total.set_and_save(packet.count);

        waypoint_timelast_receive = millis();
        waypoint_receiving   = true;
        waypoint_sending         = false;
        waypoint_request_i   = 0;
        waypoint_timelast_request = 0;
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
    case MAVLINK_MSG_ID_MISSION_ITEM:     //39
    {
        // decode
        mavlink_mission_item_t packet;
        mavlink_msg_mission_item_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // defaults
        tell_command.id = packet.command;

        /*
         *  switch (packet.frame){
         *
         *       case MAV_FRAME_MISSION:
         *       case MAV_FRAME_GLOBAL:
         *               {
         *                       tell_command.lat = 1.0e7*packet.x; // in as DD converted to * t7
         *                       tell_command.lng = 1.0e7*packet.y; // in as DD converted to * t7
         *                       tell_command.alt = packet.z*1.0e2; // in as m converted to cm
         *                       tell_command.options = 0; // absolute altitude
         *                       break;
         *               }
         *
         *       case MAV_FRAME_LOCAL: // local (relative to home position)
         *               {
         *                       tell_command.lat = 1.0e7*ToDeg(packet.x/
         *                       (radius_of_earth*cosf(ToRad(home.lat/1.0e7)))) + home.lat;
         *                       tell_command.lng = 1.0e7*ToDeg(packet.y/radius_of_earth) + home.lng;
         *                       tell_command.alt = packet.z*1.0e2;
         *                       tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
         *                       break;
         *               }
         *       //case MAV_FRAME_GLOBAL_RELATIVE_ALT: // absolute lat/lng, relative altitude
         *       default:
         *               {
         *                       tell_command.lat = 1.0e7 * packet.x; // in as DD converted to * t7
         *                       tell_command.lng = 1.0e7 * packet.y; // in as DD converted to * t7
         *                       tell_command.alt = packet.z * 1.0e2;
         *                       tell_command.options = MASK_OPTIONS_RELATIVE_ALT; // store altitude relative!! Always!!
         *                       break;
         *               }
         *  }
         */

        // we only are supporting Abs position, relative Alt
        tell_command.lat = 1.0e7f * packet.x;                 // in as DD converted to * t7
        tell_command.lng = 1.0e7f * packet.y;                 // in as DD converted to * t7
        tell_command.alt = packet.z * 1.0e2f;
        tell_command.options = 1;                 // store altitude relative to home alt!! Always!!

        switch (tell_command.id) {                                                      // Switch to map APM command fields into MAVLink command fields
        case MAV_CMD_NAV_LOITER_TURNS:
        case MAV_CMD_DO_SET_HOME:
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_NAV_ROI:
            tell_command.p1 = packet.param1;                                    // MAV_ROI (aka roi mode) is held in wp's parameter but we actually do nothing with it because we only support pointing at a specific location provided by x,y and z parameters
            break;

        case MAV_CMD_CONDITION_YAW:
            tell_command.p1 = packet.param3;
            tell_command.alt = packet.param1;
            tell_command.lat = packet.param2;
            tell_command.lng = packet.param4;
            break;

        case MAV_CMD_NAV_TAKEOFF:
            tell_command.p1 = 0;
            break;

        case MAV_CMD_CONDITION_CHANGE_ALT:
            tell_command.p1 = packet.param1 * 100;
            break;

        case MAV_CMD_NAV_LOITER_TIME:
            tell_command.p1 = packet.param1;                                    // APM loiter time is in ten second increments
            break;

        case MAV_CMD_CONDITION_DELAY:
        case MAV_CMD_CONDITION_DISTANCE:
            tell_command.lat = packet.param1;
            break;

        case MAV_CMD_DO_JUMP:
            tell_command.lat = packet.param2;
            tell_command.p1  = packet.param1;
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            tell_command.lng = packet.param4;
        case MAV_CMD_DO_REPEAT_RELAY:
        case MAV_CMD_DO_CHANGE_SPEED:
            tell_command.lat = packet.param3;
            tell_command.alt = packet.param2;
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_NAV_WAYPOINT:
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_DO_SET_PARAMETER:
        case MAV_CMD_DO_SET_RELAY:
        case MAV_CMD_DO_SET_SERVO:
            tell_command.alt = packet.param2;
            tell_command.p1 = packet.param1;
            break;
        }

        if(packet.current == 2) {                                               //current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
            guided_WP = tell_command;

            // add home alt if needed
            if (guided_WP.options & MASK_OPTIONS_RELATIVE_ALT) {
                guided_WP.alt += home.alt;
            }

            set_mode(GUIDED);

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

            set_new_altitude(tell_command.alt);

            // verify we recevied the command
            mavlink_msg_mission_ack_send(
                chan,
                msg->sysid,
                msg->compid,
                0);

        } else {
            // Check if receiving waypoints (mission upload expected)
            if (!waypoint_receiving) break;


            //cliSerial->printf("req: %d, seq: %d, total: %d\n", waypoint_request_i,packet.seq, g.command_total.get());

            // check if this is the requested waypoint
            if (packet.seq != waypoint_request_i)
                break;

            if(packet.seq != 0)
                set_cmd_with_index(tell_command, packet.seq);

            // update waypoint receiving state machine
            waypoint_timelast_receive = millis();
            waypoint_timelast_request = 0;
            waypoint_request_i++;

            if (waypoint_request_i == (uint16_t)g.command_total) {
                uint8_t type = 0;                         // ok (0), error(1)

                mavlink_msg_mission_ack_send(
                    chan,
                    msg->sysid,
                    msg->compid,
                    type);

                send_text_P(SEVERITY_LOW,PSTR("flight plan received"));
                waypoint_receiving = false;
                // XXX ignores waypoint radius for individual waypoints, can
                // only set WP_RADIUS parameter
            }
        }
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET:     // 23
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
        if ((NULL != vp) &&                                                                                     // exists
            !isnan(packet.param_value) &&                                                  // not nan
            !isinf(packet.param_value)) {                                                  // not inf

            // add a small amount before casting parameter values
            // from float to integer to avoid truncating to the
            // next lower integer value.
            float rounding_addition = 0.01;

            // handle variables with standard type IDs
            if (var_type == AP_PARAM_FLOAT) {
                ((AP_Float *)vp)->set_and_save(packet.param_value);
            } else if (var_type == AP_PARAM_INT32) {
#if LOGGING_ENABLED == ENABLED
                if (g.log_bitmask != 0) {
                    Log_Write_Data(DATA_MAVLINK_FLOAT, ((AP_Int32 *)vp)->get());
                }
#endif
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain(v, -2147483648.0, 2147483647.0);
                ((AP_Int32 *)vp)->set_and_save(v);
            } else if (var_type == AP_PARAM_INT16) {
#if LOGGING_ENABLED == ENABLED
                if (g.log_bitmask != 0) {
                    Log_Write_Data(DATA_MAVLINK_INT16, (int16_t)((AP_Int16 *)vp)->get());
                }
#endif
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain(v, -32768, 32767);
                ((AP_Int16 *)vp)->set_and_save(v);
            } else if (var_type == AP_PARAM_INT8) {
#if LOGGING_ENABLED == ENABLED
                if (g.log_bitmask != 0) {
                    Log_Write_Data(DATA_MAVLINK_INT8, (int8_t)((AP_Int8 *)vp)->get());
                }
#endif
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
                -1);                         // XXX we don't actually know what its index is...

        }

        break;
    }             // end case

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: //70
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
        break;
    }


#if HIL_MODE != HIL_MODE_DISABLED
    case MAVLINK_MSG_ID_HIL_STATE:
    {
        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(msg, &packet);

        float vel = pythagorous2(packet.vx, packet.vy);
        float cog = wrap_360(ToDeg(atan2f(packet.vx, packet.vy)) * 100);

        // set gps hil sensor
        g_gps->setHIL(packet.time_usec/1000,
                      packet.lat*1.0e-7, packet.lon*1.0e-7, packet.alt*1.0e-3,
                      vel*1.0e-2, cog*1.0e-2, 0, 10);

        if (gps_base_alt == 0) {
            gps_base_alt = g_gps->altitude;
        }
        current_loc.lng = g_gps->longitude;
        current_loc.lat = g_gps->latitude;
        current_loc.alt = g_gps->altitude - gps_base_alt;
        if (!ap.home_is_set) {
            init_home();
        }


        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;
        // m/s/s
        Vector3f accels;
        accels.x = (float)packet.xacc / 1000.0;
        accels.y = (float)packet.yacc / 1000.0;
        accels.z = (float)packet.zacc / 1000.0;

        ins.set_gyro_offsets(gyros);

        ins.set_accel_offsets(accels);


        // set AHRS hil sensor
        ahrs.setHil(packet.roll,packet.pitch,packet.yaw,packet.rollspeed,
                    packet.pitchspeed,packet.yawspeed);



        break;
    }
#endif //  HIL_MODE != HIL_MODE_DISABLED

/*
 *       case MAVLINK_MSG_ID_HEARTBEAT:
 *               {
 *                       // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
 *                       if(msg->sysid != g.sysid_my_gcs) break;
 *                       rc_override_fs_timer = millis();
 *                       break;
 *               }
 *
 * #if HIL_MODE != HIL_MODE_DISABLED
 *               // This is used both as a sensor and to pass the location
 *               // in HIL_ATTITUDE mode.
 *       case MAVLINK_MSG_ID_GPS_RAW:
 *               {
 *                       // decode
 *                       mavlink_gps_raw_t packet;
 *                       mavlink_msg_gps_raw_decode(msg, &packet);
 *
 *                       // set gps hil sensor
 *                       g_gps->setHIL(packet.usec/1000,packet.lat,packet.lon,packet.alt,
 *                       packet.v,packet.hdg,0,0);
 *                       break;
 *               }
 * #endif
 */
#if HIL_MODE == HIL_MODE_SENSORS

    case MAVLINK_MSG_ID_RAW_IMU: // 28
    {
        // decode
        mavlink_raw_imu_t packet;
        mavlink_msg_raw_imu_decode(msg, &packet);

        // set imu hil sensors
        // TODO: check scaling for temp/absPress
        float temp = 70;
        float absPress = 1;
        //      cliSerial->printf_P(PSTR("accel:\t%d\t%d\t%d\n"), packet.xacc, packet.yacc, packet.zacc);
        //      cliSerial->printf_P(PSTR("gyro:\t%d\t%d\t%d\n"), packet.xgyro, packet.ygyro, packet.zgyro);

        // rad/sec
        Vector3f gyros;
        gyros.x = (float)packet.xgyro / 1000.0;
        gyros.y = (float)packet.ygyro / 1000.0;
        gyros.z = (float)packet.zgyro / 1000.0;
        // m/s/s
        Vector3f accels;
        accels.x = (float)packet.xacc / 1000.0;
        accels.y = (float)packet.yacc / 1000.0;
        accels.z = (float)packet.zacc / 1000.0;

        ins.set_gyro_offsets(gyros);

        ins.set_accel_offsets(accels);

        compass.setHIL(packet.xmag,packet.ymag,packet.zmag);
        break;
    }

    case MAVLINK_MSG_ID_RAW_PRESSURE: //29
    {
        // decode
        mavlink_raw_pressure_t packet;
        mavlink_msg_raw_pressure_decode(msg, &packet);

        // set pressure hil sensor
        // TODO: check scaling
        float temp = 70;
        barometer.setHIL(temp,packet.press_diff1);
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

#if AP_LIMITS == ENABLED

    // receive an AP_Limits fence point from GCS and store in EEPROM
    // receive a fence point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_FENCE_POINT: {
        mavlink_fence_point_t packet;
        mavlink_msg_fence_point_decode(msg, &packet);
        if (packet.count != geofence_limit.fence_total()) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point;
            point.x = packet.lat*1.0e7f;
            point.y = packet.lng*1.0e7f;
            geofence_limit.set_fence_point_with_index(point, packet.idx);
        }
        break;
    }
    // send a fence point to GCS
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
        mavlink_fence_fetch_point_t packet;
        mavlink_msg_fence_fetch_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        if (packet.idx >= geofence_limit.fence_total()) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point = geofence_limit.get_fence_point_with_index(packet.idx);
            mavlink_msg_fence_point_send(chan, 0, 0, packet.idx, geofence_limit.fence_total(),
                                         point.x*1.0e-7f, point.y*1.0e-7f);
        }
        break;
    }


#endif // AP_LIMITS ENABLED

    }     // end switch
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
 * queued_param_send - Send the next pending parameter, called from deferred message
 * handling code
 */
void
GCS_MAVLINK::queued_param_send()
{
    // Check to see if we are sending parameters
    if (NULL == _queued_parameter) return;

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

/**
 * queued_waypoint_send - Send the next pending waypoint, called from deferred message
 * handling code
 */
void
GCS_MAVLINK::queued_waypoint_send()
{
    if (waypoint_receiving &&
        waypoint_request_i < (unsigned)g.command_total) {
        mavlink_msg_mission_request_send(
            chan,
            waypoint_dest_sysid,
            waypoint_dest_compid,
            waypoint_request_i);
    }
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
        gcs_send_heartbeat();
        gcs_send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_check_input();
        gcs_data_stream_send();
        gcs_send_deferred();
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
static void gcs_check_input(void)
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
static void gcs_send_text_fmt(const prog_char_t *fmt, ...)
{
    va_list arg_list;
    pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)pending_status.text,
            sizeof(pending_status.text), fmt, arg_list);
    va_end(arg_list);
    mavlink_send_message(MAVLINK_COMM_0, MSG_STATUSTEXT, 0);
    if (gcs3.initialised) {
        mavlink_send_message(MAVLINK_COMM_1, MSG_STATUSTEXT, 0);
    }
}

#line 1 "./Firmware/ACopter32/Log.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static bool     print_log_menu(void);
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
static int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command log_menu_commands[] PROGMEM = {
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

    cliSerial->printf_P(PSTR("logs enabled: "));

    if (0 == g.log_bitmask) {
        cliSerial->printf_P(PSTR("none"));
    }else{
        if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST) cliSerial->printf_P(PSTR(" ATTITUDE_FAST"));
        if (g.log_bitmask & MASK_LOG_ATTITUDE_MED) cliSerial->printf_P(PSTR(" ATTITUDE_MED"));
        if (g.log_bitmask & MASK_LOG_GPS) cliSerial->printf_P(PSTR(" GPS"));
        if (g.log_bitmask & MASK_LOG_PM) cliSerial->printf_P(PSTR(" PM"));
        if (g.log_bitmask & MASK_LOG_CTUN) cliSerial->printf_P(PSTR(" CTUN"));
        if (g.log_bitmask & MASK_LOG_NTUN) cliSerial->printf_P(PSTR(" NTUN"));
        if (g.log_bitmask & MASK_LOG_IMU) cliSerial->printf_P(PSTR(" IMU"));
        if (g.log_bitmask & MASK_LOG_CMD) cliSerial->printf_P(PSTR(" CMD"));
        if (g.log_bitmask & MASK_LOG_CURRENT) cliSerial->printf_P(PSTR(" CURRENT"));
        if (g.log_bitmask & MASK_LOG_MOTORS) cliSerial->printf_P(PSTR(" MOTORS"));
        if (g.log_bitmask & MASK_LOG_OPTFLOW) cliSerial->printf_P(PSTR(" OPTFLOW"));
        if (g.log_bitmask & MASK_LOG_PID) cliSerial->printf_P(PSTR(" PID"));
        if (g.log_bitmask & MASK_LOG_ITERM) cliSerial->printf_P(PSTR(" ITERM"));
        if (g.log_bitmask & MASK_LOG_INAV) cliSerial->printf_P(PSTR(" INAV"));
        if (g.log_bitmask & MASK_LOG_CAMERA) cliSerial->printf_P(PSTR(" CAMERA"));
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
    } else if ((argc != 2) || (dump_log <= (last_log_num - DataFlash.get_num_logs())) || (dump_log > last_log_num)) {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    /*cliSerial->printf_P(PSTR("Dumping Log number %d,    start %d,   end %d\n"),
     *                         dump_log,
     *                         dump_log_start,
     *                         dump_log_end);
     */
    Log_Read(dump_log_start, dump_log_end);
    //cliSerial->printf_P(PSTR("Done\n"));
    return (0);
}

static void do_erase_logs(void)
{
	gcs_send_text_P(SEVERITY_LOW, PSTR("Erasing logs\n"));
    DataFlash.EraseAll();
	gcs_send_text_P(SEVERITY_LOW, PSTR("Log erase complete\n"));
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
        TARG(MOTORS);
        TARG(OPTFLOW);
        TARG(PID);
        TARG(ITERM);
        TARG(INAV);
        TARG(CAMERA);
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

// Write an GPS packet. Total length : 31 bytes
static void Log_Write_GPS()
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

    // need to fix printf formatting

    cliSerial->printf_P(PSTR("GPS, %ld, %u, "),
                        (long)pkt.gps_time,
                        (unsigned)pkt.num_sats);
    print_latlon(cliSerial, pkt.latitude);
    cliSerial->print_P(PSTR(", "));
    print_latlon(cliSerial, pkt.longitude);
    cliSerial->printf_P(PSTR(", %4.4f, %4.4f, %lu, %ld\n"),
                        pkt.rel_altitude*0.01,
                        pkt.altitude*0.01,
                        (unsigned long)pkt.ground_speed,
                        (long)pkt.ground_course);
}

struct log_IMU {
    LOG_PACKET_HEADER;
    Vector3f gyro;
    Vector3f accel;
};

// Write an imu accel/gyro packet. Total length : 27 bytes
static void Log_Write_IMU()
{
    struct log_IMU pkt = {
        LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
        gyro      : ins.get_gyro(),
        accel     : ins.get_accel()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a raw accel/gyro packet
static void Log_Read_IMU()
{
    struct log_IMU pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                                 1      2      3      4      5      6
    cliSerial->printf_P(PSTR("IMU, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f\n"),
        (float)pkt.gyro.x,
        (float)pkt.gyro.y,
        (float)pkt.gyro.z,
        (float)pkt.accel.x,
        (float)pkt.accel.y,
        (float)pkt.accel.z);
}

struct log_Current {
    LOG_PACKET_HEADER;
    int16_t throttle_in;
    uint32_t throttle_integrator;
    int16_t battery_voltage;
    int16_t current_amps;
    int16_t current_total;
};

// Write an Current data packet. Total length : 16 bytes
static void Log_Write_Current()
{
    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        throttle_in         : g.rc_3.control_in,
        throttle_integrator : throttle_integrator,
        battery_voltage     : (int16_t) (battery_voltage1 * 100.0f),
        current_amps        : (int16_t) (current_amps1 * 100.0f),
        current_total       : (int16_t) current_total1
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a Current packet
static void Log_Read_Current()
{
    struct log_Current pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                                  1    2      3      4   5
    cliSerial->printf_P(PSTR("CURRENT, %d, %lu, %4.4f, %4.4f, %d\n"),
                    (int)pkt.throttle_in,
                    (unsigned long)pkt.throttle_integrator,
                    (float)pkt.battery_voltage/100.0f,
                    (float)pkt.current_amps/100.0f,
                    (int)pkt.current_total);
}

struct log_Motors {
    LOG_PACKET_HEADER;
#if FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME
    int16_t motor_out[8];
#elif FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME
    int16_t motor_out[6];
#elif FRAME_CONFIG == HELI_FRAME
    int16_t motor_out[4];
    int16_t ext_gyro_gain;
#else        // quads & TRI_FRAME
    int16_t motor_out[4];
#endif
};

// Write an Motors packet. Total length : 12 ~ 20 bytes
static void Log_Write_Motors()
{
    struct log_Motors pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MOTORS_MSG),
#if FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_3],
                         motors.motor_out[AP_MOTORS_MOT_4],
                         motors.motor_out[AP_MOTORS_MOT_5],
                         motors.motor_out[AP_MOTORS_MOT_6],
                         motors.motor_out[AP_MOTORS_MOT_7],
                         motors.motor_out[AP_MOTORS_MOT_8]}
#elif FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_3],
                         motors.motor_out[AP_MOTORS_MOT_4],
                         motors.motor_out[AP_MOTORS_MOT_5],
                         motors.motor_out[AP_MOTORS_MOT_6]}
#elif FRAME_CONFIG == HELI_FRAME
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_3],
                         motors.motor_out[AP_MOTORS_MOT_4]},
        ext_gyro_gain   : motors.ext_gyro_gain
#elif FRAME_CONFIG == TRI_FRAME
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_4],
                         motors.motor_out[g.rc_4.radio_out]}
#else // QUAD frame
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_3],
                         motors.motor_out[AP_MOTORS_MOT_4]}
#endif
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a Motors packet.
static void Log_Read_Motors()
{
    struct log_Motors pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

#if FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME
                                 // 1   2   3   4   5   6   7   8
    cliSerial->printf_P(PSTR("MOT, %d, %d, %d, %d, %d, %d, %d, %d\n"),
                    (int)pkt.motor_out[0],
                    (int)pkt.motor_out[1],
                    (int)pkt.motor_out[2],
                    (int)pkt.motor_out[3],
                    (int)pkt.motor_out[4],
                    (int)pkt.motor_out[5],
                    (int)pkt.motor_out[6],
                    (int)pkt.motor_out[7]);
#elif FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME
                                 // 1   2   3   4   5   6
    cliSerial->printf_P(PSTR("MOT, %d, %d, %d, %d, %d, %d\n"),
                    (int)pkt.motor_out[0],
                    (int)pkt.motor_out[1],
                    (int)pkt.motor_out[2],
                    (int)pkt.motor_out[3],
                    (int)pkt.motor_out[4],
                    (int)pkt.motor_out[5]);
#elif FRAME_CONFIG == HELI_FRAME
                                 // 1   2   3   4   5
    cliSerial->printf_P(PSTR("MOT, %d, %d, %d, %d, %d\n"),
                    (int)pkt.motor_out[0],
                    (int)pkt.motor_out[1],
                    (int)pkt.motor_out[2],
                    (int)pkt.motor_out[3],
                    (int)pkt.ext_gyro_gain);
#else // TRI_FRAME or QUAD_FRAME
                                 // 1   2   3   4
    cliSerial->printf_P(PSTR("MOT, %d, %d, %d, %d\n"),
                    (int)pkt.motor_out[0],
                    (int)pkt.motor_out[1],
                    (int)pkt.motor_out[2],
                    (int)pkt.motor_out[3]);
#endif
}

struct log_Optflow {
    LOG_PACKET_HEADER;
    int16_t dx;
    int16_t dy;
    uint8_t surface_quality;
    int16_t x_cm;
    int16_t y_cm;
    float   latitude;
    float   longitude;
    int32_t roll;
    int32_t pitch;
};

// Write an optical flow packet. Total length : 30 bytes
static void Log_Write_Optflow()
{
 #if OPTFLOW == ENABLED
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        dx              : optflow.dx,
        dy              : optflow.dx,
        surface_quality : optflow.surface_quality,
        x_cm            : (int16_t) optflow.x_cm,
        y_cm            : (int16_t) optflow.y_cm,
        latitude        : optflow.vlat,
        longitude       : optflow.vlon,
        roll            : of_roll,
        pitch           : of_pitch
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
 #endif     // OPTFLOW == ENABLED
}

// Read an optical flow packet.
static void Log_Read_Optflow()
{
    struct log_Optflow pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                             1   2   3   4   5      6      7    8    9
    cliSerial->printf_P(PSTR("OF, %d, %d, %d, %d, %d, %4.7f, %4.7f, %ld, %ld\n"),
                    (int)pkt.dx,
                    (int)pkt.dy,
                    (int)pkt.surface_quality,
                    (int)pkt.x_cm,
                    (int)pkt.y_cm,
                    (float)pkt.latitude,
                    (float)pkt.longitude,
                    (long)pkt.roll,
                    (long)pkt.pitch);
}

struct log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t wp_distance;
    int16_t wp_bearing;
    int16_t lat_error;
    int16_t lon_error;
    int16_t nav_pitch;
    int16_t nav_roll;
    int16_t lat_speed;
    int16_t lon_speed;
};

// Write an Nav Tuning packet. Total length : 24 bytes
static void Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        wp_distance : wp_distance,
        wp_bearing  : (int16_t) (wp_bearing/100),
        lat_error   : (int16_t) lat_error,
        lon_error   : (int16_t) long_error,
        nav_pitch   : (int16_t) nav_pitch,
        nav_roll    : (int16_t) nav_roll,
        lat_speed   : lat_speed,
        lon_speed   : lon_speed
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a Nav Tuning packet.
static void Log_Read_Nav_Tuning()
{
    struct log_Nav_Tuning pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                               1   2   3   4   5   6   7   8
    cliSerial->printf_P(PSTR("NTUN, %lu, %d, %d, %d, %d, %d, %d, %d\n"),
        (unsigned long)pkt.wp_distance,
        (int)pkt.wp_bearing,
        (int)pkt.lat_error,
        (int)pkt.lon_error,
        (int)pkt.nav_pitch,
        (int)pkt.nav_roll,
        (int)pkt.lat_speed,
        (int)pkt.lon_speed
    );
}

struct log_Control_Tuning {
    LOG_PACKET_HEADER;
    int16_t throttle_in;
    int16_t sonar_alt;
    int16_t baro_alt;
    int16_t next_wp_alt;
    int16_t nav_throttle;
    int16_t angle_boost;
    int16_t climb_rate;
    int16_t throttle_out;
    int16_t desired_climb_rate;
};

// Write a control tuning packet. Total length : 26 bytes
static void Log_Write_Control_Tuning()
{
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        throttle_in         : g.rc_3.control_in,
        sonar_alt           : sonar_alt,
        baro_alt            : (int16_t) baro_alt,
        next_wp_alt         : (int16_t) next_WP.alt,
        nav_throttle        : nav_throttle,
        angle_boost         : angle_boost,
        climb_rate          : climb_rate,
        throttle_out        : g.rc_3.servo_out,
        desired_climb_rate  : desired_climb_rate
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read an control tuning packet
static void Log_Read_Control_Tuning()
{
    struct log_Control_Tuning pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                               1   2   3   4   5   6   7   8   9
    cliSerial->printf_P(PSTR("CTUN, %d, %d, %d, %d, %d, %d, %d, %d, %d\n"),
        (int)pkt.throttle_in,
        (int)pkt.sonar_alt,
        (int)pkt.baro_alt,
        (int)pkt.next_wp_alt,
        (int)pkt.nav_throttle,
        (int)pkt.angle_boost,
        (int)pkt.climb_rate,
        (int)pkt.throttle_out,
        (int)pkt.desired_climb_rate
    );
}

struct log_Iterm {
    LOG_PACKET_HEADER;
    int16_t rate_roll;
    int16_t rate_pitch;
    int16_t rate_yaw;
    int16_t accel_throttle;
    int16_t nav_lat;
    int16_t nav_lon;
    int16_t loiter_rate_lat;
    int16_t loiter_rate_lon;
    int16_t throttle_cruise;
};

static void Log_Write_Iterm()
{
    struct log_Iterm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        rate_roll       : (int16_t) g.pid_rate_roll.get_integrator(),
        rate_pitch      : (int16_t) g.pid_rate_pitch.get_integrator(),
        rate_yaw        : (int16_t) g.pid_rate_yaw.get_integrator(),
        accel_throttle  : (int16_t) g.pid_throttle_accel.get_integrator(),
        nav_lat         : (int16_t) g.pid_nav_lat.get_integrator(),
        nav_lon         : (int16_t) g.pid_nav_lon.get_integrator(),
        loiter_rate_lat : (int16_t) g.pid_loiter_rate_lat.get_integrator(),
        loiter_rate_lon : (int16_t) g.pid_loiter_rate_lon.get_integrator(),
        throttle_cruise : (int16_t) g.throttle_cruise
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read an control tuning packet
static void Log_Read_Iterm()
{
    struct log_Iterm pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                                1   2   3   4   5   6   7   8   9
    cliSerial->printf_P(PSTR("ITERM, %d, %d, %d, %d, %d, %d, %d, %d, %d\n"),
        (int)pkt.rate_roll,
        (int)pkt.rate_pitch,
        (int)pkt.rate_yaw,
        (int)pkt.accel_throttle,
        (int)pkt.nav_lat,
        (int)pkt.nav_lon,
        (int)pkt.loiter_rate_lat,
        (int)pkt.loiter_rate_lon,
        (int)pkt.throttle_cruise
    );
    cliSerial->printf_P(PSTR("ITERM, "));
}

struct log_Performance {
    LOG_PACKET_HEADER;
    uint8_t renorm_count;
    uint8_t renorm_blowup;
    uint8_t gps_fix_count;
    uint16_t num_long_running;
    uint16_t num_loops;
    uint32_t max_time;
    uint8_t end;
};

// Write a performance monitoring packet. Total length : 11 bytes
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        renorm_count     : ahrs.renorm_range_count,
        renorm_blowup    : ahrs.renorm_blowup_count,
        gps_fix_count    : gps_fix_count,
        num_long_running : perf_info_get_num_long_running(),
        num_loops        : perf_info_get_num_loops(),
        max_time         : perf_info_get_max_time()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a performance packet
static void Log_Read_Performance()
{
    struct log_Performance pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    //                            1   2   3   4   5    6
    cliSerial->printf_P(PSTR("PM, %u, %u, %u, %u, %u, %lu\n"),
                        (unsigned)pkt.renorm_count,
                        (unsigned)pkt.renorm_blowup,
                        (unsigned)pkt.gps_fix_count,
                        (unsigned)pkt.num_long_running,
                        (unsigned)pkt.num_loops,
                        (unsigned long)pkt.max_time);
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

// Write a command processing packet.  Total length : 21 bytes
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

    //                               1   2   3   4   5    6    7    8
    cliSerial->printf_P(PSTR( "CMD, %u, %u, %u, %u, %u, %ld, %ld, %ld\n"),
                    (unsigned)pkt.command_total,
                    (unsigned)pkt.command_number,
                    (unsigned)pkt.waypoint_id,
                    (unsigned)pkt.waypoint_options,
                    (unsigned)pkt.waypoint_param1,
                    (long)pkt.waypoint_altitude,
                    (long)pkt.waypoint_latitude,
                    (long)pkt.waypoint_longitude);
}

struct log_Attitude {
    LOG_PACKET_HEADER;
    int16_t roll_in;
    int16_t roll;
    int16_t pitch_in;
    int16_t pitch;
    int16_t yaw_in;
    uint16_t yaw;
    uint16_t nav_yaw;
};

// Write an attitude packet. Total length : 16 bytes
static void Log_Write_Attitude()
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        roll_in     : (int16_t)control_roll,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch_in    : (int16_t)control_pitch,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw_in      : (int16_t)g.rc_4.control_in,
        yaw         : (uint16_t)ahrs.yaw_sensor,
        nav_yaw     : (uint16_t)nav_yaw
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read an attitude packet
static void Log_Read_Attitude()
{
    struct log_Attitude pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                              1   2   3    4   5   6  7
    cliSerial->printf_P(PSTR("ATT, %d, %d, %d, %d, %d, %u, %u\n"),
                    (int)pkt.roll_in,
                    (int)pkt.roll,
                    (int)pkt.pitch_in,
                    (int)pkt.pitch,
                    (int)pkt.yaw_in,
                    (unsigned)pkt.yaw,
                    (unsigned)pkt.nav_yaw);
}

struct log_INAV {
    LOG_PACKET_HEADER;
    int16_t baro_alt;
    int16_t inav_alt;
    int16_t baro_climb_rate;
    int16_t inav_climb_rate;
    float   accel_corr_x;
    float   accel_corr_y;
    float   accel_corr_z;
    float   accel_corr_ef_z;
    int32_t gps_lat_from_home;
    int32_t gps_lon_from_home;
    float   inav_lat_from_home;
    float   inav_lon_from_home;
    float   inav_lat_speed;
    float   inav_lon_speed;
};

// Write an INAV packet. Total length : 52 Bytes
static void Log_Write_INAV()
{
#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
    Vector3f accel_corr = inertial_nav.accel_correction_ef;

    struct log_INAV pkt = {
        LOG_PACKET_HEADER_INIT(LOG_INAV_MSG),
        baro_alt            : (int16_t)baro_alt,                        // 1 barometer altitude
        inav_alt            : (int16_t)inertial_nav.get_altitude(),     // 2 accel + baro filtered altitude
        baro_climb_rate     : baro_rate,                                // 3 barometer based climb rate
        inav_climb_rate     : (int16_t)inertial_nav.get_velocity_z(),   // 4 accel + baro based climb rate
        accel_corr_x        : accel_corr.x,                             // 5 accel correction x-axis
        accel_corr_y        : accel_corr.y,                             // 6 accel correction y-axis
        accel_corr_z        : accel_corr.z,                             // 7 accel correction z-axis
        accel_corr_ef_z     : inertial_nav.accel_correction_ef.z,       // 8 accel correction earth frame
        gps_lat_from_home   : g_gps->latitude-home.lat,                 // 9 lat from home
        gps_lon_from_home   : g_gps->longitude-home.lng,                // 10 lon from home
        inav_lat_from_home  : inertial_nav.get_latitude_diff(),         // 11 accel based lat from home
        inav_lon_from_home  : inertial_nav.get_longitude_diff(),        // 12 accel based lon from home
        inav_lat_speed      : inertial_nav.get_latitude_velocity(),     // 13 accel based lat velocity
        inav_lon_speed      : inertial_nav.get_longitude_velocity()     // 14 accel based lon velocity
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#endif
}

// Read an INAV packet
static void Log_Read_INAV()
{
    struct log_INAV pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

                                  // 1   2   3   4      5      6      7      8    9   10     11     12     13     14
    cliSerial->printf_P(PSTR("INAV, %d, %d, %d, %d, %6.4f, %6.4f, %6.4f, %6.4f, %ld, %ld, %6.4f, %6.4f, %6.4f, %6.4f\n"),
                    (int)pkt.baro_alt,                  // 1 barometer altitude
                    (int)pkt.inav_alt,                  // 2 accel + baro filtered altitude
                    (int)pkt.baro_climb_rate,           // 3 barometer based climb rate
                    (int)pkt.inav_climb_rate,           // 4 accel + baro based climb rate
                    (float)pkt.accel_corr_x,            // 5 accel correction x-axis
                    (float)pkt.accel_corr_y,            // 6 accel correction y-axis
                    (float)pkt.accel_corr_z,            // 7 accel correction z-axis
                    (float)pkt.accel_corr_ef_z,         // 8 accel correction earth frame
                    (long)pkt.gps_lat_from_home,        // 9 lat from home
                    (long)pkt.gps_lon_from_home,        // 10 lon from home
                    (float)pkt.inav_lat_from_home,      // 11 accel based lat from home
                    (float)pkt.inav_lon_from_home,      // 12 accel based lon from home
                    (float)pkt.inav_lat_speed,          // 13 accel based lat velocity
                    (float)pkt.inav_lon_speed);         // 14 accel based lon velocity
}

struct log_Mode {
    LOG_PACKET_HEADER;
    uint8_t mode;
    int16_t throttle_cruise;
};

// Write a mode packet. Total length : 7 bytes
static void Log_Write_Mode(uint8_t mode)
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        mode            : mode,
        throttle_cruise : g.throttle_cruise,
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
    cliSerial->printf_P(PSTR(", %d\n"),(int)pkt.throttle_cruise);
}

struct log_Startup {
    LOG_PACKET_HEADER;
};

// Write Startup packet. Total length : 4 bytes
static void Log_Write_Startup()
{
    DataFlash.WriteByte(LOG_STARTUP_MSG);
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a startup packet
static void Log_Read_Startup()
{
    struct log_Startup pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("START UP\n"));
}

struct log_Event {
    LOG_PACKET_HEADER;
    uint8_t id;
};

// Wrote an event packet
static void Log_Write_Event(uint8_t id)
{
    if (g.log_bitmask != 0) {
        struct log_Event pkt = {
            LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
            id  : id
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

// Read an event packet
static void Log_Read_Event()
{
    struct log_Event pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("EV, %u\n"), (unsigned)pkt.id);
}

struct log_Data_Int8t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int8_t data_value;
};


// Write an int16_t data packet
static void Log_Write_Data(uint8_t id, int8_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int8t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT8_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

// Read an int16_t data packet
static void Log_Read_Int8t()
{
    struct log_Data_Int8t pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("DATA, %u, %d\n"), (unsigned)pkt.id, (int)pkt.data_value);
}

struct log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
static void Log_Write_Data(uint8_t id, int16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

// Read an int16_t data packet
static void Log_Read_Int16t()
{
    struct log_Data_Int16t pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("DATA, %u, %d\n"), (unsigned)pkt.id, (int)pkt.data_value);
}

struct log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
static void Log_Write_Data(uint8_t id, uint16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

// Read an uint16_t data packet
static void Log_Read_UInt16t()
{
    struct log_Data_UInt16t pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("DATA, %u, %u\n"), (unsigned)pkt.id, (unsigned)pkt.data_value);
}

struct log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
static void Log_Write_Data(uint8_t id, int32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

// Read an int32_t data packet
static void Log_Read_Int32t()
{
    struct log_Data_Int32t pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("DATA, %u, %ld\n"), (unsigned)pkt.id, (long)pkt.data_value);
}

struct log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
static void Log_Write_Data(uint8_t id, uint32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

// Read a uint32_t data packet
static void Log_Read_UInt32t()
{
    struct log_Data_UInt32t pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("DATA, %u, %lu\n"), (unsigned)pkt.id, (unsigned long)pkt.data_value);
}

struct log_Data_Float {
    LOG_PACKET_HEADER;
    uint8_t id;
    float data_value;
};

// Write a float data packet
static void Log_Write_Data(uint8_t id, float value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

// Read a float data packet
static void Log_Read_Float()
{
    struct log_Data_Float pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("DATA, %u, %1.6f\n"), (unsigned)pkt.id, (float)pkt.data_value);
}

struct log_PID {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t error;
    int32_t p;
    int32_t i;
    int32_t d;
    int32_t output;
    float  gain;
};

// Write an PID packet. Total length : 28 bytes
static void Log_Write_PID(uint8_t pid_id, int32_t error, int32_t p, int32_t i, int32_t d, int32_t output, float gain)
{
    struct log_PID pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PID_MSG),
        id      : pid_id,
        error   : error,
        p       : p,
        i       : i,
        d       : d,
        output  : output,
        gain    : gain
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a PID packet
static void Log_Read_PID()
{
    struct log_PID pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                             1    2    3    4    5    6      7
    cliSerial->printf_P(PSTR("PID-%u, %ld, %ld, %ld, %ld, %ld, %4.4f\n"),
                    (unsigned)pkt.id,
                    (long)pkt.error,
                    (long)pkt.p,
                    (long)pkt.i,
                    (long)pkt.d,
                    (long)pkt.output,
                    (float)pkt.gain);
}

struct log_DMP {
    LOG_PACKET_HEADER;
    int16_t  dcm_roll;
    int16_t  dmp_roll;
    int16_t  dcm_pitch;
    int16_t  dmp_pitch;
    uint16_t dcm_yaw;
    uint16_t dmp_yaw;
};

// Write a DMP attitude packet. Total length : 16 bytes
static void Log_Write_DMP()
{
#if SECONDARY_DMP_ENABLED == ENABLED
    struct log_DMP pkt = {
        LOG_PACKET_HEADER_INIT(LOG_DMP_MSG),
        dcm_roll    : (int16_t)ahrs.roll_sensor,
        dmp_roll    : (int16_t)ahrs2.roll_sensor,
        dcm_pitch   : (int16_t)ahrs.pitch_sensor,
        dmp_pitch   : (int16_t)ahrs2.pitch_sensor,
        dcm_yaw     : (uint16_t)ahrs.yaw_sensor,
        dmp_yaw     : (uint16_t)ahrs2.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#endif
}

// Read a DMP attitude packet
static void Log_Read_DMP()
{
    struct log_DMP pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

                                 // 1   2   3   4   5   6
    cliSerial->printf_P(PSTR("DMP, %d, %d, %d, %d, %u, %u\n"),
                    (int)pkt.dcm_roll,
                    (int)pkt.dmp_roll,
                    (int)pkt.dcm_pitch,
                    (int)pkt.dmp_pitch,
                    (unsigned)pkt.dcm_yaw,
                    (unsigned)pkt.dmp_yaw);
}

struct log_Camera {
    LOG_PACKET_HEADER;
    uint32_t gps_time;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
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
        gps_time    : g_gps->time,
        latitude    : current_loc.lat,
        longitude   : current_loc.lng,
        altitude    : current_loc.alt,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#endif
}

// Read a camera packet
static void Log_Read_Camera()
{
    struct log_Camera pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
                                     // 1
    cliSerial->printf_P(PSTR("CAMERA, %lu, "),(unsigned long)pkt.gps_time); // 1 time
    print_latlon(cliSerial, pkt.latitude);              // 2 lat
    cliSerial->print_P(PSTR(", "));
    print_latlon(cliSerial, pkt.longitude);             // 3 lon
                               // 4   5   6   7
    cliSerial->printf_P(PSTR(", %ld, %d, %d, %u\n"),
                    (long)pkt.altitude,                 // 4 altitude
                    (int)pkt.roll,                      // 5 roll in centidegrees
                    (int)pkt.pitch,                     // 6 pitch in centidegrees
                    (unsigned)pkt.yaw);                 // 7 yaw in centidegrees
}

struct log_Error {
    LOG_PACKET_HEADER;
    uint8_t sub_system;
    uint8_t error_code;
};

// Write an error packet. Total length : 5 bytes
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        sub_system    : sub_system,
        error_code    : error_code,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read an error packet
static void Log_Read_Error()
{
    struct log_Error pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    cliSerial->print_P(PSTR("ERR, "));

    // print subsystem
    switch(pkt.sub_system) {
        case ERROR_SUBSYSTEM_MAIN:
            cliSerial->print_P(PSTR("MAIN"));
            break;
        case ERROR_SUBSYSTEM_RADIO:
            cliSerial->print_P(PSTR("RADIO"));
            break;
        case ERROR_SUBSYSTEM_COMPASS:
            cliSerial->print_P(PSTR("COM"));
            break;
        case ERROR_SUBSYSTEM_OPTFLOW:
            cliSerial->print_P(PSTR("OF"));
            break;
        case ERROR_SUBSYSTEM_FAILSAFE:
            cliSerial->print_P(PSTR("FS"));
            break;
        default:
            // if undefined print subsytem as a number
            cliSerial->printf_P(PSTR("%u"),(unsigned)pkt.sub_system);
            break;
    }

    // print error code
    cliSerial->printf_P(PSTR(", %u\n"),(unsigned)pkt.error_code);
}


// Read the DataFlash log memory
static void Log_Read(int16_t start_page, int16_t end_page)
{
 #ifdef AIRFRAME_NAME
    cliSerial->printf_P(PSTR((AIRFRAME_NAME)));
 #endif

    cliSerial->printf_P(PSTR("\n" THISFIRMWARE
                             "\nFree RAM: %u\n"),
                        (unsigned) memcheck_available_memory());

 #if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    cliSerial->printf_P(PSTR("APM 1\n"));
 #elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
    cliSerial->printf_P(PSTR("APM 2\n"));
 #endif

#if CLI_ENABLED == ENABLED
	setup_show(0, NULL);
#endif

    DataFlash.log_read_process(start_page, end_page, log_callback);
}

// read one packet from the dataflash
static void log_callback(uint8_t msgid)
{
    switch(msgid) {
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
        
    case LOG_MOTORS_MSG:
        Log_Read_Motors();
        break;
        
    case LOG_OPTFLOW_MSG:
        Log_Read_Optflow();
        break;
        
    case LOG_GPS_MSG:
        Log_Read_GPS();
        break;
        
    case LOG_EVENT_MSG:
        Log_Read_Event();
        break;
        
    case LOG_PID_MSG:
        Log_Read_PID();
        break;
        
    case LOG_ITERM_MSG:
        Log_Read_Iterm();
        break;
        
    case LOG_DMP_MSG:
        Log_Read_DMP();
        break;
        
    case LOG_INAV_MSG:
        Log_Read_INAV();
        break;
        
    case LOG_CAMERA_MSG:
        Log_Read_Camera();
        break;
        
    case LOG_ERROR_MSG:
        Log_Read_Error();
        break;
        
    case LOG_DATA_INT16_MSG:
        Log_Read_Int16t();
        break;
        
    case LOG_DATA_UINT16_MSG:
        Log_Read_UInt16t();
        break;

    case LOG_DATA_INT32_MSG:
        Log_Read_Int32t();
        break;
        
    case LOG_DATA_UINT32_MSG:
        Log_Read_UInt32t();
        break;
        
    case LOG_DATA_FLOAT_MSG:
        Log_Read_Float();
        break;
    }
}


#else // LOGGING_ENABLED

void print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon) {}
static void Log_Write_Startup() {}
static void Log_Write_Cmd(uint8_t num, struct Location *wp) {}
static void Log_Write_Mode(uint8_t mode) {}
static void Log_Write_IMU() {}
static void Log_Write_GPS() {}
static void Log_Write_Current() {}
static void Log_Write_Iterm() {}
static void Log_Write_Attitude() {}
static void Log_Write_INAV() {}
static void Log_Write_Data(uint8_t id, int8_t value){}
static void Log_Write_Data(uint8_t id, int16_t value){}
static void Log_Write_Data(uint8_t id, uint16_t value){}
static void Log_Write_Data(uint8_t id, int32_t value){}
static void Log_Write_Data(uint8_t id, uint32_t value){}
static void Log_Write_Data(uint8_t id, float value){}
static void Log_Write_Event(uint8_t id){}
static void Log_Write_Optflow() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_Motors() {}
static void Log_Write_Performance() {}
static void Log_Write_PID(uint8_t pid_id, int32_t error, int32_t p, int32_t i, int32_t d, int32_t output, float gain) {}
static void Log_Write_DMP() {}
static void Log_Write_Camera() {}
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}

#endif // LOGGING_DISABLED
#line 1 "./Firmware/ACopter32/Parameters.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
 *  ArduCopter parameter definitions
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
    // @Param: SYSID_SW_MREV
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version, "SYSID_SW_MREV",   0),

    // @Param: SYSID_SW_TYPE
    // @DisplayName: Software Type
    // @Description: This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
    // @User: Advanced
    GSCALAR(software_type,  "SYSID_SW_TYPE",   Parameters::k_software_type),

    // @Param: SYSID_THISMAV
    // @DisplayName: Mavlink version
    // @Description: Allows reconising the mavlink version
    // @User: Advanced
    GSCALAR(sysid_this_mav, "SYSID_THISMAV",   MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: My ground station number
    // @Description: Allows restricting radio overrides to only come from my ground station
    // @User: Advanced
    GSCALAR(sysid_my_gcs,   "SYSID_MYGCS",     255),

    // @Param: SERIAL3_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the telemetry port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
    GSCALAR(serial3_baud,   "SERIAL3_BAUD",     SERIAL3_BAUD/1000),

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Standard
    // @Units: seconds
    // @Range: 0 10
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: RTL_ALT
    // @DisplayName: RTL Altitude
    // @Description: The minimum altitude the model will move to before Returning to Launch.  Set to zero to return at current altitude.
    // @Units: Centimeters
    // @Range: 0 4000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_altitude,   "RTL_ALT",     RTL_ALT),

    // @Param: SONAR_ENABLE
    // @DisplayName: Enable Sonar
    // @Description: Setting this to Enabled(1) will enable the sonar. Setting this to Disabled(0) will disable the sonar
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(sonar_enabled,  "SONAR_ENABLE",     DISABLED),

    // @Param: SONAR_TYPE
    // @DisplayName: Sonar type
    // @Description: Used to adjust scaling to match the sonar used (only Maxbotix sonars are supported at this time)
    // @Values: 0:XL-EZ0,1:LV-EZ0,2:XLL-EZ0,3:HRLV
    // @User: Standard
    GSCALAR(sonar_type,     "SONAR_TYPE",           AP_RANGEFINDER_MAXSONARXL),

    // @Param: BATT_MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Voltage Only,4:Voltage and Current
    // @User: Standard
    GSCALAR(battery_monitoring, "BATT_MONITOR", DISABLED),

    // @Param: FS_BATT_ENABLE
    // @DisplayName: Battery Failsafe Enable
    // @Description: Controls whether failsafe will be invoked when battery voltage or current runs low
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(failsafe_battery_enabled, "FS_BATT_ENABLE", FS_BATTERY),

    // @Param: VOLT_DIVIDER
    // @DisplayName: Voltage Divider
    // @Description: Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin voltage * INPUT_VOLTS/1024 * VOLT_DIVIDER)
    // @User: Advanced
    GSCALAR(volt_div_ratio, "VOLT_DIVIDER",     VOLT_DIV_RATIO),

    // @Param: AMP_PER_VOLT
    // @DisplayName: Current Amps per volt
    // @Description: Used to convert the voltage on the current sensing pin (BATT_CURR_PIN) to the actual current being consumed in amps (curr pin voltage * INPUT_VOLTS/1024 * AMP_PER_VOLT )
    // @User: Advanced
    GSCALAR(curr_amp_per_volt,      "AMP_PER_VOLT", CURR_AMP_PER_VOLT),

    // @Param: INPUT_VOLTS
    // @DisplayName: Max internal voltage of the battery voltage and current sensing pins
    // @Description: Used to convert the voltage read in on the voltage and current pins for battery monitoring.  Normally 5 meaning 5 volts.
    // @User: Advanced
    GSCALAR(input_voltage,  "INPUT_VOLTS",      INPUT_VOLTAGE),

    // @Param: BATT_CAPACITY
    // @DisplayName: Battery Capacity
    // @Description: Battery capacity in milliamp-hours (mAh)
    // @Units: mAh
    GSCALAR(pack_capacity,  "BATT_CAPACITY",    HIGH_DISCHARGE),

    // @Param: MAG_ENABLE
    // @DisplayName: Enable Compass
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(compass_enabled,        "MAG_ENABLE",   MAGNETOMETER),

    // @Param: FLOW_ENABLE
    // @DisplayName: Enable Optical Flow
    // @Description: Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(optflow_enabled,        "FLOW_ENABLE",  DISABLED),

    // @Param: LOW_VOLT
    // @DisplayName: Low Voltage
    // @Description: Set this to the voltage you want to represent low voltage
    // @Range: 0 20
    // @Increment: .1
    // @User: Standard
    GSCALAR(low_voltage,    "LOW_VOLT",         LOW_VOLTAGE),

    // @Param: SUPER_SIMPLE
    // @DisplayName: Enable Super Simple Mode
    // @Description: Setting this to Enabled(1) will enable Super Simple Mode. Setting this to Disabled(0) will disable Super Simple Mode
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(super_simple,   "SUPER_SIMPLE",     SUPER_SIMPLE),

    // @Param: RTL_ALT_FINAL
    // @DisplayName: RTL Final Altitude
    // @Description: This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to zero to land.
    // @Units: Centimeters
    // @Range: -1 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_alt_final,  "RTL_ALT_FINAL", RTL_ALT_FINAL),

	// @Param: TILT
    // @DisplayName: Auto Tilt Compensation
    // @Description: This is a feed-forward compensation which helps the aircraft achieve target waypoint speed.
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
	GSCALAR(tilt_comp,      "TILT",     TILT_COMPENSATION),

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
    // @Values: -1:Disabled, 0:A0, 1:A1, 2:A2, 13:A13
    // @User: Standard
    GSCALAR(rssi_pin,            "RSSI_PIN",         -1),

    // @Param: THR_ACC_ENABLE
    // @DisplayName: Enable Accel based throttle controller
    // @Description: This allows enabling and disabling the accelerometer based throttle controller.  If disabled a velocity based controller is used.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(throttle_accel_enabled,  "THR_ACC_ENABLE",   1),

    // @Param: YAW_OVR_BEHAVE
    // @DisplayName: Yaw override behaviour
    // @Description: Controls when autopilot takes back normal control of yaw after pilot overrides
    // @Values: 0:At Next WP, 1:On Mission Restart
    // @User: Advanced
    GSCALAR(yaw_override_behaviour,  "YAW_OVR_BEHAVE",   YAW_OVERRIDE_BEHAVIOUR_AT_NEXT_WAYPOINT),

    // @Param: WP_TOTAL
    // @DisplayName: Waypoint Total
    // @Description: Total number of commands in the mission stored in the eeprom.  Do not update this parameter directly!
    // @User: Advanced
    GSCALAR(command_total,  "WP_TOTAL",         0),

    // @Param: WP_INDEX
    // @DisplayName: Waypoint Index
    // @Description: The index number of the command that is currently being executed.  Do not update this parameter directly!
    // @User: Advanced
    GSCALAR(command_index,  "WP_INDEX",         0),

    // @Param: WP_RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: Meters
    // @Range: 1 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_radius,        "WP_RADIUS",    WP_RADIUS_DEFAULT),

    // @Param: CIRCLE_RADIUS
    // @DisplayName: Circle radius
    // @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
    // @Units: Meters
    // @Range: 1 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(circle_radius,  "CIRCLE_RADIUS",    CIRCLE_RADIUS),

	// @Param: WP_SPEED_MAX
    // @DisplayName: Waypoint Max Speed Target
    // @Description: Defines the speed which the aircraft will attempt to maintain during a WP mission.
    // @Units: Centimeters/Second
    // @Increment: 100
    // @User: Standard
    GSCALAR(waypoint_speed_max,     "WP_SPEED_MAX", WAYPOINT_SPEED_MAX),

	// @Param: XTRK_GAIN_SC
    // @DisplayName: Cross-Track Gain
    // @Description: This controls the rate that the Auto Controller will attempt to return original track
    // @Units: Dimensionless
	// @User: Standard
    GSCALAR(crosstrack_gain,        "XTRK_GAIN_SC", CROSSTRACK_GAIN),

    // @Param: XTRK_MIN_DIST
    // @DisplayName: Crosstrack mininum distance
    // @Description: Minimum distance in meters between waypoints to do crosstrack correction.
    // @Units: Meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(crosstrack_min_distance, "XTRK_MIN_DIST",  CROSSTRACK_MIN_DISTANCE),

    // @Param: RTL_LOIT_TIME
    // @DisplayName: RTL loiter time
    // @Description: Time (in milliseconds) to loiter above home before begining final descent
    // @Units: ms
    // @Range: 0 60000
    // @Increment: 1000
    // @User: Standard
    GSCALAR(rtl_loiter_time,      "RTL_LOIT_TIME",    RTL_LOITER_TIME),

    // @Param: LAND_SPEED
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in cm/s
    // @Units: Centimeters/Second
    // @Range: 10 200
    // @Increment: 10
    // @User: Standard
    GSCALAR(land_speed,             "LAND_SPEED",   LAND_SPEED),

    // @Param: AUTO_VELZ_MIN
    // @DisplayName: Autopilot's min vertical speed (max descent) in cm/s
    // @Description: The minimum vertical velocity (i.e. descent speed) the autopilot may request in cm/s
    // @Units: Centimeters/Second
    // @Range: -500 -50
    // @Increment: 10
    // @User: Standard
    GSCALAR(auto_velocity_z_min,     "AUTO_VELZ_MIN",   AUTO_VELZ_MIN),

    // @Param: AUTO_VELZ_MAX
    // @DisplayName: Auto pilot's max vertical speed in cm/s
    // @Description: The maximum vertical velocity the autopilot may request in cm/s
    // @Units: Centimeters/Second
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(auto_velocity_z_max,     "AUTO_VELZ_MAX",   AUTO_VELZ_MAX),

    // @Param: PILOT_VELZ_MAX
    // @DisplayName: Pilot maximum vertical speed
    // @Description: The maximum vertical velocity the pilot may request in cm/s
    // @Units: Centimeters/Second
    // @Range: 10 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_velocity_z_max,     "PILOT_VELZ_MAX",   PILOT_VELZ_MAX),

    // @Param: THR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: The minimum throttle that will be sent to the motors to keep them spinning
    // @Units: ms
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_min,   "THR_MIN",          MINIMUM_THROTTLE),

    // @Param: THR_MAX
    // @DisplayName: Maximum Throttle
    // @Description: The maximum throttle that will be sent to the motors
    // @Units: ms
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_max,   "THR_MAX",          MAXIMUM_THROTTLE),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values: 0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode
    // @User: Standard
    GSCALAR(failsafe_throttle,  "FS_THR_ENABLE",   FS_THR_DISABLED),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers
    // @User: Standard
    GSCALAR(failsafe_throttle_value, "FS_THR_VALUE",      FS_THR_VALUE_DEFAULT),

    // @Param: TRIM_THROTTLE
    // @DisplayName: Throttle Trim
    // @Description: The autopilot's estimate of the throttle required to maintain a level hover.  Calculated automatically from the pilot's throttle input while in stabilize mode
    // @Range: 0 1000
    // @Units: PWM
    // @User: Standard
    GSCALAR(throttle_cruise,        "TRIM_THROTTLE",    THROTTLE_CRUISE),

    // @Param: THR_MID
    // @DisplayName: Throttle Mid Position
    // @Description: The throttle output (0 ~ 1000) when throttle stick is in mid position.  Used to scale the manual throttle so that the mid throttle stick position is close to the throttle required to hover
    // @User: Standard
    // @Range: 300 700
    // @Increment: 1
    GSCALAR(throttle_mid,        "THR_MID",    THR_MID),

    // @Param: FLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when Channel 5 pwm is <= 1230
    // @User: Standard
    GSCALAR(flight_mode1, "FLTMODE1",               FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when Channel 5 pwm is >1230, <= 1360
    // @User: Standard
    GSCALAR(flight_mode2, "FLTMODE2",               FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when Channel 5 pwm is >1360, <= 1490
    // @User: Standard
    GSCALAR(flight_mode3, "FLTMODE3",               FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when Channel 5 pwm is >1490, <= 1620
    // @User: Standard
    GSCALAR(flight_mode4, "FLTMODE4",               FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when Channel 5 pwm is >1620, <= 1749
    // @User: Standard
    GSCALAR(flight_mode5, "FLTMODE5",               FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when Channel 5 pwm is >=1750
    // @User: Standard
    GSCALAR(flight_mode6, "FLTMODE6",               FLIGHT_MODE_6),

    // @Param: SIMPLE
    // @DisplayName: Simple mode bitmask
    // @Description: Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode)
    // @User: Advanced
    GSCALAR(simple_modes, "SIMPLE",                 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 2 byte bitmap of log types to enable
    // @User: Advanced
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // @Param: TOY_RATE
    // @DisplayName: Toy Yaw Rate
    // @Description: Controls yaw rate in Toy mode.  Higher values will cause a slower yaw rate.  Do not set to zero!
    // @User: Advanced
    // @Range: 1 10
    GSCALAR(toy_yaw_rate, "TOY_RATE",               1),

    // @Param: ESC
    // @DisplayName: ESC Calibration
    // @Description: Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.
    // @User: Advanced
    // @Values: 0:Normal Start-up,1:Start-up in ESC Calibration mode
    GSCALAR(esc_calibrate, "ESC",                   0),

    // @Param: TUNE
    // @DisplayName: Channel 6 Tuning
    // @Description: Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob
    // @User: Standard
    // @Values: 0:CH6_NONE,1:CH6_STABILIZE_KP,2:CH6_STABILIZE_KI,3:CH6_YAW_KP,4:CH6_RATE_KP,5:CH6_RATE_KI,6:CH6_YAW_RATE_KP,7:CH6_THROTTLE_KP,8:CH6_TOP_BOTTOM_RATIO,9:CH6_RELAY,10:CH6_TRAVERSE_SPEED,11:CH6_NAV_KP,12:CH6_LOITER_KP,13:CH6_HELI_EXTERNAL_GYRO,14:CH6_THR_HOLD_KP,17:CH6_OPTFLOW_KP,18:CH6_OPTFLOW_KI,19:CH6_OPTFLOW_KD,20:CH6_NAV_KI,21:CH6_RATE_KD,22:CH6_LOITER_RATE_KP,23:CH6_LOITER_RATE_KD,24:CH6_YAW_KI,25:CH6_ACRO_KP,26:CH6_YAW_RATE_KD,27:CH6_LOITER_KI,28:CH6_LOITER_RATE_KI,29:CH6_STABILIZE_KD,30:CH6_AHRS_YAW_KP,31:CH6_AHRS_KP,32:CH6_INAV_TC,33:CH6_THROTTLE_KI,34:CH6_THR_ACCEL_KP,35:CH6_THR_ACCEL_KI,36:CH6_THR_ACCEL_KD
    GSCALAR(radio_tuning, "TUNE",                   0),

    // @Param: TUNE_LOW
    // @DisplayName: Tuning minimum
    // @Description: The minimum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(radio_tuning_low, "TUNE_LOW",           0),

    // @Param: TUNE_HIGH
    // @DisplayName: Tuning maximum
    // @Description: The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(radio_tuning_high, "TUNE_HIGH",         1000),

    // @Param: FRAME
    // @DisplayName: Frame Orientation (+, X or V)
    // @Description: Controls motor mixing for multicopters.  Not used for Tri or Traditional Helicopters.
    // @Values: 0:Plus, 1:X, 2:V
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(frame_orientation, "FRAME",             FRAME_ORIENTATION),

    // @Param: CH7_OPT
    // @DisplayName: Channel 7 option
    // @Description: Select which function if performed when CH7 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:Sonar
    // @User: Standard
    GSCALAR(ch7_option, "CH7_OPT",                  CH7_OPTION),

	// @Param: AUTO_SLEW
    // @DisplayName: Auto Slew Rate
    // @Description: This restricts the rate of change of the roll and pitch attitude commanded by the auto pilot
    // @Units: Degrees/Second
	// @Range: 1 45
    // @Increment: 1
    // @User: Advanced
    GSCALAR(auto_slew_rate, "AUTO_SLEW",            AUTO_SLEW_RATE),

#if FRAME_CONFIG ==     HELI_FRAME
    GGROUP(heli_servo_1,    "HS1_", RC_Channel),
    GGROUP(heli_servo_2,    "HS2_", RC_Channel),
    GGROUP(heli_servo_3,    "HS3_", RC_Channel),
    GGROUP(heli_servo_4,    "HS4_", RC_Channel),
	GSCALAR(heli_pitch_ff, "RATE_PIT_FF",            HELI_PITCH_FF),
	GSCALAR(heli_roll_ff, "RATE_RLL_FF",            HELI_ROLL_FF),
	GSCALAR(heli_yaw_ff, "RATE_YAW_FF",            HELI_YAW_FF),
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
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_5,    "RC5_", RC_Channel_aux),
    // @Group: RC6_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_6,    "RC6_", RC_Channel_aux),
    // @Group: RC7_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_7,    "RC7_", RC_Channel_aux),
    // @Group: RC8_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_8,    "RC8_", RC_Channel_aux),

#if MOUNT == ENABLED
    // @Group: RC10_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_10,                    "RC10_", RC_Channel_aux),

    // @Group: RC11_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_11,                    "RC11_", RC_Channel_aux),
#endif

    // @Param: RC_SPEED
    // @DisplayName: ESC Update Speed
    // @Description: This is the speed in Hertz that your ESCs will receive updates
    // @Units: Hertz (Hz)
    // @Values: 125,400,490
    // @User: Advanced
    GSCALAR(rc_speed, "RC_SPEED",              RC_FAST_SPEED),

    // @Param: ACRO_P
    // @DisplayName: Acro P gain
    // @Description: Used to convert pilot roll, pitch and yaw input into a dssired rate of rotation in ACRO mode.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard
    GSCALAR(acro_p,                 "ACRO_P",           ACRO_P),

    // @Param: AXIS_ENABLE
    // @DisplayName: Acro Axis
    // @Description: Used to control whether acro mode actively maintains the current angle when control sticks are released (Enabled = maintains current angle)
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(axis_enabled,           "AXIS_ENABLE",      AXIS_LOCK_ENABLED),

    // @Param: ACRO_BAL_ROLL
    // @DisplayName: Acro Balance Roll
    // @Description: rate at which roll angle returns to level in acro mode
    // @Range: 0 300
    // @Increment: 1
    // @User: Advanced
    GSCALAR(acro_balance_roll,      "ACRO_BAL_ROLL",    ACRO_BALANCE_ROLL),

    // @Param: ACRO_BAL_PITCH
    // @DisplayName: Acro Balance Pitch
    // @Description: rate at which pitch angle returns to level in acro mode
    // @Range: 0 300
    // @Increment: 1
    // @User: Advanced
    GSCALAR(acro_balance_pitch,     "ACRO_BAL_PITCH",   ACRO_BALANCE_PITCH),

    // @Param: ACRO_TRAINER
    // @DisplayName: Acro Trainer Enabled
    // @Description: Set to 1 (Enabled) to make roll return to within 45 degrees of level automatically
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(acro_trainer_enabled,   "ACRO_TRAINER",     ACRO_TRAINER_ENABLED),

    // @Param: LED_MODE
    // @DisplayName: Copter LED Mode
    // @Description: bitmap to control the copter led mode
    // @Values: 0:Disabled,1:Enable,2:GPS On,4:Aux,8:Buzzer,16:Oscillate,32:Nav Blink,64:GPS Nav Blink
    // @User: Standard
    GSCALAR(copter_leds_mode,       "LED_MODE",         9),

    // PID controller
    //---------------
    // @Param: RATE_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.08 0.20
    // @User: Standard

    // @Param: RATE_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 0.5
    // @User: Standard

    // @Param: RATE_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 500
    // @Unit: PWM
    // @User: Standard

    // @Param: RATE_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.001 0.008
    // @User: Standard
    GGROUP(pid_rate_roll,     "RATE_RLL_", AC_PID),

    // @Param: RATE_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.08 0.20
    // @User: Standard

    // @Param: RATE_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 0.5
    // @User: Standard

    // @Param: RATE_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 500
    // @Unit: PWM
    // @User: Standard

    // @Param: RATE_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.001 0.008
    // @User: Standard
    GGROUP(pid_rate_pitch,    "RATE_PIT_", AC_PID),

    // @Param: RATE_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.150 0.250
    // @User: Standard

    // @Param: RATE_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 0.020
    // @User: Standard

    // @Param: RATE_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 500
    // @Unit: PWM
    // @User: Standard

    // @Param: RATE_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.001
    // @User: Standard
    GGROUP(pid_rate_yaw,      "RATE_YAW_", AC_PID),

    // @Param: LOITER_LAT_P
    // @DisplayName: Loiter latitude rate controller P gain
    // @Description: Loiter latitude rate controller P gain.  Converts the difference between desired speed and actual speed into a lean angle in the latitude direction
    // @Range: 2.000 6.000
    // @User: Standard

    // @Param: LOITER_LAT_I
    // @DisplayName: Loiter latitude rate controller I gain
    // @Description: Loiter latitude rate controller I gain.  Corrects long-term difference in desired speed and actual speed in the latitude direction
    // @Range: 0.020 0.060
    // @User: Standard

    // @Param: LOITER_LAT_IMAX
    // @DisplayName: Loiter rate controller I gain maximum
    // @Description: Loiter rate controller I gain maximum.  Constrains the lean angle that the I gain will output
    // @Range: 0 4500
    // @Unit: Centi-Degrees
    // @User: Standard

    // @Param: LOITER_LAT_D
    // @DisplayName: Loiter latitude rate controller D gain
    // @Description: Loiter latitude rate controller D gain.  Compensates for short-term change in desired speed vs actual speed
    // @Range: 0.200 0.600
    // @User: Standard
    GGROUP(pid_loiter_rate_lat,      "LOITER_LAT_",  AC_PID),

    // @Param: LOITER_LON_P
    // @DisplayName: Loiter longitude rate controller P gain
    // @Description: Loiter longitude rate controller P gain.  Converts the difference between desired speed and actual speed into a lean angle in the longitude direction
    // @Range: 2.000 6.000
    // @User: Standard

    // @Param: LOITER_LON_I
    // @DisplayName: Loiter longitude rate controller I gain
    // @Description: Loiter longitude rate controller I gain.  Corrects long-term difference in desired speed and actual speed in the longitude direction
    // @Range: 0.020 0.060
    // @User: Standard

    // @Param: LOITER_LON_IMAX
    // @DisplayName: Loiter longitude rate controller I gain maximum
    // @Description: Loiter longitude rate controller I gain maximum.  Constrains the lean angle that the I gain will output
    // @Range: 0 4500
    // @Unit: Centi-Degrees
    // @User: Standard

    // @Param: LOITER_LON_D
    // @DisplayName: Loiter longituderate controller D gain
    // @Description: Loiter longitude rate controller D gain.  Compensates for short-term change in desired speed vs actual speed
    // @Range: 0.200 0.600
    // @User: Standard
    GGROUP(pid_loiter_rate_lon,      "LOITER_LON_",  AC_PID),

    // @Param: NAV_LAT_P
    // @DisplayName: Navigation latitude rate controller P gain
    // @Description: Navigation latitude rate controller P gain.  Converts the difference between desired speed and actual speed into a lean angle in the latitude direction
    // @Range: 2.000 2.800
    // @User: Standard

    // @Param: NAV_LAT_I
    // @DisplayName: Navigation latitude rate controller I gain
    // @Description: Navigation latitude rate controller I gain.  Corrects long-term difference in desired speed and actual speed in the latitude direction
    // @Range: 0.140 0.200
    // @User: Standard

    // @Param: NAV_LAT_IMAX
    // @DisplayName: Navigation rate controller I gain maximum
    // @Description: Navigation rate controller I gain maximum.  Constrains the lean angle that the I gain will output
    // @Range: 0 4500
    // @Unit: Centi-Degrees
    // @User: Standard

    // @Param: NAV_LAT_D
    // @DisplayName: Navigation latitude rate controller D gain
    // @Description: Navigation latitude rate controller D gain.  Compensates for short-term change in desired speed vs actual speed
    // @Range: 0.000 0.100
    // @User: Standard
    GGROUP(pid_nav_lat,             "NAV_LAT_",  AC_PID),

    // @Param: NAV_LON_P
    // @DisplayName: Navigation longitude rate controller P gain
    // @Description: Navigation longitude rate controller P gain.  Converts the difference between desired speed and actual speed into a lean angle in the longitude direction
    // @Range: 2.000 2.800
    // @User: Standard

    // @Param: NAV_LON_I
    // @DisplayName: Navigation longitude rate controller I gain
    // @Description: Navigation longitude rate controller I gain.  Corrects long-term difference in desired speed and actual speed in the longitude direction
    // @Range: 0.140 0.200
    // @User: Standard

    // @Param: NAV_LON_IMAX
    // @DisplayName: Navigation longitude rate controller I gain maximum
    // @Description: Navigation longitude rate controller I gain maximum.  Constrains the lean angle that the I gain will generate
    // @Range: 0 4500
    // @Unit: Centi-Degrees
    // @User: Standard

    // @Param: NAV_LON_D
    // @DisplayName: Navigation longituderate controller D gain
    // @Description: Navigation longitude rate controller D gain.  Compensates for short-term change in desired speed vs actual speed
    // @Range: 0.000 0.100
    // @User: Standard
    GGROUP(pid_nav_lon,             "NAV_LON_",  AC_PID),

    // @Param: THR_RATE_P
    // @DisplayName: Throttle rate controller P gain
    // @Description: Throttle rate controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard

    // @Param: THR_RATE_I
    // @DisplayName: Throttle rate controller I gain
    // @Description: Throttle rate controller I gain.  Corrects long-term difference in desired vertical speed and actual speed
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: THR_RATE_IMAX
    // @DisplayName: Throttle rate controller I gain maximum
    // @Description: Throttle rate controller I gain maximum.  Constrains the desired acceleration that the I gain will generate
    // @Range: 0 500
    // @Unit: cm/s/s
    // @User: Standard

    // @Param: THR_RATE_D
    // @DisplayName: Throttle rate controller D gain
    // @Description: Throttle rate controller D gain.  Compensates for short-term change in desired vertical speed vs actual speed
    // @Range: 0.000 0.400
    // @User: Standard
    GGROUP(pid_throttle,      "THR_RATE_", AC_PID),

    // @Param: THR_ACCEL_P
    // @DisplayName: Throttle acceleration controller P gain
    // @Description: Throttle acceleration controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @User: Standard

    // @Param: THR_ACCEL_I
    // @DisplayName: Throttle acceleration controller I gain
    // @Description: Throttle acceleration controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: THR_ACCEL_IMAX
    // @DisplayName: Throttle acceleration controller I gain maximum
    // @Description: Throttle acceleration controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 500
    // @Unit: PWM
    // @User: Standard

    // @Param: THR_ACCEL_D
    // @DisplayName: Throttle acceleration controller D gain
    // @Description: Throttle acceleration controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard
    GGROUP(pid_throttle_accel,"THR_ACCEL_", AC_PID),

    // @Param: OF_RLL_P
    // @DisplayName: Optical Flow based loiter controller roll axis P gain
    // @Description: Optical Flow based loiter controller roll axis P gain.  Converts the position error from the target point to a roll angle
    // @Range: 2.000 3.000
    // @User: Standard

    // @Param: OF_RLL_I
    // @DisplayName: Optical Flow based loiter controller roll axis I gain
    // @Description: Optical Flow based loiter controller roll axis I gain.  Corrects long-term position error by more persistently rolling left or right
    // @Range: 0.250 0.750
    // @User: Standard

    // @Param: OF_RLL_IMAX
    // @DisplayName: Optical Flow based loiter controller roll axis I gain maximum
    // @Description: Optical Flow based loiter controller roll axis I gain maximum.  Constrains the maximum roll angle that the I term will generate
    // @Range: 0 4500
    // @Unit: Centi-Degrees
    // @User: Standard

    // @Param: OF_RLL_D
    // @DisplayName: Optical Flow based loiter controller roll axis D gain
    // @Description: Optical Flow based loiter controller roll axis D gain.  Compensates for short-term change in speed in the roll direction
    // @Range: 0.100 0.140
    // @User: Standard
    GGROUP(pid_optflow_roll,  "OF_RLL_",   AC_PID),

    // @Param: OF_PIT_P
    // @DisplayName: Optical Flow based loiter controller pitch axis P gain
    // @Description: Optical Flow based loiter controller pitch axis P gain.  Converts the position error from the target point to a pitch angle
    // @Range: 2.000 3.000
    // @User: Standard

    // @Param: OF_PIT_I
    // @DisplayName: Optical Flow based loiter controller pitch axis I gain
    // @Description: Optical Flow based loiter controller pitch axis I gain.  Corrects long-term position error by more persistently pitching left or right
    // @Range: 0.250 0.750
    // @User: Standard

    // @Param: OF_PIT_IMAX
    // @DisplayName: Optical Flow based loiter controller pitch axis I gain maximum
    // @Description: Optical Flow based loiter controller pitch axis I gain maximum.  Constrains the maximum pitch angle that the I term will generate
    // @Range: 0 4500
    // @Unit: Centi-Degrees
    // @User: Standard

    // @Param: OF_PIT_D
    // @DisplayName: Optical Flow based loiter controller pitch axis D gain
    // @Description: Optical Flow based loiter controller pitch axis D gain.  Compensates for short-term change in speed in the pitch direction
    // @Range: 0.100 0.140
    // @User: Standard
    GGROUP(pid_optflow_pitch, "OF_PIT_",   AC_PID),

    // PI controller
    //--------------
    // @Param: STB_RLL_P
    // @DisplayName: Roll axis stabilize controller P gain
    // @Description: Roll axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 6.000
    // @User: Standard

    // @Param: STB_RLL_I
    // @DisplayName: Roll axis stabilize controller I gain
    // @Description: Roll axis stabilize (i.e. angle) controller I gain.  Corrects for longer-term difference in desired roll angle and actual angle
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: STB_RLL_IMAX
    // @DisplayName: Roll axis stabilize controller I gain maximum
    // @Description: Roll axis stabilize (i.e. angle) controller I gain maximum.  Constrains the maximum roll rate that the I term will generate
    // @Range: 0 4500
    // @Unit: Centi-Degrees/Sec
    // @User: Standard
    GGROUP(pi_stabilize_roll,       "STB_RLL_", APM_PI),

    // @Param: STB_PIT_P
    // @DisplayName: Pitch axis stabilize controller P gain
    // @Description: Pitch axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 6.000
    // @User: Standard

    // @Param: STB_PIT_I
    // @DisplayName: Pitch axis stabilize controller I gain
    // @Description: Pitch axis stabilize (i.e. angle) controller I gain.  Corrects for longer-term difference in desired pitch angle and actual angle
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: STB_PIT_IMAX
    // @DisplayName: Pitch axis stabilize controller I gain maximum
    // @Description: Pitch axis stabilize (i.e. angle) controller I gain maximum.  Constrains the maximum pitch rate that the I term will generate
    // @Range: 0 4500
    // @Unit: Centi-Degrees/Sec
    // @User: Standard
    GGROUP(pi_stabilize_pitch,      "STB_PIT_", APM_PI),

    // @Param: STB_YAW_P
    // @DisplayName: Yaw axis stabilize controller P gain
    // @Description: Yaw axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 6.000
    // @User: Standard

    // @Param: STB_YAW_I
    // @DisplayName: Yaw axis stabilize controller I gain
    // @Description: Yaw axis stabilize (i.e. angle) controller I gain.  Corrects for longer-term difference in desired yaw angle and actual angle
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: STB_YAW_IMAX
    // @DisplayName: Yaw axis stabilize controller I gain maximum
    // @Description: Yaw axis stabilize (i.e. angle) controller I gain maximum.  Constrains the maximum yaw rate that the I term will generate
    // @Range: 0 4500
    // @Unit: Centi-Degrees/Sec
    // @User: Standard
    GGROUP(pi_stabilize_yaw,        "STB_YAW_", APM_PI),

    // @Param: THR_ALT_P
    // @DisplayName: Altitude controller P gain
    // @Description: Altitude controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 3.000 6.000
    // @User: Standard

    // @Param: THR_ALT_I
    // @DisplayName: Altitude controller I gain
    // @Description: Altitude controller I gain.  Corrects for longer-term difference in desired altitude and actual altitude
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: THR_ALT_IMAX
    // @DisplayName: Altitude controller I gain maximum
    // @Description: Altitude controller I gain maximum.  Constrains the maximum climb rate rate that the I term will generate
    // @Range: 0 500
    // @Unit: cm/s
    // @User: Standard
    GGROUP(pi_alt_hold,     "THR_ALT_", APM_PI),

    // @Param: HLD_LAT_P
    // @DisplayName: Loiter latitude position controller P gain
    // @Description: Loiter latitude position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.100 0.300
    // @User: Standard

    // @Param: HLD_LAT_I
    // @DisplayName: Loiter latitude position controller I gain
    // @Description: Loiter latitude position controller I gain.  Corrects for longer-term distance (in latitude) to the target location
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: HLD_LAT_IMAX
    // @DisplayName: Loiter latitude position controller I gain maximum
    // @Description: Loiter latitude position controller I gain maximum.  Constrains the maximum desired speed that the I term will generate
    // @Range: 0 3000
    // @Unit: cm/s
    // @User: Standard
    GGROUP(pi_loiter_lat,   "HLD_LAT_", APM_PI),

    // @Param: HLD_LON_P
    // @DisplayName: Loiter longitude position controller P gain
    // @Description: Loiter longitude position controller P gain.  Converts the distance (in the longitude direction) to the target location into a desired speed which is then passed to the loiter longitude rate controller
    // @Range: 0.100 0.300
    // @User: Standard

    // @Param: HLD_LON_I
    // @DisplayName: Loiter longitude position controller I gain
    // @Description: Loiter longitude position controller I gain.  Corrects for longer-term distance (in longitude direction) to the target location
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: HLD_LON_IMAX
    // @DisplayName: Loiter longitudeposition controller I gain maximum
    // @Description: Loiter  longitudeposition controller I gain maximum.  Constrains the maximum desired speed that the I term will generate
    // @Range: 0 3000
    // @Unit: cm/s
    // @User: Standard
    GGROUP(pi_loiter_lon,   "HLD_LON_", APM_PI),

    // variables not in the g class which contain EEPROM saved variables

    // variables not in the g class which contain EEPROM saved variables
#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,           "CAM_", AP_Camera),
#endif

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

    // @Group: SR0_
    // @Path: ./GCS_Mavlink.pde
    GOBJECT(gcs0,                   "SR0_",     GCS_MAVLINK),

    // @Group: SR3_
    // @Path: ./GCS_Mavlink.pde
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

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    GOBJECT(sitl, "SIM_", SITL),
#endif

    GOBJECT(barometer, "GND_", AP_Baro),
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

#if AP_LIMITS == ENABLED
    //@Group: LIM_
    //@Path: ../libraries/AP_Limits/AP_Limits.cpp,../libraries/AP_Limits/AP_Limit_GPSLock.cpp,../libraries/AP_Limits/AP_Limit_Geofence.cpp,../libraries/AP_Limits/AP_Limit_Altitude.cpp,../libraries/AP_Limits/AP_Limit_Module.cpp
    GOBJECT(limits,                 "LIM_",    AP_Limits),
    GOBJECT(gpslock_limit,          "LIM_",    AP_Limit_GPSLock),
    GOBJECT(geofence_limit,         "LIM_",    AP_Limit_Geofence),
    GOBJECT(altitude_limit,         "LIM_",    AP_Limit_Altitude),
#endif

#if FRAME_CONFIG ==     HELI_FRAME
    // @Group: H_
    // @Path: ../libraries/AP_Motors/AP_MotorsHeli.cpp
    GOBJECT(motors, "H_",           AP_MotorsHeli),
#else
    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_Motors_Class.cpp
    GOBJECT(motors, "MOT_",         AP_Motors),
#endif

    AP_VAREND
};


static void load_parameters(void)
{
    // change the default for the AHRS_GPS_GAIN for ArduCopter
    // if it hasn't been set by the user
    if (!ahrs.gps_gain.load()) {
        ahrs.gps_gain.set_and_save(1.0);
    }

    // setup different AHRS gains for ArduCopter than the default
    // but allow users to override in their config
    if (!ahrs._kp.load()) {
        ahrs._kp.set_and_save(0.1);
    }
    if (!ahrs._kp_yaw.load()) {
        ahrs._kp_yaw.set_and_save(0.1);
    }

#if SECONDARY_DMP_ENABLED == ENABLED
    if (!ahrs2._kp.load()) {
        ahrs2._kp.set(0.1);
    }
    if (!ahrs2._kp_yaw.load()) {
        ahrs2._kp_yaw.set(0.1);
    }
#endif


    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        cliSerial->printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        default_dead_zones();
        cliSerial->println_P(PSTR("done."));
    } else {
        uint32_t before = micros();
        // Load all auto-loaded EEPROM variables
        AP_Param::load_all();

        cliSerial->printf_P(PSTR("load_all took %luus\n"), micros() - before);
    }
}
#line 1 "./Firmware/ACopter32/UserCode.pde"
void userhook_init()
{
    // put your initialisation code here


}

void userhook_50Hz()
{
    // put your 50Hz code here


}
#line 1 "./Firmware/ACopter32/commands.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_commands()
{
    g.command_index         = NO_COMMAND;
    command_nav_index       = NO_COMMAND;
    command_cond_index      = NO_COMMAND;
    prev_nav_index          = NO_COMMAND;
    command_cond_queue.id   = NO_COMMAND;
    command_nav_queue.id    = NO_COMMAND;

    ap.fast_corner          = false;
	reset_desired_speed();
}

// Getters
// -------
static struct Location get_cmd_with_index(int i)
{
    struct Location temp;

    // Find out proper location in memory by using the start_byte position + the index
    // --------------------------------------------------------------------------------
    if (i >= g.command_total) {
        // we do not have a valid command to load
        // return a WP with a "Blank" id
        temp.id = CMD_BLANK;

        // no reason to carry on
        return temp;

    }else{
        // we can load a command, we don't process it yet
        // read WP position
        uint16_t mem = (WP_START_BYTE) + (i * WP_SIZE);

        temp.id = hal.storage->read_byte(mem);

        mem++;
        temp.options = hal.storage->read_byte(mem);

        mem++;
        temp.p1 = hal.storage->read_byte(mem);

        mem++;
        temp.alt = hal.storage->read_dword(mem);           // alt is stored in CM! Alt is stored relative!

        mem += 4;
        temp.lat = hal.storage->read_dword(mem);         // lat is stored in decimal * 10,000,000

        mem += 4;
        temp.lng = hal.storage->read_dword(mem);         // lon is stored in decimal * 10,000,000
    }

    // Add on home altitude if we are a nav command (or other command with altitude) and stored alt is relative
    //if((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) && temp.options & MASK_OPTIONS_RELATIVE_ALT){
    //temp.alt += home.alt;
    //}

    if(temp.options & WP_OPTION_RELATIVE) {
        // If were relative, just offset from home
        temp.lat        +=      home.lat;
        temp.lng        +=      home.lng;
    }

    return temp;
}

// Setters
// -------
static void set_cmd_with_index(struct Location temp, int i)
{

    i = constrain(i, 0, g.command_total.get());
    //cliSerial->printf("set_command: %d with id: %d\n", i, temp.id);

    // store home as 0 altitude!!!
    // Home is always a MAV_CMD_NAV_WAYPOINT (16)
    if (i == 0) {
        temp.alt = 0;
        temp.id = MAV_CMD_NAV_WAYPOINT;
    }

    uint16_t mem = WP_START_BYTE + (i * WP_SIZE);

    hal.storage->write_byte(mem, temp.id);

    mem++;
    hal.storage->write_byte(mem, temp.options);

    mem++;
    hal.storage->write_byte(mem, temp.p1);

    mem++;
    hal.storage->write_dword(mem, temp.alt);     // Alt is stored in CM!

    mem += 4;
    hal.storage->write_dword(mem, temp.lat);     // Lat is stored in decimal degrees * 10^7

    mem += 4;
    hal.storage->write_dword(mem, temp.lng);     // Long is stored in decimal degrees * 10^7

    // Make sure our WP_total
    if(g.command_total < (i+1))
        g.command_total.set_and_save(i+1);
}

static int32_t get_RTL_alt()
{
    if(g.rtl_altitude <= 0) {
		return min(current_loc.alt, RTL_ALT_MAX);
    }else if (g.rtl_altitude < current_loc.alt) {
		return min(current_loc.alt, RTL_ALT_MAX);
    }else{
        return g.rtl_altitude;
    }
}


//********************************************************************************
// This function sets the waypoint and modes for Return to Launch
// It's not currently used
//********************************************************************************

/*
 *  This function sets the next waypoint command
 *  It precalculates all the necessary stuff.
 */

static void set_next_WP(struct Location *wp)
{
    // copy the current WP into the OldWP slot
    // ---------------------------------------
    if (next_WP.lat == 0 || command_nav_index <= 1) {
        prev_WP = current_loc;
    }else{
        if (get_distance_cm(&current_loc, &next_WP) < 500)
            prev_WP = next_WP;
        else
            prev_WP = current_loc;
    }

    // Load the next_WP slot
    // ---------------------
    next_WP.options = wp->options;
    next_WP.lat = wp->lat;
    next_WP.lng = wp->lng;

    // Set new target altitude so we can track it for climb_rate
    // To-Do: remove the restriction on negative altitude targets (after testing)
    set_new_altitude(max(wp->alt,100));

    // this is handy for the groundstation
    // -----------------------------------
    wp_distance             = get_distance_cm(&current_loc, &next_WP);
    wp_bearing              = get_bearing_cd(&prev_WP, &next_WP);

    // calc the location error:
    calc_location_error(&next_WP);

    // to check if we have missed the WP
    // ---------------------------------
    original_wp_bearing = wp_bearing;
}


// run this at setup on the ground
// -------------------------------
static void init_home()
{
    set_home_is_set(true);
    home.id         = MAV_CMD_NAV_WAYPOINT;
    home.lng        = g_gps->longitude;                                 // Lon * 10**7
    home.lat        = g_gps->latitude;                                  // Lat * 10**7
    home.alt        = 0;                                                        // Home is always 0

    // Save Home to EEPROM
    // -------------------
    // no need to save this to EPROM
    set_cmd_with_index(home, 0);

#if INERTIAL_NAV_XY == ENABLED
    // set inertial nav's home position
    inertial_nav.set_current_position(g_gps->longitude, g_gps->latitude);
#endif

    if (g.log_bitmask & MASK_LOG_CMD)
        Log_Write_Cmd(0, &home);

    // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
    scaleLongDown = longitude_scale(&home);
    scaleLongUp   = 1.0f/scaleLongDown;

    // Save prev loc this makes the calcs look better before commands are loaded
    prev_WP = home;

    // Load home for a default guided_WP
    // -------------
    guided_WP = home;
    guided_WP.alt += g.rtl_altitude;
}



#line 1 "./Firmware/ACopter32/commands_logic.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
// process_nav_command - main switch statement to initiate the next nav command in the command_nav_queue
static void process_nav_command()
{
    switch(command_nav_queue.id) {

    case MAV_CMD_NAV_TAKEOFF:                   // 22
        do_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp();
        break;

    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 Loiter indefinitely
        do_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_loiter_turns();
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19
        do_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:             //20
        do_RTL();
        break;

    // point the copter and camera at a region of interest (ROI)
    case MAV_CMD_NAV_ROI:             // 80
        do_nav_roi();
        break;

    default:
        break;
    }

}

static void process_cond_command()
{
    switch(command_cond_queue.id) {

    case MAV_CMD_CONDITION_DELAY:             // 112
        do_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114
        do_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:             // 113
        do_change_alt();
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw();
        break;

    default:
        break;
    }
}

static void process_now_command()
{
    switch(command_cond_queue.id) {

    case MAV_CMD_DO_JUMP:              // 177
        do_jump();
        break;

    case MAV_CMD_DO_CHANGE_SPEED:             // 178
        do_change_speed();
        break;

    case MAV_CMD_DO_SET_HOME:             // 179
        do_set_home();
        break;

    case MAV_CMD_DO_SET_SERVO:             // 183
        do_set_servo();
        break;

    case MAV_CMD_DO_SET_RELAY:             // 181
        do_set_relay();
        break;

    case MAV_CMD_DO_REPEAT_SERVO:             // 184
        do_repeat_servo();
        break;

    case MAV_CMD_DO_REPEAT_RELAY:             // 182
        do_repeat_relay();
        break;

#if CAMERA == ENABLED
    case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
        do_take_picture();
        break;
#endif

#if MOUNT == ENABLED
    case MAV_CMD_DO_MOUNT_CONFIGURE:                    // Mission command to configure a camera mount |Mount operation mode (see MAV_CONFIGURE_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
        camera_mount.configure_cmd();
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // Mission command to control a camera mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|
        camera_mount.control_cmd();
        break;
#endif

    default:
        // do nothing with unrecognized MAVLink messages
        break;
    }
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

// verify_must - switch statement to ensure the active navigation command is progressing
// returns true once the active navigation command completes successfully
static bool verify_must()
{
    switch(command_nav_queue.id) {

    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp();
        break;

    case MAV_CMD_NAV_LAND:
        return verify_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_loiter_turns();
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();
        break;

    case MAV_CMD_NAV_ROI:             // 80
        return verify_nav_roi();
        break;

    default:
        //gcs_send_text_P(SEVERITY_HIGH,PSTR("<verify_must: default> No current Must commands"));
        return false;
        break;
    }
}

// verify_may - switch statement to ensure the active conditional command is progressing
// returns true once the active conditional command completes successfully
static bool verify_may()
{
    switch(command_cond_queue.id) {

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        return verify_change_alt();
        break;

    case MAV_CMD_CONDITION_YAW:
        return verify_yaw();
        break;

    default:
        //gcs_send_text_P(SEVERITY_HIGH,PSTR("<verify_must: default> No current May commands"));
        return false;
        break;
    }
}

/********************************************************************************/
//
/********************************************************************************/

// do_RTL - start Return-to-Launch
static void do_RTL(void)
{
    // set rtl state
    rtl_state = RTL_STATE_INITIAL_CLIMB;

    // set roll, pitch and yaw modes
    set_roll_pitch_mode(RTL_RP);
    set_yaw_mode(YAW_HOLD);     // first stage of RTL is the initial climb so just hold current yaw
    set_throttle_mode(RTL_THR);

    // set navigation mode
    set_nav_mode(NAV_LOITER);

    // initial climb starts at current location
    set_next_WP(&current_loc);

    // override altitude to RTL altitude
    set_new_altitude(get_RTL_alt());
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

// do_takeoff - initiate takeoff navigation command
static void do_takeoff()
{
    set_nav_mode(NAV_LOITER);

    // Start with current location
    Location temp = current_loc;

    // alt is always relative
    temp.alt = command_nav_queue.alt;

    // Set our waypoint
    set_next_WP(&temp);

    // set our yaw mode
    set_yaw_mode(YAW_HOLD);

    // prevent flips
    reset_I_all();    
}

// do_nav_wp - initiate move to next waypoint
static void do_nav_wp()
{
    set_nav_mode(NAV_WP);

    set_next_WP(&command_nav_queue);

    // this is our bitmask to verify we have met all conditions to move on
    wp_verify_byte  = 0;

    // if no alt requirement in the waypoint, set the altitude achieved bit of wp_verify_byte
    if((next_WP.options & WP_OPTION_ALT_REQUIRED) == false) {
        wp_verify_byte |= NAV_ALTITUDE;
    }

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time     = 0;

    // this is the delay, stored in seconds and expanded to millis
    loiter_time_max = command_nav_queue.p1 * 1000;

    // reset control of yaw to default
    if( g.yaw_override_behaviour == YAW_OVERRIDE_BEHAVIOUR_AT_NEXT_WAYPOINT ) {
        set_yaw_mode(AUTO_YAW);
    }
}

// do_land - initiate landing procedure
static void do_land()
{
    // hold at our current location
    set_next_WP(&current_loc);
    set_nav_mode(NAV_LOITER);

    // hold current heading
    set_yaw_mode(YAW_HOLD);

    set_throttle_mode(THROTTLE_LAND);
}

static void do_loiter_unlimited()
{
    set_nav_mode(NAV_LOITER);

    //cliSerial->println("dloi ");
    if(command_nav_queue.lat == 0) {
        set_next_WP(&current_loc);
        set_nav_mode(NAV_LOITER);
    }else{
        set_next_WP(&command_nav_queue);
        set_nav_mode(NAV_WP);
    }
}

// do_loiter_turns - initiate moving in a circle
static void do_loiter_turns()
{
    set_nav_mode(NAV_CIRCLE);

    if(command_nav_queue.lat == 0) {
        // allow user to specify just the altitude
        if(command_nav_queue.alt > 0) {
            current_loc.alt = command_nav_queue.alt;
        }
        set_next_WP(&current_loc);
    }else{
        set_next_WP(&command_nav_queue);
    }

    circle_WP = next_WP;

    loiter_total = command_nav_queue.p1 * 360;
    loiter_sum       = 0;
    old_wp_bearing = wp_bearing;

    circle_angle = wp_bearing + 18000;
    circle_angle = wrap_360(circle_angle);
    circle_angle *= RADX100;
}

// do_loiter_time - initiate loitering at a point for a given time period
static void do_loiter_time()
{
    if(command_nav_queue.lat == 0) {
        set_nav_mode(NAV_LOITER);
        loiter_time     = millis();
        set_next_WP(&current_loc);
    }else{
        set_nav_mode(NAV_WP);
        set_next_WP(&command_nav_queue);
    }

    loiter_time_max = command_nav_queue.p1 * 1000;     // units are (seconds)
}

/********************************************************************************/
//	Verify Nav (Must) commands
/********************************************************************************/

// verify_takeoff - check if we have completed the takeoff
static bool verify_takeoff()
{
    // wait until we are ready!
    if(g.rc_3.control_in == 0) {
        return false;
    }
    // are we above our target altitude?
    return (alt_change_flag == REACHED_ALT);
}

// verify_land - returns true if landing has been completed
static bool verify_land()
{
    // rely on THROTTLE_LAND mode to correctly update landing status
    return ap.land_complete;
}

// verify_nav_wp - check if we have reached the next way point
static bool verify_nav_wp()
{
    // Altitude checking
    if(next_WP.options & WP_OPTION_ALT_REQUIRED) {
        // we desire a certain minimum altitude
        if(alt_change_flag == REACHED_ALT) {

            // we have reached that altitude
            wp_verify_byte |= NAV_ALTITUDE;
        }
    }

    // Did we pass the WP?	// Distance checking
    if((wp_distance <= (uint32_t)max((g.waypoint_radius * 100),0)) || check_missed_wp()) {
        wp_verify_byte |= NAV_LOCATION;
        if(loiter_time == 0) {
            loiter_time = millis();
        }
    }

    // Hold at Waypoint checking, we cant move on until this is OK
    if(wp_verify_byte & NAV_LOCATION) {
        // we have reached our goal

        // loiter at the WP
        set_nav_mode(NAV_LOITER);

        if ((millis() - loiter_time) > loiter_time_max) {
            wp_verify_byte |= NAV_DELAY;
            //gcs_send_text_P(SEVERITY_LOW,PSTR("verify_must: LOITER time complete"));
            //cliSerial->println("vlt done");
        }
    }

    if( wp_verify_byte == (NAV_LOCATION | NAV_ALTITUDE | NAV_DELAY) ) {
        gcs_send_text_fmt(PSTR("Reached Command #%i"),command_nav_index);
        wp_verify_byte = 0;
        copter_leds_nav_blink = 15;             // Cause the CopterLEDs to blink three times to indicate waypoint reached
        return true;
    }else{
        return false;
    }
}

static bool verify_loiter_unlimited()
{
    if(nav_mode == NAV_WP &&  wp_distance <= (uint32_t)max((g.waypoint_radius * 100),0)) {
        // switch to position hold
        set_nav_mode(NAV_LOITER);
    }
    return false;
}

// verify_loiter_time - check if we have loitered long enough
static bool verify_loiter_time()
{
    if(nav_mode == NAV_LOITER) {
        if ((millis() - loiter_time) > loiter_time_max) {
            return true;
        }
    }
    if(nav_mode == NAV_WP &&  wp_distance <= (uint32_t)max((g.waypoint_radius * 100),0)) {
        // reset our loiter time
        loiter_time = millis();
        // switch to position hold
        set_nav_mode(NAV_LOITER);
    }
    return false;
}

// verify_loiter_turns - check if we have circled the point enough
static bool verify_loiter_turns()
{
    //cliSerial->printf("loiter_sum: %d \n", loiter_sum);
    // have we rotated around the center enough times?
    // -----------------------------------------------
    if(abs(loiter_sum) > loiter_total) {
        loiter_total    = 0;
        loiter_sum              = 0;
        //gcs_send_text_P(SEVERITY_LOW,PSTR("verify_must: LOITER orbits complete"));
        // clear the command queue;
        return true;
    }
    return false;
}

// verify_RTL - handles any state changes required to implement RTL
// do_RTL should have been called once first to initialise all variables
// returns true with RTL has completed successfully
static bool verify_RTL()
{
    bool retval = false;

    switch( rtl_state ) {

        case RTL_STATE_INITIAL_CLIMB:
            // rely on verify_altitude function to update alt_change_flag when we've reached the target
            if(alt_change_flag == REACHED_ALT || alt_change_flag == DESCENDING) {
                // Set navigation target to home
                set_next_WP(&home);

                // override target altitude to RTL altitude
                set_new_altitude(get_RTL_alt());

                // set navigation mode
                set_nav_mode(NAV_WP);

                // set yaw mode
                set_yaw_mode(RTL_YAW);

                // advance to next rtl state
                rtl_state = RTL_STATE_RETURNING_HOME;
            }
            break;

        case RTL_STATE_RETURNING_HOME:
            // if we've reached home initiate loiter
            if (wp_distance <= (uint32_t)max((g.waypoint_radius * 100),0) || check_missed_wp()) {
                rtl_state = RTL_STATE_LOITERING_AT_HOME;
                set_nav_mode(NAV_LOITER);

                // set loiter timer
                rtl_loiter_start_time = millis();

                // give pilot back control of yaw
                set_yaw_mode(YAW_HOLD);
            }
            break;

        case RTL_STATE_LOITERING_AT_HOME:
            // check if we've loitered long enough
            if( millis() - rtl_loiter_start_time > (uint32_t)g.rtl_loiter_time.get() ) {
                // initiate landing or descent
                if(g.rtl_alt_final == 0 || ap.failsafe) {
                    // land
                    do_land();
                    // override landing location (do_land defaults to current location)
                    next_WP.lat = home.lat;
                    next_WP.lng = home.lng;
                    // update RTL state
                    rtl_state = RTL_STATE_LAND;
                }else{
                    // descend
                    if(current_loc.alt > g.rtl_alt_final) {
                        set_new_altitude(g.rtl_alt_final);
                    }
                    // update RTL state
                    rtl_state = RTL_STATE_FINAL_DESCENT;
                }
            }
            break;

        case RTL_STATE_FINAL_DESCENT:
            // rely on altitude check to confirm we have reached final altitude
            if(current_loc.alt <= g.rtl_alt_final || alt_change_flag == REACHED_ALT) {
                // switch to regular loiter mode
                set_mode(LOITER);
                // override location and altitude
                set_next_WP(&home);
                // override altitude to RTL altitude
                set_new_altitude(g.rtl_alt_final);
                retval = true;
            }
            break;

        case RTL_STATE_LAND:
            // rely on verify_land to return correct status
            retval = verify_land();
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully completed RTL
    return retval;
}

/********************************************************************************/
//	Condition (May) commands
/********************************************************************************/

static void do_wait_delay()
{
    //cliSerial->print("dwd ");
    condition_start = millis();
    condition_value = command_cond_queue.lat * 1000;     // convert to milliseconds
    //cliSerial->println(condition_value,DEC);
}

static void do_change_alt()
{
    Location temp   = next_WP;
    condition_start = current_loc.alt;
    //condition_value	= command_cond_queue.alt;
    temp.alt                = command_cond_queue.alt;
    set_next_WP(&temp);
}

static void do_within_distance()
{
    condition_value  = command_cond_queue.lat * 100;
}

static void do_yaw()
{
    // get final angle, 1 = Relative, 0 = Absolute
    if( command_cond_queue.lng == 0 ) {
        // absolute angle
        yaw_look_at_heading = wrap_360(command_cond_queue.alt * 100);
    }else{
        // relative angle
        yaw_look_at_heading = wrap_360(nav_yaw + command_cond_queue.alt * 100);
    }

    // get turn speed
    if( command_cond_queue.lat == 0 ) {
        // default to regular auto slew rate
        yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
    }else{
        int32_t turn_rate = (wrap_180(yaw_look_at_heading - nav_yaw) / 100) / command_cond_queue.lat;
        yaw_look_at_heading_slew = constrain(turn_rate, 1, 360);    // deg / sec
    }

    // set yaw mode
    set_yaw_mode(YAW_LOOK_AT_HEADING);

    // TO-DO: restore support for clockwise / counter clockwise rotation held in command_cond_queue.p1
    // command_cond_queue.p1; // 0 = undefined, 1 = clockwise, -1 = counterclockwise
}


/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_wait_delay()
{
    //cliSerial->print("vwd");
    if (millis() - condition_start > (uint32_t)max(condition_value,0)) {
        //cliSerial->println("y");
        condition_value = 0;
        return true;
    }
    //cliSerial->println("n");
    return false;
}

static bool verify_change_alt()
{
    //cliSerial->printf("change_alt, ca:%d, na:%d\n", (int)current_loc.alt, (int)next_WP.alt);
    if ((int32_t)condition_start < next_WP.alt) {
        // we are going higer
        if(current_loc.alt > next_WP.alt) {
            return true;
        }
    }else{
        // we are going lower
        if(current_loc.alt < next_WP.alt) {
            return true;
        }
    }
    return false;
}

static bool verify_within_distance()
{
    //cliSerial->printf("cond dist :%d\n", (int)condition_value);
    if (wp_distance < max(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - return true if we have reached the desired heading
static bool verify_yaw()
{
    if( labs(wrap_180(ahrs.yaw_sensor-yaw_look_at_heading)) <= 200 ) {
        return true;
    }else{
        return false;
    }
}

// verify_nav_roi - verifies that actions required by MAV_CMD_NAV_ROI have completed
//              we assume the camera command has been successfully implemented by the do_nav_roi command
//              so all we need to check is whether we needed to yaw the copter (due to the mount type) and
//              whether that yaw has completed
//	TO-DO: add support for other features of MAV_NAV_ROI including pointing at a given waypoint
static bool verify_nav_roi()
{
#if MOUNT == ENABLED
    // check if mount type requires us to rotate the quad
    if( camera_mount.get_mount_type() != AP_Mount::k_pan_tilt && camera_mount.get_mount_type() != AP_Mount::k_pan_tilt_roll ) {
        // ensure yaw has gotten to within 2 degrees of the target
        if( labs(wrap_180(ahrs.yaw_sensor-yaw_look_at_WP_bearing)) <= 200 ) {
            return true;
        }else{
            return false;
        }
    }else{
        // if no rotation required, assume the camera instruction was implemented immediately
        return true;
    }
#else
    // if we have no camera mount simply check we've reached the desired yaw
    // ensure yaw has gotten to within 2 degrees of the target
    if( labs(wrap_180(ahrs.yaw_sensor-yaw_look_at_WP_bearing)) <= 200 ) {
        return true;
    }else{
        return false;
    }
#endif
}

/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/

static void do_change_speed()
{
    g.waypoint_speed_max = command_cond_queue.p1 * 100;
}

static void do_jump()
{
    // Used to track the state of the jump command in Mission scripting
    // -10 is a value that means the register is unused
    // when in use, it contains the current remaining jumps
    static int8_t jump = -10;                                                                   // used to track loops in jump command

    //cliSerial->printf("do Jump: %d\n", jump);

    if(jump == -10) {
        //cliSerial->printf("Fresh Jump\n");
        // we use a locally stored index for jump
        jump = command_cond_queue.lat;
    }
    //cliSerial->printf("Jumps left: %d\n",jump);

    if(jump > 0) {
        //cliSerial->printf("Do Jump to %d\n",command_cond_queue.p1);
        jump--;
        change_command(command_cond_queue.p1);

    } else if (jump == 0) {
        //cliSerial->printf("Did last jump\n");
        // we're done, move along
        jump = -11;

    } else if (jump == -1) {
        //cliSerial->printf("jumpForever\n");
        // repeat forever
        change_command(command_cond_queue.p1);
    }
}

static void do_set_home()
{
    if(command_cond_queue.p1 == 1) {
        init_home();
    } else {
        home.id         = MAV_CMD_NAV_WAYPOINT;
        home.lng        = command_cond_queue.lng;                                       // Lon * 10**7
        home.lat        = command_cond_queue.lat;                                       // Lat * 10**7
        home.alt        = 0;
        //home_is_set 	= true;
        set_home_is_set(true);
    }
}

static void do_set_servo()
{
    uint8_t channel_num = 0xff;

    switch( command_cond_queue.p1 ) {
        case 1:
            channel_num = CH_1;
            break;
        case 2:
            channel_num = CH_2;
            break;
        case 3:
            channel_num = CH_3;
            break;
        case 4:
            channel_num = CH_4;
            break;
        case 5:
            channel_num = CH_5;
            break;
        case 6:
            channel_num = CH_6;
            break;
        case 7:
            channel_num = CH_7;
            break;
        case 8:
            channel_num = CH_8;
            break;
        case 9:
            // not used
            break;
        case 10:
            channel_num = CH_10;
            break;
        case 11:
            channel_num = CH_11;
            break;
    }

    // send output to channel
    if (channel_num != 0xff) {
        hal.rcout->enable_ch(channel_num);
        hal.rcout->write(channel_num, command_cond_queue.alt);
    }
}

static void do_set_relay()
{
    if (command_cond_queue.p1 == 1) {
        relay.on();
    } else if (command_cond_queue.p1 == 0) {
        relay.off();
    }else{
        relay.toggle();
    }
}

static void do_repeat_servo()
{
    event_id = command_cond_queue.p1 - 1;

    if(command_cond_queue.p1 >= CH_5 + 1 && command_cond_queue.p1 <= CH_8 + 1) {

        event_timer             = 0;
        event_value             = command_cond_queue.alt;
        event_repeat    = command_cond_queue.lat * 2;
        event_delay             = command_cond_queue.lng * 500.0f;         // /2 (half cycle time) * 1000 (convert to milliseconds)

        switch(command_cond_queue.p1) {
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
    event_id                = RELAY_TOGGLE;
    event_timer             = 0;
    event_delay             = command_cond_queue.lat * 500.0f;     // /2 (half cycle time) * 1000 (convert to milliseconds)
    event_repeat    = command_cond_queue.alt * 2;
    update_events();
}

// do_nav_roi - starts actions required by MAV_CMD_NAV_ROI
//              this involves either moving the camera to point at the ROI (region of interest)
//              and possibly rotating the copter to point at the ROI if our mount type does not support a yaw feature
//				Note: the ROI should already be in the command_nav_queue global variable
//	TO-DO: add support for other features of MAV_NAV_ROI including pointing at a given waypoint
static void do_nav_roi()
{
#if MOUNT == ENABLED

    // check if mount type requires us to rotate the quad
    if( camera_mount.get_mount_type() != AP_Mount::k_pan_tilt && camera_mount.get_mount_type() != AP_Mount::k_pan_tilt_roll ) {
        yaw_look_at_WP = command_nav_queue;
        set_yaw_mode(YAW_LOOK_AT_LOCATION);
    }
    // send the command to the camera mount
    camera_mount.set_roi_cmd(&command_nav_queue);

    // TO-DO: expand handling of the do_nav_roi to support all modes of the MAVLink.  Currently we only handle mode 4 (see below)
    //		0: do nothing
    //		1: point at next waypoint
    //		2: point at a waypoint taken from WP# parameter (2nd parameter?)
    //		3: point at a location given by alt, lon, lat parameters
    //		4: point at a target given a target id (can't be implmented)
#else
    // if we have no camera mount aim the quad at the location
    yaw_look_at_WP = command_nav_queue;
    set_yaw_mode(YAW_LOOK_AT_LOCATION);
#endif
}

// do_take_picture - take a picture with the camera library
static void do_take_picture()
{
#if CAMERA == ENABLED
    camera.trigger_pic();
    if (g.log_bitmask & MASK_LOG_CAMERA) {
        Log_Write_Camera();
    }
#endif
}
#line 1 "./Firmware/ACopter32/commands_process.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// For changing active command mid-mission
//----------------------------------------
static void change_command(uint8_t cmd_index)
{
    //cliSerial->printf("change_command: %d\n",cmd_index );
    // limit range
    cmd_index = min(g.command_total - 1, cmd_index);

    // load command
    struct Location temp = get_cmd_with_index(cmd_index);

    //cliSerial->printf("loading cmd: %d with id:%d\n", cmd_index, temp.id);

    // verify it's a nav command
    if(temp.id > MAV_CMD_NAV_LAST) {
        //gcs_send_text_P(SEVERITY_LOW,PSTR("error: non-Nav cmd"));

    }else{
        // clear out command queue
        init_commands();

        // copy command to the queue
        command_nav_queue               = temp;
        command_nav_index               = cmd_index;
        execute_nav_command();
    }
}

// update_commands - initiates new navigation commands if we have completed the previous command
// called by 10 Hz loop
static void update_commands()
{
    //cliSerial->printf("update_commands: %d\n",increment );
    // A: if we do not have any commands there is nothing to do
    // B: We have completed the mission, don't redo the mission
    // XXX debug
    //uint8_t tmp = g.command_index.get();
    //cliSerial->printf("command_index %u \n", tmp);

    if(g.command_total <= 1 || g.command_index >= 255)
        return;

    if(command_nav_queue.id == NO_COMMAND) {
        // Our queue is empty
        // fill command queue with a new command if available, or exit mission
        // -------------------------------------------------------------------

        // find next nav command
        int16_t tmp_index;

        if(command_nav_index < g.command_total) {

            // what is the next index for a nav command?
            tmp_index = find_next_nav_index(command_nav_index + 1);

            if(tmp_index == -1) {
                exit_mission();
                return;
            }else{
                command_nav_index = tmp_index;
                command_nav_queue = get_cmd_with_index(command_nav_index);
                execute_nav_command();
            }

            // try to load the next nav for better speed control
            // find_next_nav_index takes the next guess to start the search
            tmp_index = find_next_nav_index(command_nav_index + 1);

            // Fast corner management
            // ----------------------
            if(tmp_index == -1) {
                // there are no more commands left
            }else{
                // we have at least one more cmd left
                Location tmp_loc = get_cmd_with_index(tmp_index);

                if(tmp_loc.lat == 0) {
                    ap.fast_corner = false;
                }else{
                    int32_t temp = get_bearing_cd(&next_WP, &tmp_loc) - original_wp_bearing;
                    temp = wrap_180(temp);
                    ap.fast_corner = labs(temp) < 6000;
                }

                // If we try and stop at a corner, lets reset our desired speed to prevent
                // too much jerkyness.
				if(false == ap.fast_corner){
					reset_desired_speed();
				}
            }

        }else{
            // we are out of commands
            exit_mission();
            return;
        }
    }

    if(command_cond_queue.id == NO_COMMAND) {
        // Our queue is empty
        // fill command queue with a new command if available, or do nothing
        // -------------------------------------------------------------------

        // no nav commands completed yet
        if(prev_nav_index == NO_COMMAND)
            return;

        if(command_cond_index >= command_nav_index) {
            // don't process the fututre
            return;

        }else if(command_cond_index == NO_COMMAND) {
            // start from scratch
            // look at command after the most recent completed nav
            command_cond_index = prev_nav_index + 1;

        }else{
            // we've completed 1 cond, look at next command for another
            command_cond_index++;
        }

        if(command_cond_index < (g.command_total -2)) {
            // we're OK to load a new command (last command must be a nav command)
            command_cond_queue = get_cmd_with_index(command_cond_index);

            if(command_cond_queue.id > MAV_CMD_CONDITION_LAST) {
                // this is a do now command
                process_now_command();

                // clear command queue
                command_cond_queue.id = NO_COMMAND;

            }else if(command_cond_queue.id > MAV_CMD_NAV_LAST) {
                // this is a conditional command
                process_cond_command();

            }else{
                // this is a nav command, don't process
                // clear the command conditional queue and index
                prev_nav_index                  = NO_COMMAND;
                command_cond_index              = NO_COMMAND;
                command_cond_queue.id   = NO_COMMAND;
            }

        }
    }
}

// execute_nav_command - performs minor initialisation and logging before next navigation command in the queue is executed
static void execute_nav_command(void)
{
    // This is what we report to MAVLINK
    g.command_index = command_nav_index;

    // Save CMD to Log
    if(g.log_bitmask & MASK_LOG_CMD)
        Log_Write_Cmd(g.command_index, &command_nav_queue);

    // clear navigation prameters
    reset_nav_params();

    // Act on the new command
    process_nav_command();

    // clear May indexes to force loading of more commands
    // existing May commands are tossed.
    command_cond_index      = NO_COMMAND;
}

// verify_commands - high level function to check if navigation and conditional commands have completed
// called after GPS navigation update - not constantly
static void verify_commands(void)
{
    if(verify_must()) {
        //cliSerial->printf("verified must cmd %d\n" , command_nav_index);
        command_nav_queue.id    = NO_COMMAND;

        // store our most recent executed nav command
        prev_nav_index                  = command_nav_index;

        // Wipe existing conditionals
        command_cond_index              = NO_COMMAND;
        command_cond_queue.id   = NO_COMMAND;

    }else{
        //cliSerial->printf("verified must false %d\n" , command_nav_index);
    }

    if(verify_may()) {
        //cliSerial->printf("verified may cmd %d\n" , command_cond_index);
        command_cond_queue.id = NO_COMMAND;
    }
}

// Finds the next navgation command in EEPROM
static int16_t find_next_nav_index(int16_t search_index)
{
    Location tmp;
    while(search_index < g.command_total) {
        tmp = get_cmd_with_index(search_index);
        if(tmp.id <= MAV_CMD_NAV_LAST) {
            return search_index;
        }else{
            search_index++;
        }
    }
    return -1;
}

static void exit_mission()
{
    // we are out of commands
    g.command_index = 255;

    // if we are on the ground, enter stabilize, else Land
    if(ap.land_complete) {
        // we will disarm the motors after landing.
    }else{
        // If the approach altitude is valid (above 1m), do approach, else land
        if(g.rtl_alt_final == 0) {
            set_mode(LAND);
        }else{
            set_mode(LOITER);
            set_new_altitude(g.rtl_alt_final);
        }
    }

}

#line 1 "./Firmware/ACopter32/compat.pde"


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

void digitalWriteFast(uint8_t pin, uint8_t out)
{
    hal.gpio->write(pin,out);
}

void digitalWrite(uint8_t pin, uint8_t out)
{
    hal.gpio->write(pin,out);
}

uint8_t digitalReadFast(uint8_t pin)
{
    return hal.gpio->read(pin);
}

uint8_t digitalRead(uint8_t pin)
{
    return hal.gpio->read(pin);
}

#line 1 "./Firmware/ACopter32/control_modes.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define CONTROL_SWITCH_COUNTER  10  // 10 iterations at 100hz (i.e. 1/10th of a second) at a new switch position will cause flight mode change
static void read_control_switch()
{
    static uint8_t switch_counter = 0;

    uint8_t switchPosition = readSwitch();

    if (oldSwitchPosition != switchPosition) {
        switch_counter++;
        if(switch_counter >= CONTROL_SWITCH_COUNTER) {
            oldSwitchPosition       = switchPosition;
            switch_counter          = 0;

            // ignore flight mode changes if in failsafe
            if( !ap.failsafe ) {
                set_mode(flight_modes[switchPosition]);

                if(g.ch7_option != CH7_SIMPLE_MODE) {
                    // set Simple mode using stored paramters from Mission planner
                    // rather than by the control switch
                    set_simple_mode(g.simple_modes & (1 << switchPosition));
                }
            }
        }
    }else{
        // reset switch_counter if there's been no change
        // we don't want 10 intermittant blips causing a flight mode change
        switch_counter = 0;
    }
}

static uint8_t readSwitch(void){
    int16_t pulsewidth = g.rc_5.radio_in;   // default for Arducopter

    if (pulsewidth < 1231) return 0;
    if (pulsewidth < 1361) return 1;
    if (pulsewidth < 1491) return 2;
    if (pulsewidth < 1621) return 3;
    if (pulsewidth < 1750) return 4;        // Software Manual
    return 5;                               // Hardware Manual
}

static void reset_control_switch()
{
    oldSwitchPosition = -1;
    read_control_switch();
}

// read at 10 hz
// set this to your trainer switch
static void read_trim_switch()
{
    // return immediately if the CH7 switch has not changed position
    if (ap_system.CH7_flag == (g.rc_7.radio_in >= CH7_PWM_TRIGGER)) {
        return;
    }

    // set the ch7 flag
    ap_system.CH7_flag = (g.rc_7.radio_in >= CH7_PWM_TRIGGER);

    // multi-mode
    int8_t option;

    if(g.ch7_option == CH7_MULTI_MODE) {
        if (g.rc_6.radio_in < CH6_PWM_TRIGGER_LOW) {
            option = CH7_FLIP;
        }else if (g.rc_6.radio_in > CH6_PWM_TRIGGER_HIGH) {
            option = CH7_SAVE_WP;
        }else{
            option = CH7_RTL;
        }
    }else{
        option = g.ch7_option;
    }

    switch(option) {
        case CH7_FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if(ap_system.CH7_flag && g.rc_3.control_in >= 0 && ap.takeoff_complete) {
                init_flip();
            }
            break;

        case CH7_SIMPLE_MODE:
            set_simple_mode(ap_system.CH7_flag);
            break;

        case CH7_RTL:
            if (ap_system.CH7_flag) {
                // engage RTL
                set_mode(RTL);
            }else{
                // disengage RTL to previous flight mode if we are currently in RTL or loiter
                if (control_mode == RTL || control_mode == LOITER) {
                    reset_control_switch();
                }
            }
            break;

        case CH7_SAVE_TRIM:
            if(ap_system.CH7_flag && control_mode <= ACRO && g.rc_3.control_in == 0) {
                save_trim();
            }
            break;

        case CH7_SAVE_WP:
            // save when CH7 switch is switched off
            if (ap_system.CH7_flag == false) {

                // if in auto mode, reset the mission
                if(control_mode == AUTO) {
                    CH7_wp_index = 0;
                    g.command_total.set_and_save(1);
                    set_mode(RTL);
                    return;
                }

                if(CH7_wp_index == 0) {
                    // this is our first WP, let's save WP 1 as a takeoff
                    // increment index to WP index of 1 (home is stored at 0)
                    CH7_wp_index = 1;

                    Location temp   = home;
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    temp.id = MAV_CMD_NAV_TAKEOFF;
                    temp.alt = current_loc.alt;

                    // save command:
                    // we use the current altitude to be the target for takeoff.
                    // only altitude will matter to the AP mission script for takeoff.
                    // If we are above the altitude, we will skip the command.
                    set_cmd_with_index(temp, CH7_wp_index);
                }

                // increment index
                CH7_wp_index++;

                // set the next_WP (home is stored at 0)
                // max out at 100 since I think we need to stay under the EEPROM limit
                CH7_wp_index = constrain(CH7_wp_index, 1, 100);

                if(g.rc_3.control_in > 0) {
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    current_loc.id = MAV_CMD_NAV_WAYPOINT;
                }else{
                    // set our location ID to 21, MAV_CMD_NAV_LAND
                    current_loc.id = MAV_CMD_NAV_LAND;
                }

                // save command
                set_cmd_with_index(current_loc, CH7_wp_index);

                // Cause the CopterLEDs to blink twice to indicate saved waypoint
                copter_leds_nav_blink = 10;
            }
            break;

#if CAMERA == ENABLED
        case CH7_CAMERA_TRIGGER:
            if(ap_system.CH7_flag) {
                do_take_picture();
            }
            break;
#endif

        case CH7_SONAR:
            // enable or disable the sonar
            g.sonar_enabled = ap_system.CH7_flag;
            break;
    }
}

// save_trim - adds roll and pitch trims from the radio to ahrs
static void save_trim()
{
    // save roll and pitch trim
    float roll_trim = ToRad((float)g.rc_1.control_in/100.0f);
    float pitch_trim = ToRad((float)g.rc_2.control_in/100.0f);
    ahrs.add_trim(roll_trim, pitch_trim);
}

// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
static void auto_trim()
{
    if(auto_trim_counter > 0) {
        auto_trim_counter--;

        // flash the leds
        led_mode = SAVE_TRIM_LEDS;

        // calculate roll trim adjustment
        float roll_trim_adjustment = ToRad((float)g.rc_1.control_in / 4000.0f);

        // calculate pitch trim adjustment
        float pitch_trim_adjustment = ToRad((float)g.rc_2.control_in / 4000.0f);

        // make sure accelerometer values impact attitude quickly
        ahrs.set_fast_gains(true);

        // add trim to ahrs object
        // save to eeprom on last iteration
        ahrs.add_trim(roll_trim_adjustment, pitch_trim_adjustment, (auto_trim_counter == 0));

        // on last iteration restore leds and accel gains to normal
        if(auto_trim_counter == 0) {
            ahrs.set_fast_gains(false);
            led_mode = NORMAL_LEDS;
        }
    }
}

#line 1 "./Firmware/ACopter32/events.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */
static void failsafe_on_event()
{
    // if motors are not armed there is nothing to do
    if( !motors.armed() ) {
        return;
    }

    // This is how to handle a failsafe.
    switch(control_mode) {
        case STABILIZE:
        case ACRO:
            // if throttle is zero disarm motors
            if (g.rc_3.control_in == 0) {
                init_disarm_motors();
            }else if(ap.home_is_set == true && home_distance > g.waypoint_radius) {
                set_mode(RTL);
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
        case AUTO:
            // failsafe_throttle is 1 do RTL, 2 means continue with the mission
            if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_RTL) {
                if(home_distance > g.waypoint_radius) {
                    set_mode(RTL);
                }else{
                    // We are very close to home so we will land
                    set_mode(LAND);
                }
            }
            // if failsafe_throttle is 2 (i.e. FS_THR_ENABLED_CONTINUE_MISSION) no need to do anything
            break;
        default:
            if(ap.home_is_set == true && home_distance > g.waypoint_radius) {
                set_mode(RTL);
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE, ERROR_CODE_FAILSAFE_THROTTLE);

}

// failsafe_off_event - respond to radio contact being regained
// we must be in AUTO, LAND or RTL modes
// or Stabilize or ACRO mode but with motors disarmed
static void failsafe_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE, ERROR_CODE_ERROR_RESOLVED);
}

static void low_battery_event(void)
{
    // failsafe check
    if (g.failsafe_battery_enabled && !ap.low_battery && motors.armed()) {
        switch(control_mode) {
            case STABILIZE:
            case ACRO:
                // if throttle is zero disarm motors
                if (g.rc_3.control_in == 0) {
                    init_disarm_motors();
                }else{
                    set_mode(LAND);
                }
                break;
            case AUTO:
                if(ap.home_is_set == true && home_distance > g.waypoint_radius) {
                    set_mode(RTL);
                }else{
                    // We have no GPS or are very close to home so we will land
                    set_mode(LAND);
                }
                break;
            default:
                set_mode(LAND);
                break;
        }
    }

    // set the low battery flag
    set_low_battery(true);

    // warn the ground station and log to dataflash
    gcs_send_text_P(SEVERITY_LOW,PSTR("Low Battery!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE, ERROR_CODE_FAILSAFE_BATTERY);

#if COPTER_LEDS == ENABLED
    if ( bitRead(g.copter_leds_mode, 3) ) {         // Only Activate if a battery is connected to avoid alarm on USB only
        piezo_on();
    }
#endif // COPTER_LEDS
}


static void update_events()     // Used for MAV_CMD_DO_REPEAT_SERVO and MAV_CMD_DO_REPEAT_RELAY
{
    if(event_repeat == 0 || (millis() - event_timer) < event_delay)
        return;

    if(event_repeat != 0) {             // event_repeat = -1 means repeat forever
        event_timer = millis();

        if (event_id >= CH_5 && event_id <= CH_8) {
            if(event_repeat%2) {
                hal.rcout->write(event_id, event_value);                 // send to Servos
            } else {
                hal.rcout->write(event_id, event_undo_value);
            }
        }

        if  (event_id == RELAY_TOGGLE) {
            relay.toggle();
        }
        if (event_repeat > 0) {
            event_repeat--;
        }
    }
}

#line 1 "./Firmware/ACopter32/failsafe.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//  failsafe support
//  Andrew Tridgell, December 2011
//
//  our failsafe strategy is to detect main loop lockup and disarm the motors
//

static bool failsafe_enabled = true;
static uint16_t failsafe_last_mainLoop_count;
static uint32_t failsafe_last_timestamp;
static bool in_failsafe;

//
// failsafe_enable - enable failsafe
//
void failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = micros();
}

//
// failsafe_disable - used when we know we are going to delay the mainloop significantly
//
void failsafe_disable()
{
    failsafe_enabled = false;
}

//
//  failsafe_check - this function is called from the core timer interrupt at 1kHz.
//
void failsafe_check(uint32_t tnow)
{
    if (mainLoop_count != failsafe_last_mainLoop_count) {
        // the main loop is running, all is OK
        failsafe_last_mainLoop_count = mainLoop_count;
        failsafe_last_timestamp = tnow;
        in_failsafe = false;
        return;
    }

    if (failsafe_enabled && tnow - failsafe_last_timestamp > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors.
        in_failsafe = true;
    }

    if (failsafe_enabled && in_failsafe && tnow - failsafe_last_timestamp > 1000000) {
        // disarm motors every second
        failsafe_last_timestamp = tnow;
        if(motors.armed()) {
            motors.armed(false);
        	set_armed(true);
            motors.output();
            Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE, ERROR_CODE_FAILSAFE_WATCHDOG);
        }
    }
}
#line 1 "./Firmware/ACopter32/flip.pde"
// 2010 Jose Julio
// 2011 Adapted and updated for AC2 by Jason Short
//
// Automatic Acrobatic Procedure (AAP) v1 : Roll flip
// State machine aproach:
//    Some states are fixed commands (for a fixed time)
//    Some states are fixed commands (until some IMU condition)
//    Some states include controls inside
uint8_t flip_timer;
uint8_t flip_state;

#define AAP_THR_INC 170
#define AAP_THR_DEC 120
#define AAP_ROLL_OUT 2000

static int8_t flip_dir;

void init_flip()
{
    if(false == ap.do_flip) {
        ap.do_flip = true;
        flip_state = 0;
        flip_dir = (ahrs.roll_sensor >= 0) ? 1 : -1;
		Log_Write_Event(DATA_BEGIN_FLIP);
    }
}

void roll_flip()
{
    // Pitch
    //g.rc_2.servo_out = get_stabilize_pitch(g.rc_2.control_in);
    get_stabilize_pitch(g.rc_2.control_in);

    int32_t roll = ahrs.roll_sensor * flip_dir;

    // Roll State machine
    switch (flip_state) {
    case 0:
        if (roll < 4500) {
            // Roll control
            g.rc_1.servo_out = AAP_ROLL_OUT * flip_dir;
            set_throttle_out(g.rc_3.control_in + AAP_THR_INC, false);
        }else{
            flip_state++;
        }
        break;

    case 1:
        if((roll >= 4500) || (roll < -9000)) {
		#if FRAME_CONFIG == HELI_FRAME
			g.rc_1.servo_out = get_heli_rate_roll(40000 * flip_dir);
		#else
            g.rc_1.servo_out = get_rate_roll(40000 * flip_dir);
		#endif // HELI_FRAME
        set_throttle_out(g.rc_3.control_in - AAP_THR_DEC, false);
        }else{
            flip_state++;
            flip_timer = 0;
        }
        break;

    case 2:
        if (flip_timer < 100) {
            //g.rc_1.servo_out = get_stabilize_roll(g.rc_1.control_in);
            get_stabilize_roll(g.rc_1.control_in);
            set_throttle_out(g.rc_3.control_in + AAP_THR_INC, false);
            flip_timer++;
        }else{
        	Log_Write_Event(DATA_END_FLIP);
            ap.do_flip = false;
            flip_state = 0;
        }
        break;
    }
}
#line 1 "./Firmware/ACopter32/inertia.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// read_inertia - read inertia in from accelerometers
static void read_inertia()
{
#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
    static uint8_t log_counter_inav = 0;

    // inertial altitude estimates
    inertial_nav.update(G_Dt);

    if( motors.armed() && g.log_bitmask & MASK_LOG_INAV ) {
        log_counter_inav++;
        if( log_counter_inav >= 10 ) {
            log_counter_inav = 0;
            Log_Write_INAV();
        }
    }
#endif
}
#line 1 "./Firmware/ACopter32/leds.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void update_lights()
{
    switch(led_mode) {
    case NORMAL_LEDS:
        update_motor_light();
        update_GPS_light();
        break;

    case SAVE_TRIM_LEDS:
        dancing_light();
        break;
    }
}

static void update_GPS_light(void)
{
    // GPS LED on if we have a fix or Blink GPS LED if we are receiving data
    // ---------------------------------------------------------------------
    switch (g_gps->status()) {

    case (2):
        if(ap.home_is_set) {                                      // JLN update
            digitalWriteFast(C_LED_PIN, LED_ON);                  //Turn LED C on when gps has valid fix AND home is set.
        } else {
            digitalWriteFast(C_LED_PIN, LED_OFF);
        }
        break;

    case (1):
        if (g_gps->valid_read == true) {
            ap_system.GPS_light = !ap_system.GPS_light;                     // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
            if (ap_system.GPS_light) {
                digitalWriteFast(C_LED_PIN, LED_OFF);
            }else{
                digitalWriteFast(C_LED_PIN, LED_ON);
            }
            g_gps->valid_read = false;
        }
        break;

    default:
        digitalWriteFast(C_LED_PIN, LED_OFF);
        break;
    }
}

static void update_motor_light(void)
{
    if(motors.armed() == false) {
        ap_system.motor_light = !ap_system.motor_light;

        // blink
        if(ap_system.motor_light) {
            digitalWriteFast(A_LED_PIN, LED_ON);
        }else{
            digitalWriteFast(A_LED_PIN, LED_OFF);
        }
    }else{
        if(!ap_system.motor_light) {
            ap_system.motor_light = true;
            digitalWriteFast(A_LED_PIN, LED_ON);
        }
    }
}

static void dancing_light()
{
    static uint8_t step;

    if (step++ == 3)
        step = 0;

    switch(step)
    {
    case 0:
        digitalWriteFast(C_LED_PIN, LED_OFF);
        digitalWriteFast(A_LED_PIN, LED_ON);
        break;

    case 1:
        digitalWriteFast(A_LED_PIN, LED_OFF);
        digitalWriteFast(B_LED_PIN, LED_ON);
        break;

    case 2:
        digitalWriteFast(B_LED_PIN, LED_OFF);
        digitalWriteFast(C_LED_PIN, LED_ON);
        break;
    }
}
static void clear_leds()
{
    digitalWriteFast(A_LED_PIN, LED_OFF);
    digitalWriteFast(B_LED_PIN, LED_OFF);
    digitalWriteFast(C_LED_PIN, LED_OFF);
    ap_system.motor_light = false;
    led_mode = NORMAL_LEDS;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//	Copter LEDS by Robert Lefebvre
//	Based on the work of U4eake, Bill Sanford, Max Levine, and Oliver
//	g.copter_leds_mode controls the copter leds function via bitmath
//	Zeroeth bit turns motor leds on and off:                                00000001
//	First bit turns GPS function on and off:                                00000010
//	Second bit turns Aux function on and off:                               00000100
//	Third bit turns on Beeper (legacy Piezo) function:                      00001000
//	Fourth bit toggles between Fast Flash or Oscillate on Low Battery:      00010000		(0) does Fast Flash, (1) does Oscillate
//	Fifth bit causes motor LEDs to Nav Blink:                               00100000
//	Sixth bit causes GPS LEDs to Nav Blink:                                 01000000
//	This code is written in order to be backwards compatible with the old Motor_LEDS code
//	I hope to include at least some of the Show_LEDS code in the future
//	copter_leds_GPS_blink controls the blinking of the GPS LEDS
//	copter_leds_motor_blink controls the blinking of the motor LEDS
//	Piezo Code and beeps once on Startup to verify operation
//	Piezo Enables Tone on reaching low battery or current alert
/////////////////////////////////////////////////////////////////////////////////////////////


#if COPTER_LEDS == ENABLED
static void update_copter_leds(void)
{
    if (g.copter_leds_mode == 0) {
        copter_leds_reset();                                        //method of reintializing LED state
    }

    if ( bitRead(g.copter_leds_mode, 0) ) {
        if (motors.armed() == true) {
            if (ap.low_battery == true) {
                if ( bitRead(g.copter_leds_mode, 4 )) {
                    copter_leds_oscillate();                        //if motors are armed, but battery level is low, motor leds fast blink
                } else {
                    copter_leds_fast_blink();                       //if motors are armed, but battery level is low, motor leds oscillate
                }
            } else if ( !bitRead(g.copter_leds_mode, 5 ) ) {
                copter_leds_on();                                   //if motors are armed, battery level OK, all motor leds ON
            } else if ( bitRead(g.copter_leds_mode, 5 ) ) {
                if ( copter_leds_nav_blink >0 ) {
                    copter_leds_slow_blink();                       //if nav command was seen, blink LEDs.
                } else {
                    copter_leds_on();
                }
            }
        } else {
            copter_leds_slow_blink();                               //if motors are not armed, blink motor leds
        }
    }

    if ( bitRead(g.copter_leds_mode, 1) ) {

        // GPS LED on if we have a fix or Blink GPS LED if we are receiving data
        // ---------------------------------------------------------------------
        switch (g_gps->status()) {

        case (2):
            if(ap.home_is_set) {
                if ( !bitRead(g.copter_leds_mode, 6 ) ) {
                    copter_leds_GPS_on();							//Turn GPS LEDs on when gps has valid fix AND home is set
                } else if (bitRead(g.copter_leds_mode, 6 ) ) {
                    if ( copter_leds_nav_blink >0 ) {
                        copter_leds_GPS_slow_blink();               //if nav command was seen, blink LEDs.
                    } else {
                        copter_leds_GPS_on();
                    }
                }
            } else {
                copter_leds_GPS_fast_blink();                       //if GPS has fix, but home is not set, blink GPS LED fast
            }
            break;

        case (1):

            copter_leds_GPS_slow_blink();                           //if GPS has valid reads, but no fix, blink GPS LED slow

            break;

        default:
            copter_leds_GPS_off();                                  //if no valid GPS signal, turn GPS LED off
            break;
        }
    }

    if ( bitRead(g.copter_leds_mode, 2) ) {
        if (200 <= g.rc_7.control_in && g.rc_7.control_in < 400) {
            copter_leds_aux_on();                                   //if sub-control of Ch7 is high, turn Aux LED on
        } else if (g.rc_7.control_in < 200) {
            copter_leds_aux_off();                                  //if sub-control of Ch7 is low, turn Aux LED off
        }
    }

}

static void copter_leds_reset(void) {
    digitalWriteFast(COPTER_LED_1, COPTER_LED_OFF);
    digitalWriteFast(COPTER_LED_2, COPTER_LED_OFF);
    digitalWriteFast(COPTER_LED_3, COPTER_LED_OFF);
    digitalWriteFast(COPTER_LED_4, COPTER_LED_OFF);
    digitalWriteFast(COPTER_LED_5, COPTER_LED_OFF);
    digitalWriteFast(COPTER_LED_6, COPTER_LED_OFF);
    digitalWriteFast(COPTER_LED_7, COPTER_LED_OFF);
    digitalWriteFast(COPTER_LED_8, COPTER_LED_OFF);
}

static void copter_leds_on(void) {
    if ( !bitRead(g.copter_leds_mode, 2) ) {
        digitalWriteFast(COPTER_LED_1, COPTER_LED_ON);
    }
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    if ( !bitRead(g.copter_leds_mode, 3) ) {
        digitalWriteFast(COPTER_LED_2, COPTER_LED_ON);
    }
 #else
    digitalWriteFast(COPTER_LED_2, COPTER_LED_ON);
 #endif
    if ( !bitRead(g.copter_leds_mode, 1) ) {
        digitalWriteFast(COPTER_LED_3, COPTER_LED_ON);
    }
    digitalWriteFast(COPTER_LED_4, COPTER_LED_ON);
    digitalWriteFast(COPTER_LED_5, COPTER_LED_ON);
    digitalWriteFast(COPTER_LED_6, COPTER_LED_ON);
    digitalWriteFast(COPTER_LED_7, COPTER_LED_ON);
    digitalWriteFast(COPTER_LED_8, COPTER_LED_ON);
}

static void copter_leds_off(void) {
    if ( !bitRead(g.copter_leds_mode, 2) ) {
        digitalWriteFast(COPTER_LED_1, COPTER_LED_OFF);
    }
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    if ( !bitRead(g.copter_leds_mode, 3) ) {
        digitalWriteFast(COPTER_LED_2, COPTER_LED_OFF);
    }
 #else
    digitalWriteFast(COPTER_LED_2, COPTER_LED_OFF);
 #endif
    if ( !bitRead(g.copter_leds_mode, 1) ) {
        digitalWriteFast(COPTER_LED_3, COPTER_LED_OFF);
    }
    digitalWriteFast(COPTER_LED_4, COPTER_LED_OFF);
    digitalWriteFast(COPTER_LED_5, COPTER_LED_OFF);
    digitalWriteFast(COPTER_LED_6, COPTER_LED_OFF);
    digitalWriteFast(COPTER_LED_7, COPTER_LED_OFF);
    digitalWriteFast(COPTER_LED_8, COPTER_LED_OFF);
}

static void copter_leds_slow_blink(void) {
    copter_leds_motor_blink++;                                                                                                                                                  // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_motor_blink && copter_leds_motor_blink < 6 ) {                                                                                                 // when the counter reaches 5 (1/2 sec), then toggle the leds
        copter_leds_off();
        if (  bitRead(g.copter_leds_mode, 5 ) && !bitRead(g.copter_leds_mode, 6 ) && copter_leds_nav_blink >0 ) {               // if blinking is called by the Nav Blinker...
            copter_leds_nav_blink--;                                                                                                                                                                            // decrement the Nav Blink counter
        }
    }else if (5 < copter_leds_motor_blink && copter_leds_motor_blink < 11) {
        copter_leds_on();
    }
    else copter_leds_motor_blink = 0;                                                                                                                                                   // start blink cycle again
}

static void copter_leds_fast_blink(void) {
    copter_leds_motor_blink++;                                                                                                                                                  // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_motor_blink && copter_leds_motor_blink < 3 ) {                                                                                                 // when the counter reaches 3 (1/5 sec), then toggle the leds
        copter_leds_on();
    }else if (2 < copter_leds_motor_blink && copter_leds_motor_blink < 5) {
        copter_leds_off();
    }
    else copter_leds_motor_blink = 0;                                                                                                                                                   // start blink cycle again
}


static void copter_leds_oscillate(void) {
    copter_leds_motor_blink++;                                                                                                                                                  // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_motor_blink && copter_leds_motor_blink < 3 ) {                                                                                                 // when the counter reaches 3 (1/5 sec), then toggle the leds
        if ( !bitRead(g.copter_leds_mode, 2) ) {
            digitalWriteFast(COPTER_LED_1, COPTER_LED_ON);
        }
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
        if ( !bitRead(g.copter_leds_mode, 3) ) {
            digitalWriteFast(COPTER_LED_2, COPTER_LED_ON);
        }
 #else
        digitalWriteFast(COPTER_LED_2, COPTER_LED_ON);
 #endif
        if ( !bitRead(g.copter_leds_mode, 1) ) {
            digitalWriteFast(COPTER_LED_3, COPTER_LED_OFF);
        }
        digitalWriteFast(COPTER_LED_4, COPTER_LED_OFF);
        digitalWriteFast(COPTER_LED_5, COPTER_LED_ON);
        digitalWriteFast(COPTER_LED_6, COPTER_LED_ON);
        digitalWriteFast(COPTER_LED_7, COPTER_LED_OFF);
        digitalWriteFast(COPTER_LED_8, COPTER_LED_OFF);
    }else if (2 < copter_leds_motor_blink && copter_leds_motor_blink < 5) {
        if ( !bitRead(g.copter_leds_mode, 2) ) {
            digitalWriteFast(COPTER_LED_1, COPTER_LED_OFF);
        }
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
        if ( !bitRead(g.copter_leds_mode, 3) ) {
            digitalWriteFast(COPTER_LED_2, COPTER_LED_OFF);
        }
 #else
        digitalWriteFast(COPTER_LED_2, COPTER_LED_OFF);
 #endif
        if ( !bitRead(g.copter_leds_mode, 1) ) {
            digitalWriteFast(COPTER_LED_3, COPTER_LED_ON);
        }
        digitalWriteFast(COPTER_LED_4, COPTER_LED_ON);
        digitalWriteFast(COPTER_LED_5, COPTER_LED_OFF);
        digitalWriteFast(COPTER_LED_6, COPTER_LED_OFF);
        digitalWriteFast(COPTER_LED_7, COPTER_LED_ON);
        digitalWriteFast(COPTER_LED_8, COPTER_LED_ON);
    }
    else copter_leds_motor_blink = 0;                                           // start blink cycle again
}



static void copter_leds_GPS_on(void) {
    digitalWriteFast(COPTER_LED_3, COPTER_LED_ON);
}

static void copter_leds_GPS_off(void) {
    digitalWriteFast(COPTER_LED_3, COPTER_LED_OFF);
}

static void copter_leds_GPS_slow_blink(void) {
    copter_leds_GPS_blink++;                                                    // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_GPS_blink && copter_leds_GPS_blink < 6 ) {             // when the counter reaches 5 (1/2 sec), then toggle the leds
        copter_leds_GPS_off();
        if (  bitRead(g.copter_leds_mode, 6 ) && copter_leds_nav_blink >0 ) {   // if blinking is called by the Nav Blinker...
            copter_leds_nav_blink--;                                                                                                                    // decrement the Nav Blink counter
        }
    }else if (5 < copter_leds_GPS_blink && copter_leds_GPS_blink < 11) {
        copter_leds_GPS_on();
    }
    else copter_leds_GPS_blink = 0;                                             // start blink cycle again
}

static void copter_leds_GPS_fast_blink(void) {
    copter_leds_GPS_blink++;                                                    // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_GPS_blink && copter_leds_GPS_blink < 3 ) {             // when the counter reaches 3 (1/5 sec), then toggle the leds
        copter_leds_GPS_off();
    }else if (2 < copter_leds_GPS_blink && copter_leds_GPS_blink < 5) {
        copter_leds_GPS_on();
    }
    else copter_leds_GPS_blink = 0;                                             // start blink cycle again
}

static void copter_leds_aux_off(void){
    digitalWriteFast(COPTER_LED_1, COPTER_LED_OFF);
}

static void copter_leds_aux_on(void){
    digitalWriteFast(COPTER_LED_1, COPTER_LED_ON);
}

void piezo_on(){
    digitalWriteFast(PIEZO_PIN,HIGH);
}

void piezo_off(){
    digitalWriteFast(PIEZO_PIN,LOW);
}

void piezo_beep(){                                                              // Note! This command should not be used in time sensitive loops
    piezo_on();
    delay(100);
    piezo_off();
}

#endif                  //COPTER_LEDS
#line 1 "./Firmware/ACopter32/limits.pde"
// Main state machine loop for AP_Limits. Called from slow or superslow loop.

#if AP_LIMITS == ENABLED

uint8_t lim_state = 0, lim_old_state = 0;

void set_recovery_home_alt() {

    uint32_t return_altitude_cm_ahl = 0;     // in centimeters above home level.
    uint32_t amin_meters_ahl, amax_meters_ahl;

    // for flying vehicles only
    if (altitude_limit.enabled()) {

        amin_meters_ahl = (uint32_t) (altitude_limit.min_alt());
        amax_meters_ahl = (uint32_t) (altitude_limit.max_alt());

        // See if we have a meaningful setting
        if (amax_meters_ahl && ((amax_meters_ahl - amin_meters_ahl) > 1)) {
            // there is a max_alt set
            // set a return altitude that is halfway between the minimum and maximum altitude setting.
            // return_altitude is in centimeters, not meters, so we multiply
            return_altitude_cm_ahl = (uint32_t) (home.alt + (100 * (uint16_t) ((amax_meters_ahl - amin_meters_ahl) / 2)));
        }
    } else {
        return_altitude_cm_ahl = (uint32_t) (home.alt + g.rtl_altitude);
    }
    // final sanity check
    // if our return is less than 4 meters from ground, set it to 4m, to clear "people" height.
    if ((return_altitude_cm_ahl - (uint32_t) home.alt) < 400) {
        return_altitude_cm_ahl = home.alt + 400;
    }
    guided_WP.id = 0;
    guided_WP.p1 = 0;
    guided_WP.options = 0;
    guided_WP.lat = home.lat;
    guided_WP.lng = home.lng;
    guided_WP.alt = return_altitude_cm_ahl;
}

static void limits_loop() {

    lim_state = limits.state();

    // Use limits channel to determine LIMITS_ENABLED or LIMITS_DISABLED state
    if (lim_state != LIMITS_DISABLED  && limits.channel() !=0 
            && hal.rcin->read(limits.channel()-1) < LIMITS_ENABLE_PWM) {
        limits.set_state(LIMITS_DISABLED);
    }
    else if (lim_state == LIMITS_DISABLED && limits.channel() !=0
            && hal.rcin->read(limits.channel()-1) >= LIMITS_ENABLE_PWM) {
        limits.set_state(LIMITS_ENABLED);
    }

    if ((uint32_t) millis() - (uint32_t) limits.last_status_update > 1000) {     // more than a second has passed - time for an update
        gcs_send_message(MSG_LIMITS_STATUS);
    }

    if (lim_state != lim_old_state) {     // state changed
        lim_old_state = lim_state;         // we only use lim_oldstate here, for reporting purposes. So, reset it.
        gcs_send_message(MSG_LIMITS_STATUS);

        if (limits.debug()) switch (lim_state) {
            case LIMITS_INIT: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: INIT")); break;
            case LIMITS_DISABLED: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: DISABLED")); break;
            case LIMITS_ENABLED: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: ENABLED")); break;
            case LIMITS_TRIGGERED: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: TRIGGERED")); break;
            case LIMITS_RECOVERING: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: RECOVERING")); break;
            case LIMITS_RECOVERED: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: RECOVERED")); break;
            default: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: UNKNOWN")); break;
            }
    }

    switch (limits.state()) {

    // have not initialized yet
    case LIMITS_INIT:
        if (limits.init()) {                 // initialize system

            // See what the "master" on/off swith is and go to the appropriate start state
            if (!limits.enabled()) {
                limits.set_state(LIMITS_DISABLED);
            }
            else {
                limits.set_state(LIMITS_ENABLED);
            }
        }
        break;

    // We have been switched off
    case LIMITS_DISABLED:

        // check if we have been switched on
        if (limits.enabled()) {
            limits.set_state(LIMITS_ENABLED);
            break;
        }
        break;

    // Limits module is enabled
    case LIMITS_ENABLED:

        // check if we've been switched off
        if (!limits.enabled()) {
            limits.set_state(LIMITS_DISABLED);
            break;
        }

        // Until motors are armed, do nothing, just wait in ENABLED state
        if (!motors.armed()) {

            // we are waiting for motors to arm
            // do nothing
            break;
        }

        bool required_only;

        required_only = (limits.last_clear == 0);                 // if we haven't yet 'cleared' all limits, check required limits only

        // check if any limits have been breached and trigger if they have
        if  (limits.check_triggered(required_only)) {

            //
            // TRIGGER - BREACH OF LIMITS
            //
            // make a note of which limits triggered, so if we know if we recovered them
            limits.mods_recovering = limits.mods_triggered;

            limits.last_action = 0;
            limits.last_trigger = millis();
            limits.breach_count++;

            limits.set_state(LIMITS_TRIGGERED);
            break;
        }

        if (motors.armed() && limits.enabled() && !limits.mods_triggered) {

            // All clear.
	    if (limits.debug()) gcs_send_text_P(SEVERITY_LOW, PSTR("Limits - All Clear"));
            limits.last_clear = millis();
        }

        break;

    // Limits have been triggered
    case LIMITS_TRIGGERED:

        // check if we've been switched off
        if (!limits.enabled()) {
            limits.set_state(LIMITS_DISABLED);
            break;
        }

#if LIMITS_TRIGGERED_PIN > 0
        digitalWrite(LIMITS_TRIGGERED_PIN, HIGH);
#endif

        if (limits.debug()) {
		if (limits.mods_triggered & LIMIT_GPSLOCK) gcs_send_text_P(SEVERITY_LOW, PSTR("!GPSLock"));
		if (limits.mods_triggered & LIMIT_GEOFENCE) gcs_send_text_P(SEVERITY_LOW, PSTR("!Geofence"));
		if (limits.mods_triggered & LIMIT_ALTITUDE) gcs_send_text_P(SEVERITY_LOW, PSTR("!Altitude"));
        }

        // If the motors are not armed, we have triggered pre-arm checks. Do nothing
        if (motors.armed() == false) {
            limits.set_state(LIMITS_ENABLED);                     // go back to checking limits
            break;
        }

        // If we are triggered but no longer in breach, that means we recovered
        // somehow, via auto recovery or pilot action
        if (!limits.check_all()) {
            limits.last_recovery = millis();
            limits.set_state(LIMITS_RECOVERED);
            break;
        }
        else {
            limits.set_state(LIMITS_RECOVERING);
            limits.last_action = 0;                     // reset timer
            // We are about to take action on a real breach. Make sure we notify immediately
            gcs_send_message(MSG_LIMITS_STATUS);
            break;
        }
        break;

    // Take action to recover
    case LIMITS_RECOVERING:
        // If the motors are not armed, we have triggered pre-arm checks. Do nothing
        if (motors.armed() == false) {
            limits.set_state(LIMITS_ENABLED);                     // go back to checking limits
            break;
        }

        // check if we've been switched off
        if (!limits.enabled() && limits.old_mode_switch == oldSwitchPosition) {
            limits.old_mode_switch = 0;
            reset_control_switch();
            limits.set_state(LIMITS_DISABLED);
            break;
        }

        // Still need action?
        if (limits.check_all() == 0) {                 // all triggers clear
            limits.set_state(LIMITS_RECOVERED);
            break;
        }

        if (limits.mods_triggered != limits.mods_recovering)  {                 // if any *new* triggers, hit the trigger again
            //
            // TRIGGER - BREACH OF LIMITS
            //
            // make a note of which limits triggered, so if we know if we recovered them
            limits.mods_recovering = limits.mods_triggered;

            limits.last_action = 0;
            limits.last_trigger = millis();
            limits.breach_count++;

            limits.set_state(LIMITS_TRIGGERED);
            limits.set_state(LIMITS_TRIGGERED);
            break;
        }

        // Recovery Action
        // if there was no previous action, take action, take note of time  send GCS.
        if (limits.last_action == 0) {

            // save mode switch
            limits.old_mode_switch = oldSwitchPosition;


//				// Take action
//				// This ensures no "radical" RTL, like a full throttle take-off,happens if something triggers at ground level
//				if ((uint32_t) current_loc.alt < ((uint32_t)home.alt * 200) ) { // we're under 2m (200cm), already at "people" height or on the ground
//					if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits Action: near ground - do nothing"));
//					// TODO: Will this work for a plane? Does it make sense in general?
//
//					//set_mode(LAND);
//					limits.last_action = millis(); // start counter
//				    gcs_send_message(MSG_LIMITS_STATUS);
//
//					break;
//				}


            // TODO: This applies only to planes - hold for porting
//					if (control_mode == MANUAL && g.auto_trim) {
//					            // make sure we don't auto trim the surfaces on this change
//					            control_mode = STABILIZE;
//					}


            switch (limits.recmode()) {

            case 0:                                     // RTL mode

                if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits Action - RTL"));

                set_mode(RTL);
                limits.last_action = millis();
                gcs_send_message(MSG_LIMITS_STATUS);
                break;

            case 1:                                     // Bounce mode

                if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits Action - bounce mode, POSITION"));
                // ALT_HOLD gives us yaw hold, roll& pitch hold and throttle hold.
                // It is like position hold, but without manual throttle control.

                //set_recovery_home_alt();
                set_mode(POSITION);
                set_throttle_mode(THROTTLE_AUTO);
                limits.last_action = millis();
                gcs_send_message(MSG_LIMITS_STATUS);
                break;

            }
            break;
        }


        // In bounce mode, take control for 3 seconds, and then wait for the pilot to make us "safe".
        // If the vehicle does not recover, the escalation action will trigger.
        if (limits.recmode() == 1) {

            if (control_mode == POSITION && ((uint32_t)millis() - (uint32_t)limits.last_action) > 3000) {
                if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits Recovery Bounce: Returning control to pilot"));
                set_mode(STABILIZE);
            } else if (control_mode == STABILIZE && ((uint32_t)millis() - (uint32_t)limits.last_action) > 6000) {
                // after 3 more seconds, reset action counter to take action again
                limits.last_action = 0;
            }
        }

        // ESCALATE We have not recovered after 2 minutes of recovery action

        if (((uint32_t)millis() - (uint32_t)limits.last_action) > 120000 ) {

            // TODO: Secondary recovery
            if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits Recovery Escalation: RTL"));
            set_mode(RTL);
            limits.last_action = millis();
            break;
        }
        break;

    // Have recovered, relinquish control and re-enable
    case LIMITS_RECOVERED:


        // check if we've been switched off
        if (!limits.enabled()) {
            limits.set_state(LIMITS_DISABLED);
            break;
        }

#if LIMITS_TRIGGERED_PIN > 0
        digitalWrite(LIMITS_TRIGGERED_PIN, LOW);
#endif

        // Reset action counter
        limits.last_action = 0;

        if (((uint32_t)millis() - (uint32_t)limits.last_recovery) > (uint32_t)(limits.safetime() * 1000)) {                 // Wait "safetime" seconds of recovery before we give back control

            // Our recovery action worked.
            limits.set_state(LIMITS_ENABLED);

            // Switch to stabilize
            if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - Returning controls"));
            set_mode(STABILIZE);                            limits.last_recovery = millis();

            break;
        }
        break;

    default:
        if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits: unknown state"));
        break;
    }
}

// This function below, should really be in the AP_Limits class, but it is impossible to untangle the mavlink includes.

void limits_send_mavlink_status(mavlink_channel_t chan) {

    limits.last_status_update = millis();

    if (limits.enabled()) {
        mavlink_msg_limits_status_send(chan,
                                       (uint8_t) limits.state(),
                                       (uint32_t) limits.last_trigger,
                                       (uint32_t) limits.last_action,
                                       (uint32_t) limits.last_recovery,
                                       (uint32_t) limits.last_clear,
                                       (uint16_t) limits.breach_count,
                                       (LimitModuleBits) limits.mods_enabled,
                                       (LimitModuleBits) limits.mods_required,
                                       (LimitModuleBits) limits.mods_triggered);
    }
}

#endif
#line 1 "./Firmware/ACopter32/motors.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// 10 = 1 second
#define ARM_DELAY 20
#define DISARM_DELAY 20
#define AUTO_TRIM_DELAY 100


// called at 10hz
static void arm_motors()
{
    static int16_t arming_counter;

    // don't allow arming/disarming in anything but manual
    if (g.rc_3.control_in > 0) {
        arming_counter = 0;
        return;
    }

    if ((control_mode > ACRO) && ((control_mode != TOY_A) && (control_mode != TOY_M))) {
        arming_counter = 0;
        return;
    }
	
	#if FRAME_CONFIG == HELI_FRAME
	if ((motors.rsc_mode > 0) && (g.rc_8.control_in >= 10)){
		arming_counter = 0;
		return;
	}
	#endif  // HELI_FRAME

#if TOY_EDF == ENABLED
    int16_t tmp = g.rc_1.control_in;
#else
    int16_t tmp = g.rc_4.control_in;
#endif

    // full right
    if (tmp > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if( arming_counter <= AUTO_TRIM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors.armed()) {
////////////////////////////////////////////////////////////////////////////////
// Experimental AP_Limits library - set constraints, limits, fences, minima, maxima on various parameters
////////////////////////////////////////////////////////////////////////////////
#if AP_LIMITS == ENABLED
            if (limits.enabled() && limits.required()) {
                gcs_send_text_P(SEVERITY_LOW, PSTR("Limits - Running pre-arm checks"));

                // check only pre-arm required modules
                if (limits.check_required()) {
                    gcs_send_text_P(SEVERITY_LOW, PSTR("ARMING PREVENTED - Limit Breached"));
                    limits.set_state(LIMITS_TRIGGERED);
                    gcs_send_message(MSG_LIMITS_STATUS);

                    arming_counter++;                                 // restart timer by cycling
                }else{
                    init_arm_motors();
                }
            }else{
                init_arm_motors();
            }
#else  // without AP_LIMITS, just arm motors
            init_arm_motors();
#endif //AP_LIMITS_ENABLED
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors.armed()) {
            auto_trim_counter = 250;
        }

    // full left
    }else if (tmp < -4000) {

        // increase the counter to a maximum of 1 beyond the disarm delay
        if( arming_counter <= DISARM_DELAY ) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors.armed()) {
            init_disarm_motors();
        }

    // Yaw is centered so reset arming counter
    }else{
        arming_counter = 0;
    }
}


static void init_arm_motors()
{
	// arming marker
    // Flag used to track if we have armed the motors the first time.
    // This is used to decide if we should run the ground_start routine
    // which calibrates the IMU
    static bool did_ground_start = false;

    // disable failsafe because initialising everything takes a while
    failsafe_disable();

    //cliSerial->printf("\nARM\n");
#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
#endif

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we arm
    // the motors
    hal.uartA->set_blocking_writes(false);
    if (gcs3.initialised) {
        hal.uartC->set_blocking_writes(false);
    }

#if COPTER_LEDS == ENABLED
    if ( bitRead(g.copter_leds_mode, 3) ) {
        piezo_beep();
        delay(50);
        piezo_beep();
    }
#endif

    // Remember Orientation
    // --------------------
    init_simple_bearing();

    // Reset home position
    // -------------------
    if(ap.home_is_set)
        init_home();

    // all I terms are invalid
    // -----------------------
    reset_I_all();

    if(did_ground_start == false) {
        did_ground_start = true;
        startup_ground();
    }

#if HIL_MODE != HIL_MODE_ATTITUDE
    // read Baro pressure at ground -
    // this resets Baro for more accuracy
    //-----------------------------------
    init_barometer();
#endif

    // temp hack
    ap_system.motor_light = true;
    digitalWrite(A_LED_PIN, LED_ON);

    // go back to normal AHRS gains
    ahrs.set_fast_gains(false);
#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.set_fast_gains(false);
#endif

    // finally actually arm the motors
    motors.armed(true);
    set_armed(true);

    // reenable failsafe
    failsafe_enable();
}


static void init_disarm_motors()
{
#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("DISARMING MOTORS"));
#endif

    motors.armed(false);
    set_armed(false);

    motors.auto_armed(false);
    set_auto_armed(false);

    compass.save_offsets();

    g.throttle_cruise.save();

#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
    inertial_nav.save_params();
#endif

    // we are not in the air
    set_takeoff_complete(false);

#if COPTER_LEDS == ENABLED
    if ( bitRead(g.copter_leds_mode, 3) ) {
        piezo_beep();
    }
#endif

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);
#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.set_fast_gains(true);
#endif
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void
set_servos_4()
{
#if FRAME_CONFIG == TRI_FRAME
    // To-Do: implement improved stability patch for tri so that we do not need to limit throttle input to motors
    g.rc_3.servo_out = min(g.rc_3.servo_out, 800);
#endif
    motors.output();
}

#line 1 "./Firmware/ACopter32/navigation.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// update_navigation - checks for new GPS updates and invokes navigation routines
// called at 50hz
static void update_navigation()
{
    static uint32_t nav_last_update = 0;        // the system time of the last time nav was run update
    bool pos_updated = false;
    bool log_output = false;

#if INERTIAL_NAV_XY == ENABLED
    static uint8_t nav_counter = 0;             // used to slow down the navigation to 10hz

    // check for inertial nav updates
    if( inertial_nav.position_ok() ) {
        nav_counter++;
        if( nav_counter >= 5) {
            nav_counter = 0;

            // calculate time since nav controllers last ran
            dTnav = (float)(millis() - nav_last_update)/ 1000.0f;
            nav_last_update = millis();

            // prevent runnup in dTnav value
            dTnav = min(dTnav, 1.0f);

            // signal to run nav controllers
            pos_updated = true;

            // signal to create log entry
            log_output = true;
        }
    }
#else

    static uint32_t nav_last_gps_time = 0;      // the time according to the gps

    // check for new gps data
    if( g_gps->fix && g_gps->time != nav_last_gps_time ) {

        // used to calculate speed in X and Y, iterms
        // ------------------------------------------
        dTnav = (float)(millis() - nav_last_update)/ 1000.0f;
        nav_last_update = millis();

        // prevent runup from bad GPS
        dTnav = min(dTnav, 1.0f);

        // save GPS time
        nav_last_gps_time = g_gps->time;

        // signal to run nav controllers
        pos_updated = true;

        // signal to create log entry
        log_output = true;
    }
#endif

    // setup to calculate new navigation values and run controllers if
    // we've received a position update
    if( pos_updated ) {

        nav_updates.need_velpos = 1;
        nav_updates.need_dist_bearing = 1;
        nav_updates.need_nav_controllers = 1;
        nav_updates.need_nav_pitch_roll = 1;

        // update log
        if (log_output && (g.log_bitmask & MASK_LOG_NTUN) && motors.armed()) {
            Log_Write_Nav_Tuning();
        }
    }

    // reduce nav outputs to zero if we have not seen a position update in 2 seconds
    if( millis() - nav_last_update > 2000 ) {
        // after 12 reads we guess we may have lost GPS signal, stop navigating
        // we have lost GPS signal for a moment. Reduce our error to avoid flyaways
        auto_roll  >>= 1;
        auto_pitch >>= 1;
    }
}

/*
  run navigation updates from nav_updates. Only run one at a time to
  prevent too much cpu usage hurting the main loop
 */
static void run_nav_updates(void)
{
    if (nav_updates.need_velpos) {
        calc_velocity_and_position();
        nav_updates.need_velpos = 0;
    } else if (nav_updates.need_dist_bearing) {
        calc_distance_and_bearing();
        nav_updates.need_dist_bearing = 0;
    } else if (nav_updates.need_nav_controllers) {
        run_autopilot();
        update_nav_mode();
        nav_updates.need_nav_controllers = 0;
    } else if (nav_updates.need_nav_pitch_roll) {
        calc_nav_pitch_roll();
        nav_updates.need_nav_pitch_roll = 0;
    }
}


//*******************************************************************************************************
// calc_velocity_and_filtered_position - velocity in lon and lat directions calculated from GPS position
//       and accelerometer data
// lon_speed expressed in cm/s.  positive numbers mean moving east
// lat_speed expressed in cm/s.  positive numbers when moving north
// Note: we use gps locations directly to calculate velocity instead of asking gps for velocity because
//       this is more accurate below 1.5m/s
// Note: even though the positions are projected using a lead filter, the velocities are calculated
//       from the unaltered gps locations.  We do not want noise from our lead filter affecting velocity
//*******************************************************************************************************
static void calc_velocity_and_position(){
#if INERTIAL_NAV_XY == ENABLED
    if( inertial_nav.position_ok() ) {
        // pull velocity from interial nav library
        lon_speed = inertial_nav.get_longitude_velocity();
        lat_speed = inertial_nav.get_latitude_velocity();

        // pull position from interial nav library
        current_loc.lng = inertial_nav.get_longitude();
        current_loc.lat = inertial_nav.get_latitude();
    }
#else
    static int32_t last_gps_longitude = 0;
    static int32_t last_gps_latitude  = 0;

    // initialise last_longitude and last_latitude
    if( last_gps_longitude == 0 && last_gps_latitude == 0 ) {
        last_gps_longitude = g_gps->longitude;
        last_gps_latitude = g_gps->latitude;
    }

    // this speed is ~ in cm because we are using 10^7 numbers from GPS
    float tmp = 1.0f/dTnav;

    // calculate velocity
    lon_speed  = (float)(g_gps->longitude - last_gps_longitude)  * scaleLongDown * tmp;
    lat_speed  = (float)(g_gps->latitude  - last_gps_latitude)  * tmp;

    // calculate position from gps + expected travel during gps_lag
    current_loc.lng = xLeadFilter.get_position(g_gps->longitude, lon_speed, g_gps->get_lag());
    current_loc.lat = yLeadFilter.get_position(g_gps->latitude,  lat_speed, g_gps->get_lag());

    // store gps lat and lon values for next iteration
    last_gps_longitude  = g_gps->longitude;
    last_gps_latitude   = g_gps->latitude;
#endif
}

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void calc_distance_and_bearing()
{
    // waypoint distance from plane in cm
    // ---------------------------------------
    wp_distance     = get_distance_cm(&current_loc, &next_WP);
    home_distance   = get_distance_cm(&current_loc, &home);

    // wp_bearing is bearing to next waypoint
    // --------------------------------------------
    wp_bearing          = get_bearing_cd(&current_loc, &next_WP);
    home_bearing        = get_bearing_cd(&current_loc, &home);

    // update super simple bearing (if required) because it relies on home_bearing
    update_super_simple_beading();

    // bearing to target (used when yaw_mode = YAW_LOOK_AT_LOCATION)
    yaw_look_at_WP_bearing = get_bearing_cd(&current_loc, &yaw_look_at_WP);
}

static void calc_location_error(struct Location *next_loc)
{
    /*
     *  Becuase we are using lat and lon to do our distance errors here's a quick chart:
     *  100     = 1m
     *  1000    = 11m	 = 36 feet
     *  1800    = 19.80m = 60 feet
     *  3000    = 33m
     *  10000   = 111m
     */

    // X Error
    long_error      = (float)(next_loc->lng - current_loc.lng) * scaleLongDown;       // 500 - 0 = 500 Go East

    // Y Error
    lat_error       = next_loc->lat - current_loc.lat;                                                          // 500 - 0 = 500 Go North
}

// run_autopilot - highest level call to process mission commands
static void run_autopilot()
{
    switch( control_mode ) {
        case AUTO:
            // majority of command logic is in commands_logic.pde
            verify_commands();
            break;
        case GUIDED:
            // switch to loiter once we've reached the target location and altitude
            if(verify_nav_wp()) {
                set_nav_mode(NAV_LOITER);
            }
        case RTL:
            verify_RTL();
            break;
    }
}

// set_nav_mode - update nav mode and initialise any variables as required
static bool set_nav_mode(uint8_t new_nav_mode)
{
    // boolean to ensure proper initialisation of nav modes
    bool nav_initialised = false;

    // return immediately if no change
    if( new_nav_mode == nav_mode ) {
        return true;
    }

    switch( new_nav_mode ) {

        case NAV_NONE:
            nav_initialised = true;
            break;

        case NAV_CIRCLE:
            // start circling around current location
            set_next_WP(&current_loc);
            circle_WP       = next_WP;
            circle_angle    = 0;
            nav_initialised = true;
            break;

        case NAV_LOITER:
            // set target to current position
            next_WP.lat = current_loc.lat;
            next_WP.lng = current_loc.lng;
            nav_initialised = true;
            break;

        case NAV_WP:
            nav_initialised = true;
            break;

        case NAV_LOITER_INAV:
            loiter_set_target(inertial_nav.get_latitude_diff(), inertial_nav.get_longitude_diff());
            nav_initialised = true;
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( nav_initialised ) {
        nav_mode = new_nav_mode;
    }

    // return success or failure
    return nav_initialised;
}

// update_nav_mode - run navigation controller based on nav_mode
static void update_nav_mode()
{
    int16_t loiter_delta;
    int16_t speed;

    switch( nav_mode ) {

        case NAV_NONE:
            // do nothing
            break;

        case NAV_CIRCLE:
            // check if we have missed the WP
            loiter_delta = (wp_bearing - old_wp_bearing)/100;

            // reset the old value
            old_wp_bearing = wp_bearing;

            // wrap values
            if (loiter_delta > 180) loiter_delta -= 360;
            if (loiter_delta < -180) loiter_delta += 360;

            // sum the angle around the WP
            loiter_sum += loiter_delta;

            circle_angle += (circle_rate * dTnav);

            //1 degree = 0.0174532925 radians

            // wrap
            if (circle_angle > 6.28318531f)
                circle_angle -= 6.28318531f;

            next_WP.lng = circle_WP.lng + (g.circle_radius * 100 * cosf(1.57f - circle_angle) * scaleLongUp);
            next_WP.lat = circle_WP.lat + (g.circle_radius * 100 * sinf(1.57f - circle_angle));

            // use error as the desired rate towards the target
            // nav_lon, nav_lat is calculated

            // if the target location is >4m use waypoint controller
            if(wp_distance > 400) {
                calc_nav_rate(get_desired_speed(g.waypoint_speed_max));
            }else{
                // calc the lat and long error to the target
                calc_location_error(&next_WP);
                // call loiter controller
                calc_loiter(long_error, lat_error);
            }
            break;

        case NAV_LOITER:
            // check if user is overriding the loiter controller
            if((abs(g.rc_2.control_in) + abs(g.rc_1.control_in)) > 500) {
                if(wp_distance > 500){
                    ap.loiter_override = true;
                }
            }

            // check if user has release sticks
            if(ap.loiter_override) {
                if(g.rc_2.control_in == 0 && g.rc_1.control_in == 0) {
                    ap.loiter_override  = false;
                    // reset LOITER to current position
                    next_WP.lat = current_loc.lat;
                    next_WP.lng = current_loc.lng;
                }
                // We bring copy over our Iterms for wind control, but we don't navigate
                nav_lon = g.pid_loiter_rate_lon.get_integrator();
                nav_lat = g.pid_loiter_rate_lon.get_integrator();
                nav_lon = constrain(nav_lon, -2000, 2000);
                nav_lat = constrain(nav_lat, -2000, 2000);
            }else{
                // calc error to target
                calc_location_error(&next_WP);
                // use error as the desired rate towards the target
                calc_loiter(long_error, lat_error);
            }
            break;

        case NAV_WP:
            // calc position error to target
            calc_location_error(&next_WP);
            // calc speed to target
            speed = get_desired_speed(g.waypoint_speed_max);
            // use error as the desired rate towards the target
            calc_nav_rate(speed);
            break;

        case NAV_LOITER_INAV:
            get_loiter_pos_lat_lon(loiter_lat_from_home_cm, loiter_lon_from_home_cm, 0.1f);
            break;
    }

    /*
    // To-Do: check that we haven't broken toy mode
    case TOY_A:
    case TOY_M:
        set_nav_mode(NAV_NONE);
        update_nav_wp();
        break;
    }
    */
}

static bool check_missed_wp()
{
    int32_t temp;
    temp = wp_bearing - original_wp_bearing;
    temp = wrap_180(temp);
    return (labs(temp) > 9000);         // we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////
// Loiter controller (based on GPS position)
////////////////////////////////////////////////////////////////
#define NAV_ERR_MAX 600
#define NAV_RATE_ERR_MAX 250
static void calc_loiter(int16_t x_error, int16_t y_error)
{
    int32_t p,i,d;                                              // used to capture pid values for logging
    int32_t output;
    int32_t x_target_speed, y_target_speed;

    // East / West
    x_target_speed  = g.pi_loiter_lon.get_p(x_error);                           // calculate desired speed from lon error

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_LOITER_KP || g.radio_tuning == CH6_LOITER_KI) ) {
        Log_Write_PID(CH6_LOITER_KP, x_error, x_target_speed, 0, 0, x_target_speed, tuning_value);
    }
#endif

    // calculate rate error
    x_rate_error    = x_target_speed - lon_speed;                           // calc the speed error

    p                               = g.pid_loiter_rate_lon.get_p(x_rate_error);
    i                               = g.pid_loiter_rate_lon.get_i(x_rate_error + x_error, dTnav);
    d                               = g.pid_loiter_rate_lon.get_d(x_error, dTnav);
    d                               = constrain(d, -2000, 2000);

    // get rid of noise
    if(abs(lon_speed) < 50) {
        d = 0;
    }

    output                  = p + i + d;
    nav_lon                 = constrain(output, -4500, 4500); // constrain max angle to 45 degrees

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_LOITER_RATE_KP || g.radio_tuning == CH6_LOITER_RATE_KI || g.radio_tuning == CH6_LOITER_RATE_KD) ) {
        Log_Write_PID(CH6_LOITER_RATE_KP, x_rate_error, p, i, d, nav_lon, tuning_value);
    }
#endif

    // North / South
    y_target_speed  = g.pi_loiter_lat.get_p(y_error);                           // calculate desired speed from lat error

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_LOITER_KP || g.radio_tuning == CH6_LOITER_KI) ) {
        Log_Write_PID(CH6_LOITER_KP+100, y_error, y_target_speed, 0, 0, y_target_speed, tuning_value);
    }
#endif

    // calculate rate error
    y_rate_error    = y_target_speed - lat_speed;                          // calc the speed error

    p                               = g.pid_loiter_rate_lat.get_p(y_rate_error);
    i                               = g.pid_loiter_rate_lat.get_i(y_rate_error + y_error, dTnav);
    d                               = g.pid_loiter_rate_lat.get_d(y_error, dTnav);
    d                               = constrain(d, -2000, 2000);

    // get rid of noise
    if(abs(lat_speed) < 50) {
        d = 0;
    }

    output                  = p + i + d;
    nav_lat                 = constrain(output, -4500, 4500); // constrain max angle to 45 degrees

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_LOITER_RATE_KP || g.radio_tuning == CH6_LOITER_RATE_KI || g.radio_tuning == CH6_LOITER_RATE_KD) ) {
        Log_Write_PID(CH6_LOITER_RATE_KP+100, y_rate_error, p, i, d, nav_lat, tuning_value);
    }
#endif

    // copy over I term to Nav_Rate
    g.pid_nav_lon.set_integrator(g.pid_loiter_rate_lon.get_integrator());
    g.pid_nav_lat.set_integrator(g.pid_loiter_rate_lat.get_integrator());
}

///////////////////////////////////////////////////////////
// Waypoint controller (based on GPS position)
///////////////////////////////////////////////////////////
static void calc_nav_rate(int16_t max_speed)
{
    float temp, temp_x, temp_y;

    // push us towards the original track
    update_crosstrack();

    int16_t cross_speed = crosstrack_error * -g.crosstrack_gain;     // scale down crosstrack_error in cm
    cross_speed     = constrain(cross_speed, -150, 150);

    // rotate by 90 to deal with trig functions
    temp                    = (9000l - wp_bearing) * RADX100;
    temp_x                  = cosf(temp);
    temp_y                  = sinf(temp);

    // rotate desired spped vector:
    int32_t x_target_speed = max_speed   * temp_x - cross_speed * temp_y;
    int32_t y_target_speed = cross_speed * temp_x + max_speed   * temp_y;

    // East / West
    // calculate rate error
    x_rate_error    = x_target_speed - lon_speed;

    x_rate_error    = constrain(x_rate_error, -500, 500);
    nav_lon         = g.pid_nav_lon.get_pid(x_rate_error, dTnav);
    int32_t tilt    = (x_target_speed * x_target_speed * (int32_t)g.tilt_comp) / 10000;

    if(x_target_speed < 0) tilt = -tilt;
    nav_lon                 += tilt;


    // North / South
    // calculate rate error
    y_rate_error    = y_target_speed - lat_speed;

    y_rate_error    = constrain(y_rate_error, -500, 500);       // added a rate error limit to keep pitching down to a minimum
    nav_lat         = g.pid_nav_lat.get_pid(y_rate_error, dTnav);
    tilt            = (y_target_speed * y_target_speed * (int32_t)g.tilt_comp) / 10000;

    if(y_target_speed < 0) tilt = -tilt;
    nav_lat                 += tilt;

    // copy over I term to Loiter_Rate
    g.pid_loiter_rate_lon.set_integrator(g.pid_nav_lon.get_integrator());
    g.pid_loiter_rate_lat.set_integrator(g.pid_nav_lat.get_integrator());
}


// this calculation rotates our World frame of reference to the copter's frame of reference
// We use the DCM's matrix to precalculate these trig values at 50hz
static void calc_nav_pitch_roll()
{
    // To-Do: remove this hack dependent upon nav_mode
    if( nav_mode != NAV_LOITER_INAV ) {
        // rotate the vector
        auto_roll       = (float)nav_lon * sin_yaw_y - (float)nav_lat * cos_yaw_x;
        auto_pitch      = (float)nav_lon * cos_yaw_x + (float)nav_lat * sin_yaw_y;

        // flip pitch because forward is negative
        auto_pitch = -auto_pitch;

        // constrain maximum roll and pitch angles to 45 degrees
        auto_roll = constrain(auto_roll, -4500, 4500);
        auto_pitch = constrain(auto_pitch, -4500, 4500);
    }
}

static int16_t get_desired_speed(int16_t max_speed)
{
    /*
    Based on Equation by Bill Premerlani & Robert Lefebvre
    	(sq(V2)-sq(V1))/2 = A(X2-X1)
        derives to:
        V1 = sqrt(sq(V2) - 2*A*(X2-X1))
     */

    if(ap.fast_corner) {
        // don't slow down
    }else{
        if(wp_distance < 20000){ // limit the size of numbers we're dealing with to avoid overflow
            // go slower
    	 	int32_t temp 	= 2 * 100 * (int32_t)(wp_distance - g.waypoint_radius * 100);
    	 	int32_t s_min 	= WAYPOINT_SPEED_MIN;
    	 	temp 			+= s_min * s_min;
            if( temp < 0 ) temp = 0;                // check to ensure we don't try to take the sqrt of a negative number
    		max_speed 		= sqrtf((float)temp);
            max_speed 		= min(max_speed, g.waypoint_speed_max);
        }
    }

    max_speed 		= min(max_speed, max_speed_old + (100 * dTnav));// limit going faster
    max_speed 		= max(max_speed, WAYPOINT_SPEED_MIN); 	// don't go too slow
    max_speed_old 	= max_speed;
    return max_speed;
}

static void reset_desired_speed()
{
    max_speed_old = 0;
}

static void update_crosstrack(void)
{
    // Crosstrack Error
    // ----------------
    if (wp_distance >= (unsigned long)max((g.crosstrack_min_distance * 100),0) &&
        abs(wrap_180(wp_bearing - original_wp_bearing)) < 4500) {

	    float temp = (wp_bearing - original_wp_bearing) * RADX100;
    	crosstrack_error = sinf(temp) * wp_distance;          // Meters we are off track line
    }else{
        // fade out crosstrack
        crosstrack_error >>= 1;
    }
}

static void force_new_altitude(int32_t new_alt)
{
    next_WP.alt     = new_alt;
    set_alt_change(REACHED_ALT);
}

static void set_new_altitude(int32_t new_alt)
{
    // if no change exit immediately
    if(new_alt == next_WP.alt) {
        return;
    }

    // update new target altitude
    next_WP.alt     = new_alt;

    if(next_WP.alt > (current_loc.alt + 80)) {
        // we are below, going up
        set_alt_change(ASCENDING);

    }else if(next_WP.alt < (current_loc.alt - 80)) {
        // we are above, going down
        set_alt_change(DESCENDING);

    }else{
        // No Change
        set_alt_change(REACHED_ALT);
    }
}

static void verify_altitude()
{
    if(alt_change_flag == ASCENDING) {
        // we are below, going up
        if(current_loc.alt >  next_WP.alt - 50) {
        	set_alt_change(REACHED_ALT);
        }
    }else if (alt_change_flag == DESCENDING) {
        // we are above, going down
        if(current_loc.alt <=  next_WP.alt + 50){
        	set_alt_change(REACHED_ALT);
        }
    }
}

// Keeps old data out of our calculation / logs
static void reset_nav_params(void)
{
    // always start Circle mode at same angle
    circle_angle                    = 0;

    // We must be heading to a new WP, so XTrack must be 0
    crosstrack_error                = 0;

    // Will be set by new command
    wp_bearing                      = 0;

    // Will be set by new command
    wp_distance                     = 0;

    // Will be set by new command, used by loiter
    long_error                      = 0;
    lat_error                       = 0;
    nav_lon 						= 0;
    nav_lat 						= 0;
    nav_roll 						= 0;
    nav_pitch 						= 0;
    auto_roll 						= 0;
    auto_pitch 						= 0;
}

static int32_t wrap_360(int32_t error)
{
    if (error > 36000) error -= 36000;
    if (error < 0) error += 36000;
    return error;
}

static int32_t wrap_180(int32_t error)
{
    if (error > 18000) error -= 36000;
    if (error < -18000) error += 36000;
    return error;
}

// get_yaw_slew - reduces rate of change of yaw to a maximum
// assumes it is called at 100hz so centi-degrees and update rate cancel each other out
static int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec)
{
    return wrap_360(current_yaw + constrain(wrap_180(desired_yaw - current_yaw), -deg_per_sec, deg_per_sec));
}

////////////////////////////////////////////////////
// Loiter controller using inertial nav
////////////////////////////////////////////////////

// get_loiter_accel - loiter acceration controllers with desired accelerations provided in forward/right directions in cm/s/s
static void
get_loiter_accel(int16_t accel_req_forward, int16_t accel_req_right)
{
    static float z_accel_meas = 0;      // The acceleration error in cm.
    static float accel_forward = 0;   // The acceleration error in cm.
    static float accel_right = 0;     // The acceleration error in cm.

    z_accel_meas = -AP_INTERTIALNAV_GRAVITY * 100;

    // calculate accel and filter with fc = 2 Hz
    // 100hz sample rate, 2hz filter, alpha = 0.11164f 
    // 20hz sample rate, 2hz filter, alpha = 0.38587f
    // 10hz sample rate, 2hz filter, alpha = 0.55686f
    accel_forward = accel_forward + 0.55686f * (accel_req_forward - accel_forward);
    accel_right = accel_right + 0.55686f * (accel_req_right - accel_right);

    // update angle targets that will be passed to stabilize controller
    auto_roll = constrain((accel_right/(-z_accel_meas))*(18000/M_PI), -4500, 4500);
    auto_pitch = constrain((-accel_forward/(-z_accel_meas*cos_roll_x))*(18000/M_PI), -4500, 4500);
}


// get_loiter_accel_lat_lon - loiter acceration controller with desired accelerations provided in lat/lon directions in cm/s/s
static void
get_loiter_accel_lat_lon(int16_t accel_lat, int16_t accel_lon)
{
    float accel_forward;
    float accel_right;

    accel_forward = accel_lat*cos_yaw + accel_lon*sin_yaw;
    accel_right = -accel_lat*sin_yaw + accel_lon*cos_yaw;

    get_loiter_accel(accel_forward, accel_right);
}


// get_loiter_vel_lat_lon - loiter velocity controller with desired velocity provided in lat/lon directions in cm/s
#define MAX_LOITER_VEL_ACCEL 400        // should be 1.5 times larger than MAX_LOITER_POS_ACCEL
static void
get_loiter_vel_lat_lon(int16_t vel_lat, int16_t vel_lon, float dt)
{
    static float speed_error_lat = 0;     // The velocity in cm/s.
    static float speed_error_lon = 0;     // The velocity in cm/s.

    float speed_lat = inertial_nav.get_latitude_velocity();
    float speed_lon = inertial_nav.get_longitude_velocity();

    int32_t accel_lat;
    int32_t accel_lon;
    int32_t accel_total;

    int16_t lat_p,lat_i,lat_d;
    int16_t lon_p,lon_i,lon_d;

    // calculate vel error and Filter with fc = 2 Hz
    // 100hz sample rate, 2hz filter, alpha = 0.11164f 
    // 20hz sample rate, 2hz filter, alpha = 0.38587f
    // 10hz sample rate, 2hz filter, alpha = 0.55686f
    speed_error_lat = speed_error_lat + 0.55686f * ((vel_lat - speed_lat) - speed_error_lat);
    speed_error_lon = speed_error_lon + 0.55686f * ((vel_lon - speed_lon) - speed_error_lon);

    lat_p   = g.pid_loiter_rate_lat.get_p(speed_error_lat);
    lat_i   = g.pid_loiter_rate_lat.get_i(speed_error_lat, dt);
    lat_d   = g.pid_loiter_rate_lat.get_d(speed_error_lat, dt);

    lon_p   = g.pid_loiter_rate_lon.get_p(speed_error_lon);
    lon_i   = g.pid_loiter_rate_lon.get_i(speed_error_lon, dt);
    lon_d   = g.pid_loiter_rate_lon.get_d(speed_error_lon, dt);

    accel_lat = (lat_p+lat_i+lat_d);
    accel_lon = (lon_p+lon_i+lon_d);

    accel_total = safe_sqrt(accel_lat*accel_lat + accel_lon*accel_lon);

    if( accel_total > MAX_LOITER_VEL_ACCEL ) {
        accel_lat = MAX_LOITER_VEL_ACCEL * accel_lat/accel_total;
        accel_lon = MAX_LOITER_VEL_ACCEL * accel_lon/accel_total;
    }

    get_loiter_accel_lat_lon(accel_lat, accel_lon);
}

// get_loiter_pos_lat_lon - loiter position controller with desired position provided as distance from home in lat/lon directions in cm
#define MAX_LOITER_POS_VELOCITY 750     // should be 1.5 ~ 2.0 times the pilot input's max velocity
#define MAX_LOITER_POS_ACCEL 250
static void
get_loiter_pos_lat_lon(int32_t target_lat, int32_t target_lon, float dt)
{
    static float dist_error_lat;
    int32_t desired_vel_lat;

    static float dist_error_lon;
    int32_t desired_vel_lon;

    int32_t dist_error_total;

    int16_t vel_sqrt;
    int32_t vel_total;

    int16_t linear_distance;      // the distace we swap between linear and sqrt.

    // calculate distance error and Filter with fc = 2 Hz
    // 100hz sample rate, 2hz filter, alpha = 0.11164f 
    // 20hz sample rate, 2hz filter, alpha = 0.38587f
    // 10hz sample rate, 2hz filter, alpha = 0.55686f
    dist_error_lat = dist_error_lat + 0.55686f * ((target_lat - inertial_nav.get_latitude_diff()) - dist_error_lat);
    dist_error_lon = dist_error_lon + 0.55686f * ((target_lon - inertial_nav.get_longitude_diff()) - dist_error_lon);

    linear_distance = MAX_LOITER_POS_ACCEL/(2*g.pi_loiter_lat.kP()*g.pi_loiter_lat.kP());

    dist_error_total = safe_sqrt(dist_error_lat*dist_error_lat + dist_error_lon*dist_error_lon);
    if( dist_error_total > 2*linear_distance ) {
        vel_sqrt = constrain(safe_sqrt(2*MAX_LOITER_POS_ACCEL*(dist_error_total-linear_distance)),0,1000);
        desired_vel_lat = vel_sqrt * dist_error_lat/dist_error_total;
        desired_vel_lon = vel_sqrt * dist_error_lon/dist_error_total;
    }else{
        desired_vel_lat = g.pi_loiter_lat.get_p(dist_error_lat);
        desired_vel_lon = g.pi_loiter_lon.get_p(dist_error_lon);
    }

    vel_total = safe_sqrt(desired_vel_lat*desired_vel_lat + desired_vel_lon*desired_vel_lon);
    if( vel_total > MAX_LOITER_POS_VELOCITY ) {
        desired_vel_lat = MAX_LOITER_POS_VELOCITY * desired_vel_lat/vel_total;
        desired_vel_lon = MAX_LOITER_POS_VELOCITY * desired_vel_lon/vel_total;
    }

    get_loiter_vel_lat_lon(desired_vel_lat, desired_vel_lon, dt);
}


#define MAX_LOITER_POS_VEL_VELOCITY 1000
// loiter_set_pos_from_velocity - loiter velocity controller with desired velocity provided in front/right directions in cm/s
static void
loiter_set_pos_from_velocity(int16_t vel_forward_cms, int16_t vel_right_cms, float dt)
{
    int32_t vel_lat;
    int32_t vel_lon;
    int32_t vel_total;

    vel_lat = vel_forward_cms*cos_yaw - vel_right_cms*sin_yaw;
    vel_lon = vel_forward_cms*sin_yaw + vel_right_cms*cos_yaw;

    // constrain the velocity vector and scale if necessary
    vel_total = safe_sqrt(vel_lat*vel_lat + vel_lon*vel_lon);
    if( vel_total > MAX_LOITER_POS_VEL_VELOCITY ) {
        vel_lat = MAX_LOITER_POS_VEL_VELOCITY * vel_lat/vel_total;
        vel_lon = MAX_LOITER_POS_VEL_VELOCITY * vel_lon/vel_total;
    }

    // update loiter target position
    loiter_lat_from_home_cm += vel_lat * dt;
    loiter_lon_from_home_cm += vel_lon * dt;

    // update next_WP location for reporting purposes
    next_WP.lat = home.lat + loiter_lat_from_home_cm;
    next_WP.lng = home.lng + loiter_lat_from_home_cm * scaleLongUp;
}

// loiter_set_target - set loiter's target position from home in cm
static void
loiter_set_target(float lat_from_home_cm, float lon_from_home_cm)
{
    loiter_lat_from_home_cm = lat_from_home_cm;
    loiter_lon_from_home_cm = lon_from_home_cm;

    // update next_WP location for reporting purposes
    next_WP.lat = home.lat + loiter_lat_from_home_cm;
    next_WP.lng = home.lng + loiter_lat_from_home_cm * scaleLongUp;
}
#line 1 "./Firmware/ACopter32/perf_info.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//  high level performance monitoring
//
//  we measure the main loop time
//

#define PERF_INFO_OVERTIME_THRESHOLD_MICROS 10500

uint16_t perf_info_loop_count;
uint32_t perf_info_max_time;
uint16_t perf_info_long_running;

// perf_info_reset - reset all records of loop time to zero
void perf_info_reset()
{
    perf_info_loop_count = 0;
    perf_info_max_time = 0;
    perf_info_long_running = 0;
}

// perf_info_check_loop_time - check latest loop time vs min, max and overtime threshold
void perf_info_check_loop_time(uint32_t time_in_micros)
{
    perf_info_loop_count++;
    if( time_in_micros > perf_info_max_time) {
        perf_info_max_time = time_in_micros;
    }
    if( time_in_micros > PERF_INFO_OVERTIME_THRESHOLD_MICROS ) {
        perf_info_long_running++;
    }
}

// perf_info_get_long_running_percentage - get number of long running loops as a percentage of the total number of loops
uint16_t perf_info_get_num_loops()
{
    return perf_info_loop_count;
}

// perf_info_get_max_time - return maximum loop time (in microseconds)
uint32_t perf_info_get_max_time()
{
    return perf_info_max_time;
}

// perf_info_get_num_long_running - get number of long running loops
uint16_t perf_info_get_num_long_running()
{
    return perf_info_long_running;
}
#line 1 "./Firmware/ACopter32/radio.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

extern RC_Channel* rc_ch[8];

static void default_dead_zones()
{
    g.rc_1.set_dead_zone(60);
    g.rc_2.set_dead_zone(60);
#if FRAME_CONFIG == HELI_FRAME
    g.rc_3.set_dead_zone(20);
    g.rc_4.set_dead_zone(30);
#else
    g.rc_3.set_dead_zone(60);
    g.rc_4.set_dead_zone(80);
#endif
}

static void init_rc_in()
{
    // set rc channel ranges
    g.rc_1.set_angle(MAX_INPUT_ROLL_ANGLE);
    g.rc_2.set_angle(MAX_INPUT_PITCH_ANGLE);
#if FRAME_CONFIG == HELI_FRAME
    // we do not want to limit the movment of the heli's swash plate
    g.rc_3.set_range(0, 1000);
#else
    g.rc_3.set_range(g.throttle_min, g.throttle_max);
#endif
    g.rc_4.set_angle(4500);

    // reverse: CW = left
    // normal:  CW = left???

    g.rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    rc_ch[CH_1] = &g.rc_1;
    rc_ch[CH_2] = &g.rc_2;
    rc_ch[CH_3] = &g.rc_3;
    rc_ch[CH_4] = &g.rc_4;
    rc_ch[CH_5] = &g.rc_5;
    rc_ch[CH_6] = &g.rc_6;
    rc_ch[CH_7] = &g.rc_7;
    rc_ch[CH_8] = &g.rc_8;

    //set auxiliary ranges
    g.rc_5.set_range(0,1000);
    g.rc_6.set_range(0,1000);
    g.rc_7.set_range(0,1000);
    g.rc_8.set_range(0,1000);

#if MOUNT == ENABLED
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_10, &g.rc_11);
#endif
}

static void init_rc_out()
{
    motors.set_update_rate(g.rc_speed);
    motors.set_frame_orientation(g.frame_orientation);
    motors.Init();                                              // motor initialisation
    motors.set_min_throttle(g.throttle_min);
    motors.set_max_throttle(g.throttle_max);

    for(uint8_t i = 0; i < 5; i++) {
        delay(20);
        read_radio();
    }

    // we want the input to be scaled correctly
    g.rc_3.set_range_out(0,1000);

    // sanity check - prevent unconfigured radios from outputting
    if(g.rc_3.radio_min >= 1300) {
        g.rc_3.radio_min = g.rc_3.radio_in;
    }

    // we are full throttle
    if(g.rc_3.control_in >= (MAXIMUM_THROTTLE - 50)) {
        if(g.esc_calibrate == 0) {
            // we will enter esc_calibrate mode on next reboot
            g.esc_calibrate.set_and_save(1);
            // send minimum throttle out to ESC
            motors.output_min();
            // display message on console
            cliSerial->printf_P(PSTR("Entering ESC Calibration: please restart APM.\n"));
            // block until we restart
            while(1) {
                delay(200);
                dancing_light();
            }
        }else{
            cliSerial->printf_P(PSTR("ESC Calibration active: passing throttle through to ESCs.\n"));
            // clear esc flag
            g.esc_calibrate.set_and_save(0);
            // pass through user throttle to escs
            init_esc();
        }
    }else{
        // did we abort the calibration?
        if(g.esc_calibrate == 1)
            g.esc_calibrate.set_and_save(0);

        // send miinimum throttle out to ESC
        output_min();
    }

#if TOY_EDF == ENABLED
    // add access to CH8 and CH6
    APM_RC.enable_out(CH_8);
    APM_RC.enable_out(CH_6);
#endif
}

void output_min()
{
    // enable motors
    motors.enable();
    motors.output_min();
}

#define RADIO_FS_TIMEOUT_MS 2000       // 2 seconds
static void read_radio()
{
    static uint32_t last_update = 0;
    if (hal.rcin->valid() > 0) {
        last_update = millis();
        ap_system.new_radio_frame = true;
        uint16_t periods[8];
        hal.rcin->read(periods,8);
        g.rc_1.set_pwm(periods[0]);
        g.rc_2.set_pwm(periods[1]);

        set_throttle_and_failsafe(periods[2]);

        g.rc_4.set_pwm(periods[3]);
        g.rc_5.set_pwm(periods[4]);
        g.rc_6.set_pwm(periods[5]);
        g.rc_7.set_pwm(periods[6]);
        g.rc_8.set_pwm(periods[7]);

#if FRAME_CONFIG != HELI_FRAME
        // limit our input to 800 so we can still pitch and roll
        g.rc_3.control_in = min(g.rc_3.control_in, MAXIMUM_THROTTLE);
#endif
    }else{
        uint32_t elapsed = millis() - last_update;
        // turn on throttle failsafe if no update from ppm encoder for 2 seconds
        if ((elapsed >= RADIO_FS_TIMEOUT_MS)
                && g.failsafe_throttle && motors.armed() && !ap.failsafe) {
            Log_Write_Error(ERROR_SUBSYSTEM_RADIO, ERROR_CODE_RADIO_LATE_FRAME);
            set_failsafe(true);
        }
    }
}

#define FS_COUNTER 3
static void set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    static int8_t failsafe_counter = 0;

    // if failsafe not enabled pass through throttle and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        g.rc_3.set_pwm(throttle_pwm);
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (ap.failsafe || !motors.armed()) {
            g.rc_3.set_pwm(throttle_pwm);
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are recieved
        failsafe_counter++;
        if( failsafe_counter >= FS_COUNTER ) {
            failsafe_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe(true);
            g.rc_3.set_pwm(throttle_pwm);   // pass through failsafe throttle
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        failsafe_counter--;
        if( failsafe_counter <= 0 ) {
            failsafe_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (ap.failsafe) {
                set_failsafe(false);
            }
        }
        // pass through throttle
        g.rc_3.set_pwm(throttle_pwm);
    }
}

static void trim_radio()
{
    for (uint8_t i = 0; i < 30; i++) {
        read_radio();
    }

    g.rc_1.trim();      // roll
    g.rc_2.trim();      // pitch
    g.rc_4.trim();      // yaw

    g.rc_1.save_eeprom();
    g.rc_2.save_eeprom();
    g.rc_4.save_eeprom();
}

#line 1 "./Firmware/ACopter32/sensors.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

static void ReadSCP1000(void) {
}

 #if CONFIG_SONAR == ENABLED
static void init_sonar(void)
{
  #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
    sonar->calculate_scaler(g.sonar_type, 3.3f);
  #else
    sonar->calculate_scaler(g.sonar_type, 5.0f);
  #endif
}
 #endif

static void init_barometer(void)
{
    barometer.calibrate();
    ahrs.set_barometer(&barometer);
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// return barometric altitude in centimeters
static int32_t read_barometer(void)
{
    barometer.read();
    return baro_filter.apply(barometer.get_altitude() * 100.0f);
}

// return sonar altitude in centimeters
static int16_t read_sonar(void)
{
#if CONFIG_SONAR == ENABLED
    // exit immediately if sonar is disabled
    if( !g.sonar_enabled ) {
        sonar_alt_health = 0;
        return 0;
    }

    int16_t temp_alt = sonar->read();

    if (temp_alt >= sonar->min_distance && temp_alt <= sonar->max_distance * 0.70f) {
        if ( sonar_alt_health < SONAR_ALT_HEALTH_MAX ) {
            sonar_alt_health++;
        }
    }else{
        sonar_alt_health = 0;
    }

 #if SONAR_TILT_CORRECTION == 1
    // correct alt for angle of the sonar
    float temp = cos_pitch_x * cos_roll_x;
    temp = max(temp, 0.707f);
    temp_alt = (float)temp_alt * temp;
 #endif

    return temp_alt;
#else
    return 0;
#endif
}


#endif // HIL_MODE != HIL_MODE_ATTITUDE

static void init_compass()
{
    compass.set_orientation(MAG_ORIENTATION);                                                   // set compass's orientation on aircraft
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        cliSerial->println_P(PSTR("COMPASS INIT ERROR"));
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.set_compass(&compass);
#endif
}

static void init_optflow()
{
#if OPTFLOW == ENABLED
    if( optflow.init() == false ) {
        g.optflow_enabled = false;
        cliSerial->print_P(PSTR("\nFailed to Init OptFlow "));
        Log_Write_Error(ERROR_SUBSYSTEM_OPTFLOW,ERROR_CODE_FAILED_TO_INITIALISE);
    }else{
        // suspend timer while we set-up SPI communication
        hal.scheduler->suspend_timer_procs();

        optflow.set_orientation(OPTFLOW_ORIENTATION);   // set optical flow sensor's orientation on aircraft
        optflow.set_frame_rate(2000);                   // set minimum update rate (which should lead to maximum low light performance
        optflow.set_resolution(OPTFLOW_RESOLUTION);     // set optical flow sensor's resolution
        optflow.set_field_of_view(OPTFLOW_FOV);         // set optical flow sensor's field of view

        // resume timer
        hal.scheduler->resume_timer_procs();
    }
#endif      // OPTFLOW == ENABLED
}

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
#define BATTERY_FS_COUNTER  100     // 100 iterations at 10hz is 10 seconds
static void read_battery(void)
{
    static uint8_t low_battery_counter = 0;

    if(g.battery_monitoring == 0) {
        battery_voltage1 = 0;
        return;
    }

    if(g.battery_monitoring == 3 || g.battery_monitoring == 4) {
        batt_volt_analog_source->set_pin(g.battery_volt_pin);
        battery_voltage1 = BATTERY_VOLTAGE(batt_volt_analog_source->read_average());
    }
    if(g.battery_monitoring == 4) {
        batt_curr_analog_source->set_pin(g.battery_curr_pin);
        current_amps1    = CURRENT_AMPS(batt_curr_analog_source->read_average());
        current_total1   += current_amps1 * 0.02778f;            // called at 100ms on average, .0002778 is 1/3600 (conversion to hours)
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    if (!ap.low_battery && ( battery_voltage1 < g.low_voltage || (g.battery_monitoring == 4 && current_total1 > g.pack_capacity))) {
        low_battery_counter++;
        if( low_battery_counter >= BATTERY_FS_COUNTER ) {
            low_battery_counter = BATTERY_FS_COUNTER;   // ensure counter does not overflow
            low_battery_event();
        }
    }else{
        // reset low_battery_counter in case it was a temporary voltage dip
        low_battery_counter = 0;
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    rssi_analog_source->set_pin(g.rssi_pin);
    float ret = rssi_analog_source->read_latest();
    receiver_rssi = constrain(ret, 0, 255);
}
#line 1 "./Firmware/ACopter32/setup.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// Functions called from the setup menu
static int8_t   setup_radio             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_motors            (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_accel             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_accel_scale       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_frame             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_factory           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_erase             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_flightmodes       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_batt_monitor      (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_sonar             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_compass           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_tune              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_range             (uint8_t argc, const Menu::arg *argv);
//static int8_t	setup_mag_offset		(uint8_t argc, const Menu::arg *argv);
static int8_t   setup_declination       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_optflow           (uint8_t argc, const Menu::arg *argv);


 #if FRAME_CONFIG == HELI_FRAME
static int8_t   setup_heli              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_gyro              (uint8_t argc, const Menu::arg *argv);
 #endif

// Command/function table for the setup menu
const struct Menu::command setup_menu_commands[] PROGMEM = {
    // command			function called
    // =======          ===============
    {"erase",                       setup_erase},
    {"reset",                       setup_factory},
    {"radio",                       setup_radio},
    {"frame",                       setup_frame},
    {"motors",                      setup_motors},
    {"level",                       setup_accel},
    {"accel",                       setup_accel_scale},
    {"modes",                       setup_flightmodes},
    {"battery",                     setup_batt_monitor},
    {"sonar",                       setup_sonar},
    {"compass",                     setup_compass},
    {"tune",                        setup_tune},
    {"range",                       setup_range},
//	{"offsets",			setup_mag_offset},
    {"declination",         setup_declination},
    {"optflow",                     setup_optflow},
 #if FRAME_CONFIG == HELI_FRAME
    {"heli",                        setup_heli},
    {"gyro",                        setup_gyro},
 #endif
    {"show",                        setup_show}
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
static int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
    // Give the user some guidance
    cliSerial->printf_P(PSTR("Setup Mode\n\n\n"));
    //"\n"
    //"IMPORTANT: if you have not previously set this system up, use the\n"
    //"'reset' command to initialize the EEPROM to sensible default values\n"
    //"and then the 'radio' command to configure for your radio.\n"
    //"\n"));

    if(g.rc_1.radio_min >= 1300) {
        delay(1000);
        cliSerial->printf_P(PSTR("\n!Warning, radio not configured!"));
        delay(1000);
        cliSerial->printf_P(PSTR("\n Type 'radio' now.\n\n"));
    }

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

    report_version();
    report_radio();
    report_frame();
    report_batt_monitor();
    report_sonar();
    //report_gains();
    //report_xtrack();
    //report_throttle();
    report_flight_modes();
    report_ins();
    report_compass();
    report_optflow();

 #if FRAME_CONFIG == HELI_FRAME
    report_heli();
    report_gyro();
 #endif

    AP_Param::show_all();

    return(0);
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{
    int16_t c;

    cliSerial->printf_P(PSTR("\n'Y' = factory reset, any other key to abort:\n"));

    do {
        c = cliSerial->read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);

    AP_Param::erase_all();
    cliSerial->printf_P(PSTR("\nReboot APM"));

    delay(1000);
    //default_gains();

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
    cliSerial->println_P(PSTR("\n\nRadio Setup:"));
    uint8_t i;

    for(i = 0; i < 100; i++) {
        delay(20);
        read_radio();
    }

    if(g.rc_1.radio_in < 500) {
        while(1) {
            //cliSerial->printf_P(PSTR("\nNo radio; Check connectors."));
            delay(1000);
            // stop here
        }
    }

    g.rc_1.radio_min = g.rc_1.radio_in;
    g.rc_2.radio_min = g.rc_2.radio_in;
    g.rc_3.radio_min = g.rc_3.radio_in;
    g.rc_4.radio_min = g.rc_4.radio_in;
    g.rc_5.radio_min = g.rc_5.radio_in;
    g.rc_6.radio_min = g.rc_6.radio_in;
    g.rc_7.radio_min = g.rc_7.radio_in;
    g.rc_8.radio_min = g.rc_8.radio_in;

    g.rc_1.radio_max = g.rc_1.radio_in;
    g.rc_2.radio_max = g.rc_2.radio_in;
    g.rc_3.radio_max = g.rc_3.radio_in;
    g.rc_4.radio_max = g.rc_4.radio_in;
    g.rc_5.radio_max = g.rc_5.radio_in;
    g.rc_6.radio_max = g.rc_6.radio_in;
    g.rc_7.radio_max = g.rc_7.radio_in;
    g.rc_8.radio_max = g.rc_8.radio_in;

    g.rc_1.radio_trim = g.rc_1.radio_in;
    g.rc_2.radio_trim = g.rc_2.radio_in;
    g.rc_4.radio_trim = g.rc_4.radio_in;
    // 3 is not trimed
    g.rc_5.radio_trim = 1500;
    g.rc_6.radio_trim = 1500;
    g.rc_7.radio_trim = 1500;
    g.rc_8.radio_trim = 1500;


    cliSerial->printf_P(PSTR("\nMove all controls to extremes. Enter to save: "));
    while(1) {

        delay(20);
        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        g.rc_1.update_min_max();
        g.rc_2.update_min_max();
        g.rc_3.update_min_max();
        g.rc_4.update_min_max();
        g.rc_5.update_min_max();
        g.rc_6.update_min_max();
        g.rc_7.update_min_max();
        g.rc_8.update_min_max();

        if(cliSerial->available() > 0) {
            delay(20);
            while (cliSerial->read() != -1); /* flush */

            g.rc_1.save_eeprom();
            g.rc_2.save_eeprom();
            g.rc_3.save_eeprom();
            g.rc_4.save_eeprom();
            g.rc_5.save_eeprom();
            g.rc_6.save_eeprom();
            g.rc_7.save_eeprom();
            g.rc_8.save_eeprom();

            print_done();
            break;
        }
    }
    report_radio();
    return(0);
}

static int8_t
setup_motors(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR(
                        "Connect battery for this test.\n"
                        "Motors will not spin in channel order (1,2,3,4) but by frame position order.\n"
                        "Front (& right of centerline) motor first, then in clockwise order around frame.\n"
                        "http://code.google.com/p/arducopter/wiki/AC2_Props_2 for demo video.\n"
                        "Remember to disconnect battery after this test.\n"
                        "Any key to exit.\n"));
    while(1) {
        delay(20);
        read_radio();
        motors.output_test();
        if(cliSerial->available() > 0) {
            g.esc_calibrate.set_and_save(0);
            return(0);
        }
    }
}

static int8_t
setup_accel(uint8_t argc, const Menu::arg *argv)
{
    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate,
             flash_leds);
    ins.init_accel(flash_leds);
    ahrs.set_trim(Vector3f(0,0,0));     // clear out saved trim
    report_ins();
    return(0);
}

/*
  handle full accelerometer calibration via user dialog
 */

static void setup_printf_P(const prog_char_t *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    cliSerial->printf_P(fmt, arg_list);
    va_end(arg_list);
}

static void setup_wait_key(void)
{
    // wait for user input
    while (!cliSerial->available()) {
        delay(20);
    }
    // clear input buffer
    while( cliSerial->available() ) {
        cliSerial->read();
    }
}


static int8_t
setup_accel_scale(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->println_P(PSTR("Initialising gyros"));
    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate,
             flash_leds);
    AP_InertialSensor_UserInteractStream interact(hal.console);
    ins.calibrate_accel(flash_leds, &interact);
    report_ins();
    return(0);
}

static int8_t
setup_frame(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("x"))) {
        g.frame_orientation.set_and_save(X_FRAME);
    } else if (!strcmp_P(argv[1].str, PSTR("p"))) {
        g.frame_orientation.set_and_save(PLUS_FRAME);
    } else if (!strcmp_P(argv[1].str, PSTR("+"))) {
        g.frame_orientation.set_and_save(PLUS_FRAME);
    } else if (!strcmp_P(argv[1].str, PSTR("v"))) {
        g.frame_orientation.set_and_save(V_FRAME);
    }else{
        cliSerial->printf_P(PSTR("\nOp:[x,+,v]\n"));
        report_frame();
        return 0;
    }

    report_frame();
    return 0;
}

static int8_t
setup_flightmodes(uint8_t argc, const Menu::arg *argv)
{
    uint8_t _switchPosition = 0;
    uint8_t _oldSwitchPosition = 0;
    int8_t mode = 0;

    cliSerial->printf_P(PSTR("\nMode switch to edit, aileron: select modes, rudder: Simple on/off\n"));
    print_hit_enter();

    while(1) {
        delay(20);
        read_radio();
        _switchPosition = readSwitch();


        // look for control switch change
        if (_oldSwitchPosition != _switchPosition) {

            mode = flight_modes[_switchPosition];
            mode = constrain(mode, 0, NUM_MODES-1);

            // update the user
            print_switch(_switchPosition, mode, (g.simple_modes & (1<<_switchPosition)));

            // Remember switch position
            _oldSwitchPosition = _switchPosition;
        }

        // look for stick input
        if (abs(g.rc_1.control_in) > 3000) {
            mode++;
            if(mode >= NUM_MODES)
                mode = 0;

            // save new mode
            flight_modes[_switchPosition] = mode;

            // print new mode
            print_switch(_switchPosition, mode, (g.simple_modes & (1<<_switchPosition)));
            delay(500);
        }

        // look for stick input
        if (g.rc_4.control_in > 3000) {
            g.simple_modes |= (1<<_switchPosition);
            // print new mode
            print_switch(_switchPosition, mode, (g.simple_modes & (1<<_switchPosition)));
            delay(500);
        }

        // look for stick input
        if (g.rc_4.control_in < -3000) {
            g.simple_modes &= ~(1<<_switchPosition);
            // print new mode
            print_switch(_switchPosition, mode, (g.simple_modes & (1<<_switchPosition)));
            delay(500);
        }

        // escape hatch
        if(cliSerial->available() > 0) {
            for (mode = 0; mode < 6; mode++)
                flight_modes[mode].save();

            g.simple_modes.save();
            print_done();
            report_flight_modes();
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
setup_tune(uint8_t argc, const Menu::arg *argv)
{
    g.radio_tuning.set_and_save(argv[1].i);
    //g.radio_tuning_high.set_and_save(1000);
    //g.radio_tuning_low.set_and_save(0);
    report_tuning();
    return 0;
}

static int8_t
setup_range(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("\nCH 6 Ranges are divided by 1000: [low, high]\n"));

    g.radio_tuning_low.set_and_save(argv[1].i);
    g.radio_tuning_high.set_and_save(argv[2].i);
    report_tuning();
    return 0;
}



static int8_t
setup_erase(uint8_t argc, const Menu::arg *argv)
{
    zero_eeprom();
    return 0;
}

static int8_t
setup_compass(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.compass_enabled.set_and_save(true);
        init_compass();

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        clear_offsets();
        g.compass_enabled.set_and_save(false);

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on,off]\n"));
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
    if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.battery_monitoring.set_and_save(0);

    } else if(argv[1].i > 0 && argv[1].i <= 4) {
        g.battery_monitoring.set_and_save(argv[1].i);

    } else {
        cliSerial->printf_P(PSTR("\nOp: off, 3-4"));
    }

    report_batt_monitor();
    return 0;
}

static int8_t
setup_sonar(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.sonar_enabled.set_and_save(true);

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.sonar_enabled.set_and_save(false);

    } else if (argc > 1 && (argv[1].i >= 0 && argv[1].i <= 3)) {
        g.sonar_enabled.set_and_save(true);      // if you set the sonar type, surely you want it on
        g.sonar_type.set_and_save(argv[1].i);

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on, off, 0-3]\n"));
        report_sonar();
        return 0;
    }

    report_sonar();
    return 0;
}

 #if FRAME_CONFIG == HELI_FRAME

// Perform heli setup.
// Called by the setup menu 'radio' command.
static int8_t
setup_heli(uint8_t argc, const Menu::arg *argv)
{

    uint8_t active_servo = 0;
    int16_t value = 0;
    int16_t temp;
    int16_t state = 0;       // 0 = set rev+pos, 1 = capture min/max
    int16_t max_roll=0, max_pitch=0, min_collective=0, max_collective=0, min_tail=0, max_tail=0;

    // initialise swash plate
    motors.init_swash();

    // source swash plate movements directly from radio
    motors.servo_manual = true;

    // display initial settings
    report_heli();

    // display help
    cliSerial->printf_P(PSTR("Instructions:"));
    print_divider();
    cliSerial->printf_P(PSTR("\td\t\tdisplay settings\n"));
    cliSerial->printf_P(PSTR("\t1~4\t\tselect servo\n"));
    cliSerial->printf_P(PSTR("\ta or z\t\tmove mid up/down\n"));
    cliSerial->printf_P(PSTR("\tc\t\tset coll when blade pitch zero\n"));
    cliSerial->printf_P(PSTR("\tm\t\tset roll, pitch, coll min/max\n"));
    cliSerial->printf_P(PSTR("\tp<angle>\tset pos (i.e. p0 = front, p90 = right)\n"));
    cliSerial->printf_P(PSTR("\tr\t\treverse servo\n"));
    cliSerial->printf_P(PSTR("\tu a|d\t\tupdate rate (a=analog servo, d=digital)\n"));
    cliSerial->printf_P(PSTR("\tt<angle>\tset trim (-500 ~ 500)\n"));
    cliSerial->printf_P(PSTR("\tx\t\texit & save\n"));

    // start capturing
    while( value != 'x' ) {

        // read radio although we don't use it yet
        read_radio();

        // allow swash plate to move
        motors.output_armed();

        // record min/max
        if( state == 1 ) {
            if( abs(g.rc_1.control_in) > max_roll )
                max_roll = abs(g.rc_1.control_in);
            if( abs(g.rc_2.control_in) > max_pitch )
                max_pitch = abs(g.rc_2.control_in);
            if( g.rc_3.radio_out < min_collective )
                min_collective = g.rc_3.radio_out;
            if( g.rc_3.radio_out > max_collective )
                max_collective = g.rc_3.radio_out;
            min_tail = min(g.rc_4.radio_out, min_tail);
            max_tail = max(g.rc_4.radio_out, max_tail);
        }

        if( cliSerial->available() ) {
            value = cliSerial->read();

            // process the user's input
            switch( value ) {
            case '1':
                active_servo = CH_1;
                break;
            case '2':
                active_servo = CH_2;
                break;
            case '3':
                active_servo = CH_3;
                break;
            case '4':
                active_servo = CH_4;
                break;
            case 'a':
            case 'A':
                heli_get_servo(active_servo)->radio_trim += 10;
                break;
            case 'c':
            case 'C':
                if( g.rc_3.radio_out >= 900 && g.rc_3.radio_out <= 2100 ) {
                    motors.collective_mid = g.rc_3.radio_out;
                    cliSerial->printf_P(PSTR("Collective when blade pitch at zero: %d\n"),(int)motors.collective_mid);
                }
                break;
            case 'd':
            case 'D':
                // display settings
                report_heli();
                break;
            case 'm':
            case 'M':
                if( state == 0 ) {
                    state = 1;                          // switch to capture min/max mode
                    cliSerial->printf_P(PSTR("Move coll, roll, pitch and tail to extremes, press 'm' when done\n"));

                    // reset servo ranges
                    motors.roll_max = motors.pitch_max = 4500;
                    motors.collective_min = 1000;
                    motors.collective_max = 2000;
                    motors._servo_4->radio_min = 1000;
                    motors._servo_4->radio_max = 2000;

                    // set sensible values in temp variables
                    max_roll = abs(g.rc_1.control_in);
                    max_pitch = abs(g.rc_2.control_in);
                    min_collective = 2000;
                    max_collective = 1000;
                    min_tail = max_tail = abs(g.rc_4.radio_out);
                }else{
                    state = 0;                          // switch back to normal mode
                    // double check values aren't totally terrible
                    if( max_roll <= 1000 || max_pitch <= 1000 || (max_collective - min_collective < 200) || (max_tail - min_tail < 200) || min_tail < 1000 || max_tail > 2000 )
                        cliSerial->printf_P(PSTR("Invalid min/max captured roll:%d,  pitch:%d,  collective min: %d max: %d,  tail min:%d max:%d\n"),max_roll,max_pitch,min_collective,max_collective,min_tail,max_tail);
                    else{
                        motors.roll_max = max_roll;
                        motors.pitch_max = max_pitch;
                        motors.collective_min = min_collective;
                        motors.collective_max = max_collective;
                        motors._servo_4->radio_min = min_tail;
                        motors._servo_4->radio_max = max_tail;

                        // reinitialise swash
                        motors.init_swash();

                        // display settings
                        report_heli();
                    }
                }
                break;
            case 'p':
            case 'P':
                temp = read_num_from_serial();
                if( temp >= -360 && temp <= 360 ) {
                    if( active_servo == CH_1 )
                        motors.servo1_pos = temp;
                    if( active_servo == CH_2 )
                        motors.servo2_pos = temp;
                    if( active_servo == CH_3 )
                        motors.servo3_pos = temp;
                    motors.init_swash();
                    cliSerial->printf_P(PSTR("Servo %d\t\tpos:%d\n"),active_servo+1, temp);
                }
                break;
            case 'r':
            case 'R':
                heli_get_servo(active_servo)->set_reverse(!heli_get_servo(active_servo)->get_reverse());
                break;
            case 't':
            case 'T':
                temp = read_num_from_serial();
                if( temp > 1000 )
                    temp -= 1500;
                if( temp > -500 && temp < 500 ) {
                    heli_get_servo(active_servo)->radio_trim = 1500 + temp;
                    motors.init_swash();
                    cliSerial->printf_P(PSTR("Servo %d\t\ttrim:%d\n"),active_servo+1, 1500 + temp);
                }
                break;
            case 'u':
            case 'U':
                temp = 0;
                // delay up to 2 seconds for servo type from user
                while( !cliSerial->available() && temp < 20 ) {
                    temp++;
                    delay(100);
                }
                if( cliSerial->available() ) {
                    value = cliSerial->read();
                    if( value == 'a' || value == 'A' ) {
                        g.rc_speed.set_and_save(AP_MOTORS_HELI_SPEED_ANALOG_SERVOS);
                        //motors._speed_hz = AP_MOTORS_HELI_SPEED_ANALOG_SERVOS;  // need to force this update to take effect immediately
                        cliSerial->printf_P(PSTR("Analog Servo %dhz\n"),(int)g.rc_speed);
                    }
                    if( value == 'd' || value == 'D' ) {
                        g.rc_speed.set_and_save(AP_MOTORS_HELI_SPEED_ANALOG_SERVOS);
                        //motors._speed_hz = AP_MOTORS_HELI_SPEED_ANALOG_SERVOS;  // need to force this update to take effect immediately
                        cliSerial->printf_P(PSTR("Digital Servo %dhz\n"),(int)g.rc_speed);
                    }
                }
                break;
            case 'z':
            case 'Z':
                heli_get_servo(active_servo)->radio_trim -= 10;
                break;
            }
        }

        delay(20);
    }

    // display final settings
    report_heli();

    // save to eeprom
    motors._servo_1->save_eeprom();
    motors._servo_2->save_eeprom();
    motors._servo_3->save_eeprom();
    motors._servo_4->save_eeprom();
    motors.servo1_pos.save();
    motors.servo2_pos.save();
    motors.servo3_pos.save();
    motors.roll_max.save();
    motors.pitch_max.save();
    motors.collective_min.save();
    motors.collective_max.save();
    motors.collective_mid.save();

    // return swash plate movements to attitude controller
    motors.servo_manual = false;

    return(0);
}

// setup for external tail gyro (for heli only)
static int8_t
setup_gyro(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        motors.ext_gyro_enabled.set_and_save(true);

        // optionally capture the gain
        if( argc >= 2 && argv[2].i >= 1000 && argv[2].i <= 2000 ) {
            motors.ext_gyro_gain = argv[2].i;
            motors.ext_gyro_gain.save();
        }

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        motors.ext_gyro_enabled.set_and_save(false);

        // capture gain if user simply provides a number
    } else if( argv[1].i >= 1000 && argv[1].i <= 2000 ) {
        motors.ext_gyro_enabled.set_and_save(true);
        motors.ext_gyro_gain = argv[1].i;
        motors.ext_gyro_gain.save();

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on, off] gain\n"));
    }

    report_gyro();
    return 0;
}

 #endif // FRAME_CONFIG == HELI

static void clear_offsets()
{
    Vector3f _offsets(0.0,0.0,0.0);
    compass.set_offsets(_offsets);
    compass.save_offsets();
}

static int8_t
setup_optflow(uint8_t argc, const Menu::arg *argv)
{
 #if OPTFLOW == ENABLED
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.optflow_enabled = true;
        init_optflow();

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.optflow_enabled = false;

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on, off]\n"));
        report_optflow();
        return 0;
    }

    g.optflow_enabled.save();
    report_optflow();
 #endif     // OPTFLOW == ENABLED
    return 0;
}



/***************************************************************************/
// CLI reports
/***************************************************************************/

static void report_batt_monitor()
{
    cliSerial->printf_P(PSTR("\nBatt Mon:\n"));
    print_divider();
    if(g.battery_monitoring == 0) print_enabled(false);
    if(g.battery_monitoring == 3) cliSerial->printf_P(PSTR("volts"));
    if(g.battery_monitoring == 4) cliSerial->printf_P(PSTR("volts and cur"));
    print_blanks(2);
}

static void report_wp(uint8_t index = 255)
{
    if(index == 255) {
        for(uint8_t i = 0; i < g.command_total; i++) {
            struct Location temp = get_cmd_with_index(i);
            print_wp(&temp, i);
        }
    }else{
        struct Location temp = get_cmd_with_index(index);
        print_wp(&temp, index);
    }
}

static void report_sonar()
{
    cliSerial->printf_P(PSTR("Sonar\n"));
    print_divider();
    print_enabled(g.sonar_enabled.get());
    cliSerial->printf_P(PSTR("Type: %d (0=XL, 1=LV, 2=XLL, 3=HRLV)"), (int)g.sonar_type);
    print_blanks(2);
}

static void report_frame()
{
    cliSerial->printf_P(PSTR("Frame\n"));
    print_divider();

 #if FRAME_CONFIG == QUAD_FRAME
    cliSerial->printf_P(PSTR("Quad frame\n"));
 #elif FRAME_CONFIG == TRI_FRAME
    cliSerial->printf_P(PSTR("TRI frame\n"));
 #elif FRAME_CONFIG == HEXA_FRAME
    cliSerial->printf_P(PSTR("Hexa frame\n"));
 #elif FRAME_CONFIG == Y6_FRAME
    cliSerial->printf_P(PSTR("Y6 frame\n"));
 #elif FRAME_CONFIG == OCTA_FRAME
    cliSerial->printf_P(PSTR("Octa frame\n"));
 #elif FRAME_CONFIG == HELI_FRAME
    cliSerial->printf_P(PSTR("Heli frame\n"));
 #endif

 #if FRAME_CONFIG != HELI_FRAME
    if(g.frame_orientation == X_FRAME)
        cliSerial->printf_P(PSTR("X mode\n"));
    else if(g.frame_orientation == PLUS_FRAME)
        cliSerial->printf_P(PSTR("+ mode\n"));
    else if(g.frame_orientation == V_FRAME)
        cliSerial->printf_P(PSTR("V mode\n"));
 #endif

    print_blanks(2);
}

static void report_radio()
{
    cliSerial->printf_P(PSTR("Radio\n"));
    print_divider();
    // radio
    print_radio_values();
    print_blanks(2);
}

static void report_ins()
{
    cliSerial->printf_P(PSTR("INS\n"));
    print_divider();

    print_gyro_offsets();
    print_accel_offsets_and_scaling();
    print_blanks(2);
}

static void report_compass()
{
    cliSerial->printf_P(PSTR("Compass\n"));
    print_divider();

    print_enabled(g.compass_enabled);

    // mag declination
    cliSerial->printf_P(PSTR("Mag Dec: %4.4f\n"),
                    degrees(compass.get_declination()));

    Vector3f offsets = compass.get_offsets();

    // mag offsets
    cliSerial->printf_P(PSTR("Mag off: %4.4f, %4.4f, %4.4f"),
                    offsets.x,
                    offsets.y,
                    offsets.z);
    print_blanks(2);
}

static void report_flight_modes()
{
    cliSerial->printf_P(PSTR("Flight modes\n"));
    print_divider();

    for(int16_t i = 0; i < 6; i++ ) {
        print_switch(i, flight_modes[i], (g.simple_modes & (1<<i)));
    }
    print_blanks(2);
}

void report_optflow()
{
 #if OPTFLOW == ENABLED
    cliSerial->printf_P(PSTR("OptFlow\n"));
    print_divider();

    print_enabled(g.optflow_enabled);

    // field of view
    //cliSerial->printf_P(PSTR("FOV: %4.0f\n"),
    //						degrees(g.optflow_fov));

    print_blanks(2);
 #endif     // OPTFLOW == ENABLED
}

 #if FRAME_CONFIG == HELI_FRAME
static void report_heli()
{
    cliSerial->printf_P(PSTR("Heli\n"));
    print_divider();

    // main servo settings
    cliSerial->printf_P(PSTR("Servo \tpos \tmin \tmax \trev\n"));
    cliSerial->printf_P(PSTR("1:\t%d \t%d \t%d \t%d\n"),(int)motors.servo1_pos, (int)motors._servo_1->radio_min, (int)motors._servo_1->radio_max, (int)motors._servo_1->get_reverse());
    cliSerial->printf_P(PSTR("2:\t%d \t%d \t%d \t%d\n"),(int)motors.servo2_pos, (int)motors._servo_2->radio_min, (int)motors._servo_2->radio_max, (int)motors._servo_2->get_reverse());
    cliSerial->printf_P(PSTR("3:\t%d \t%d \t%d \t%d\n"),(int)motors.servo3_pos, (int)motors._servo_3->radio_min, (int)motors._servo_3->radio_max, (int)motors._servo_3->get_reverse());
    cliSerial->printf_P(PSTR("tail:\t\t%d \t%d \t%d\n"), (int)motors._servo_4->radio_min, (int)motors._servo_4->radio_max, (int)motors._servo_4->get_reverse());

    cliSerial->printf_P(PSTR("roll max: \t%d\n"), (int)motors.roll_max);
    cliSerial->printf_P(PSTR("pitch max: \t%d\n"), (int)motors.pitch_max);
    cliSerial->printf_P(PSTR("coll min:\t%d\t mid:%d\t max:%d\n"),(int)motors.collective_min, (int)motors.collective_mid, (int)motors.collective_max);

    // calculate and print servo rate
    cliSerial->printf_P(PSTR("servo rate:\t%d hz\n"),(int)g.rc_speed);

    print_blanks(2);
}

static void report_gyro()
{

    cliSerial->printf_P(PSTR("Gyro:\n"));
    print_divider();

    print_enabled( motors.ext_gyro_enabled );
    if( motors.ext_gyro_enabled )
        cliSerial->printf_P(PSTR("gain: %d"),(int)motors.ext_gyro_gain);

    print_blanks(2);
}

 #endif // FRAME_CONFIG == HELI_FRAME

/***************************************************************************/
// CLI utilities
/***************************************************************************/

/*static void
 *  print_PID(PI * pid)
 *  {
 *       cliSerial->printf_P(PSTR("P: %4.2f, I:%4.2f, IMAX:%ld\n"),
 *                                               pid->kP(),
 *                                               pid->kI(),
 *                                               (long)pid->imax());
 *  }
 */

static void
print_radio_values()
{
    cliSerial->printf_P(PSTR("CH1: %d | %d\n"), (int)g.rc_1.radio_min, (int)g.rc_1.radio_max);
    cliSerial->printf_P(PSTR("CH2: %d | %d\n"), (int)g.rc_2.radio_min, (int)g.rc_2.radio_max);
    cliSerial->printf_P(PSTR("CH3: %d | %d\n"), (int)g.rc_3.radio_min, (int)g.rc_3.radio_max);
    cliSerial->printf_P(PSTR("CH4: %d | %d\n"), (int)g.rc_4.radio_min, (int)g.rc_4.radio_max);
    cliSerial->printf_P(PSTR("CH5: %d | %d\n"), (int)g.rc_5.radio_min, (int)g.rc_5.radio_max);
    cliSerial->printf_P(PSTR("CH6: %d | %d\n"), (int)g.rc_6.radio_min, (int)g.rc_6.radio_max);
    cliSerial->printf_P(PSTR("CH7: %d | %d\n"), (int)g.rc_7.radio_min, (int)g.rc_7.radio_max);
    //cliSerial->printf_P(PSTR("CH8: %d | %d\n"), (int)g.rc_8.radio_min, (int)g.rc_8.radio_max);
}

static void
print_switch(uint8_t p, uint8_t m, bool b)
{
    cliSerial->printf_P(PSTR("Pos %d:\t"),p);
    print_flight_mode(m);
    cliSerial->printf_P(PSTR(",\t\tSimple: "));
    if(b)
        cliSerial->printf_P(PSTR("ON\n"));
    else
        cliSerial->printf_P(PSTR("OFF\n"));
}

static void
print_done()
{
    cliSerial->printf_P(PSTR("\nSaved\n"));
}


static void zero_eeprom(void)
{
    cliSerial->printf_P(PSTR("\nErasing EEPROM\n"));

    for (uint16_t i = 0; i < EEPROM_MAX_ADDR; i++) {
        hal.storage->write_byte(i, 0);
    }

    cliSerial->printf_P(PSTR("done\n"));
}

static void
print_accel_offsets_and_scaling(void)
{
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f accel_scale = ins.get_accel_scale();
    cliSerial->printf_P(PSTR("A_off: %4.2f, %4.2f, %4.2f\tA_scale: %4.2f, %4.2f, %4.2f\n"),
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
    cliSerial->printf_P(PSTR("G_off: %4.2f, %4.2f, %4.2f\n"),
                    (float)gyro_offsets.x,
                    (float)gyro_offsets.y,
                    (float)gyro_offsets.z);
}

 #if FRAME_CONFIG == HELI_FRAME

static RC_Channel *
heli_get_servo(int16_t servo_num){
    if( servo_num == CH_1 )
        return motors._servo_1;
    if( servo_num == CH_2 )
        return motors._servo_2;
    if( servo_num == CH_3 )
        return motors._servo_3;
    if( servo_num == CH_4 )
        return motors._servo_4;
    return NULL;
}

// Used to read integer values from the serial port
static int16_t read_num_from_serial() {
    uint8_t index = 0;
    uint8_t timeout = 0;
    char data[5] = "";

    do {
        if (cliSerial->available() == 0) {
            delay(10);
            timeout++;
        }else{
            data[index] = cliSerial->read();
            timeout = 0;
            index++;
        }
    } while (timeout < 5 && index < 5);

    return atoi(data);
}
 #endif

#endif // CLI_ENABLED

static void
print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        cliSerial->println("");
    }
}

static void
print_divider(void)
{
    for (int i = 0; i < 40; i++) {
        cliSerial->print_P(PSTR("-"));
    }
    cliSerial->println();
}

static void print_enabled(bool b)
{
    if(b)
        cliSerial->print_P(PSTR("en"));
    else
        cliSerial->print_P(PSTR("dis"));
    cliSerial->print_P(PSTR("abled\n"));
}


static void
init_esc()
{
    // reduce update rate to motors to 50Hz
    motors.set_update_rate(50);
    motors.enable();
    motors.armed(true);
    while(1) {
        read_radio();
        delay(100);
        dancing_light();
        motors.throttle_pass_through();
    }
}

static void print_wp(struct Location *cmd, uint8_t index)
{
   	//float t1 = (float)cmd->lat / t7;
    //float t2 = (float)cmd->lng / t7;

    cliSerial->printf_P(PSTR("cmd#: %d | %d, %d, %d, %ld, %ld, %ld\n"),
                    index,
                    cmd->id,
                    cmd->options,
                    cmd->p1,
                    cmd->alt,
                    cmd->lat,
                    cmd->lng);

	/*
    cliSerial->printf_P(PSTR("cmd#: %d id:%d op:%d p1:%d p2:%ld p3:%4.7f p4:%4.7f \n"),
                    (int)index,
                    (int)cmd->id,
                    (int)cmd->options,
                    (int)cmd->p1,
                    (long)cmd->alt,
                    t1,
                    t2);
	*/
}

static void report_version()
{
    cliSerial->printf_P(PSTR("FW Ver: %d\n"),(int)g.k_format_version);
    print_divider();
    print_blanks(2);
}


static void report_tuning()
{
    cliSerial->printf_P(PSTR("\nTUNE:\n"));
    print_divider();
    if (g.radio_tuning == 0) {
        print_enabled(g.radio_tuning.get());
    }else{
        float low  = (float)g.radio_tuning_low.get() / 1000;
        float high = (float)g.radio_tuning_high.get() / 1000;
        cliSerial->printf_P(PSTR(" %d, Low:%1.4f, High:%1.4f\n"),(int)g.radio_tuning.get(), low, high);
    }
    print_blanks(2);
}
#line 1 "./Firmware/ACopter32/system.pde"
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
                         "  logs\n"
                         "  setup\n"
                         "  test\n"
                         "  reboot\n"
                         "\n"));
    return(0);
}

// Command/function table for the top-level menu.
const struct Menu::command main_menu_commands[] PROGMEM = {
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
    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

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

    ap_system.usb_connected = !digitalReadFast(USB_MUX_PIN);
    if (!ap_system.usb_connected) {
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
#if HIL_MODE != HIL_MODE_DISABLED
    // we need more memory for HIL, as we get a much higher packet rate
    hal.uartA->begin(SERIAL0_BAUD, 256, 256);
#else
    // use a bit less for non-HIL operation
    hal.uartA->begin(SERIAL0_BAUD, 128, 128);
#endif

    // GPS serial port.
    //
#if GPS_PROTOCOL != GPS_PROTOCOL_IMU
    // standard gps running. Note that we need a 256 byte buffer for some
    // GPS types (eg. UBLOX)
    hal.uartB->begin(38400, 256, 16);
#endif

    cliSerial->printf_P(PSTR("\n\nInit " THISFIRMWARE
                         "\n\nFree RAM: %u\n"),
                    memcheck_available_memory());

    //
    // Report firmware version code expect on console (check of actual EEPROM format version is done in load_parameters function)
    //
    report_version();

    // setup IO pins
    pinMode(A_LED_PIN, OUTPUT);                                 // GPS status LED
    digitalWrite(A_LED_PIN, LED_OFF);

    pinMode(B_LED_PIN, OUTPUT);                         // GPS status LED
    digitalWrite(B_LED_PIN, LED_OFF);

    pinMode(C_LED_PIN, OUTPUT);                         // GPS status LED
    digitalWrite(C_LED_PIN, LED_OFF);

#if SLIDE_SWITCH_PIN > 0
    pinMode(SLIDE_SWITCH_PIN, INPUT);           // To enter interactive mode
#endif
#if CONFIG_PUSHBUTTON == ENABLED
    pinMode(PUSHBUTTON_PIN, INPUT);                     // unused
#endif

    relay.init(); 

#if COPTER_LEDS == ENABLED
    pinMode(COPTER_LED_1, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_2, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_3, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_4, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_5, OUTPUT);              //Motor or Aux LED
    pinMode(COPTER_LED_6, OUTPUT);              //Motor or Aux LED
    pinMode(COPTER_LED_7, OUTPUT);              //Motor or GPS LED
    pinMode(COPTER_LED_8, OUTPUT);              //Motor or GPS LED

    if ( !bitRead(g.copter_leds_mode, 3) ) {
        piezo_beep();
    }

#endif


    // load parameters from EEPROM
    load_parameters();

    // init the GCS
    gcs0.init(hal.uartA);

    // Register the mavlink service callback. This will run
    // anytime there are more than 5ms remaining in a call to
    // hal.scheduler->delay.
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

#if USB_MUX_PIN > 0
    if (!ap_system.usb_connected) {
        // we are not connected via USB, re-init UART0 with right
        // baud rate
        hal.uartA->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD));
    }
#else
    // we have a 2nd serial port for telemetry
    hal.uartC->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
    gcs3.init(hal.uartC);
#endif

    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;
    mavlink_system.type = 2; //MAV_QUADROTOR;

#if LOGGING_ENABLED == ENABLED
    DataFlash.Init();
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
        do_erase_logs();
    }
    if (g.log_bitmask != 0) {
        DataFlash.start_new_log();
    }
#endif

#if FRAME_CONFIG == HELI_FRAME
    motors.servo_manual = false;
    motors.init_swash();              // heli initialisation
#endif

    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up the timer libs
    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

#if HIL_MODE != HIL_MODE_ATTITUDE
 #if CONFIG_ADC == ENABLED
    // begin filtering the ADC Gyros
    adc.Init();           // APM ADC library initialization
 #endif // CONFIG_ADC

    barometer.init();

#endif // HIL_MODE

    // Do GPS init
    g_gps = &g_gps_driver;
    // GPS Initialization
    g_gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_1G);

    if(g.compass_enabled)
        init_compass();

    // init the optical flow sensor
    if(g.optflow_enabled) {
        init_optflow();
    }

#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
    // initialise inertial nav
    inertial_nav.init();
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

#if CLI_ENABLED == ENABLED && CLI_SLIDER_ENABLED == ENABLED
    // If the switch is in 'menu' mode, run the main menu.
    //
    // Since we can't be sure that the setup or test mode won't leave
    // the system in an odd state, we don't let the user exit the top
    // menu; they must reset in order to fly.
    //
    if (check_startup_for_CLI()) {
        digitalWrite(A_LED_PIN, LED_ON);                        // turn on setup-mode LED
        cliSerial->printf_P(PSTR("\nCLI:\n\n"));
        run_cli(cliSerial);
    }
#else
    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
#if USB_MUX_PIN == 0
    hal.uartC->println_P(msg);
#endif
#endif // CLI_ENABLED

#if HIL_MODE != HIL_MODE_ATTITUDE
    // read Baro pressure at ground
    //-----------------------------
    init_barometer();
#endif

    // initialise sonar
#if CONFIG_SONAR == ENABLED
    init_sonar();
#endif

#if FRAME_CONFIG == HELI_FRAME
// initialise controller filters
init_rate_controllers();
#endif // HELI_FRAME

    // initialize commands
    // -------------------
    init_commands();

    // set the correct flight mode
    // ---------------------------
    reset_control_switch();


    startup_ground();

#if LOGGING_ENABLED == ENABLED
    Log_Write_Startup();
#endif

    init_ap_limits();

    cliSerial->print_P(PSTR("\nReady to FLY "));
}



///////////////////////////////////////////////////////////////////////////////
// Experimental AP_Limits library - set constraints, limits, fences, minima,
// maxima on various parameters
////////////////////////////////////////////////////////////////////////////////
static void init_ap_limits() {
#if AP_LIMITS == ENABLED
    // The linked list looks (logically) like this [limits module] -> [first
    // limit module] -> [second limit module] -> [third limit module] -> NULL


    // The details of the linked list are handled by the methods
    // modules_first, modules_current, modules_next, modules_last, modules_add
    // in limits

    limits.modules_add(&gpslock_limit);
    limits.modules_add(&geofence_limit);
    limits.modules_add(&altitude_limit);


    if (limits.debug())  {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Limits Modules Loaded"));

        AP_Limit_Module *m = limits.modules_first();
        while (m) {
            gcs_send_text_P(SEVERITY_LOW, get_module_name(m->get_module_id()));
            m = limits.modules_next();
        }
    }
#endif
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
static void startup_ground(void)
{
    gcs_send_text_P(SEVERITY_LOW,PSTR("GROUND START"));

    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();

    // Warm up and read Gyro offsets
    // -----------------------------
    ins.init(AP_InertialSensor::COLD_START,
             ins_sample_rate,
             flash_leds);
 #if CLI_ENABLED == ENABLED
    report_ins();
 #endif

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);

#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.init(&timer_scheduler);
    ahrs2.set_as_secondary(true);
    ahrs2.set_fast_gains(true);
#endif

    // reset the leds
    // ---------------------------
    clear_leds();

    // when we re-calibrate the gyros,
    // all previous I values are invalid
    reset_I_all();
}

// set_mode - change flight mode and perform any necessary initialisation
static void set_mode(uint8_t mode)
{
    // Switch to stabilize mode if requested mode requires a GPS lock
    if(!ap.home_is_set) {
        if (mode > ALT_HOLD && mode != TOY_A && mode != TOY_M && mode != OF_LOITER && mode != LAND) {
            mode = STABILIZE;
        }
    }

    // Switch to stabilize if OF_LOITER requested but no optical flow sensor
    if (mode == OF_LOITER && !g.optflow_enabled ) {
        mode = STABILIZE;
    }

    control_mode 	= mode;
    control_mode    = constrain(control_mode, 0, NUM_MODES - 1);

    // used to stop fly_aways
    // set to false if we have low throttle
    motors.auto_armed(g.rc_3.control_in > 0 || ap.failsafe);
    set_auto_armed(g.rc_3.control_in > 0 || ap.failsafe);

    // if we change modes, we must clear landed flag
    set_land_complete(false);

    // debug to Serial terminal
    //cliSerial->println(flight_mode_strings[control_mode]);

    ap.loiter_override  = false;

    // report the GPS and Motor arming status
    led_mode = NORMAL_LEDS;

    switch(control_mode)
    {
    case ACRO:
    	ap.manual_throttle = true;
    	ap.manual_attitude = true;
        set_yaw_mode(ACRO_YAW);
        set_roll_pitch_mode(ACRO_RP);
        set_throttle_mode(ACRO_THR);
        set_nav_mode(ACRO_NAV);
        // reset acro axis targets to current attitude
		if(g.axis_enabled){
            roll_axis 	= ahrs.roll_sensor;
            pitch_axis 	= ahrs.pitch_sensor;
            nav_yaw 	= ahrs.yaw_sensor;
        }
        break;

    case STABILIZE:
    	ap.manual_throttle = true;
    	ap.manual_attitude = true;
        set_yaw_mode(YAW_HOLD);
        set_roll_pitch_mode(ROLL_PITCH_STABLE);
        set_throttle_mode(THROTTLE_MANUAL_TILT_COMPENSATED);
        set_nav_mode(NAV_NONE);
        break;

    case ALT_HOLD:
    	ap.manual_throttle = false;
    	ap.manual_attitude = true;
        set_yaw_mode(ALT_HOLD_YAW);
        set_roll_pitch_mode(ALT_HOLD_RP);
        set_throttle_mode(ALT_HOLD_THR);
        set_nav_mode(ALT_HOLD_NAV);
        break;

    case AUTO:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(AUTO_YAW);
        set_roll_pitch_mode(AUTO_RP);
        set_throttle_mode(AUTO_THR);
        // we do not set nav mode for auto because it will be overwritten when first command runs
        // loads the commands from where we left off
        init_commands();
        break;

    case CIRCLE:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        // set yaw to point to center of circle
        yaw_look_at_WP = circle_WP;
        set_yaw_mode(CIRCLE_YAW);
        set_roll_pitch_mode(CIRCLE_RP);
        set_throttle_mode(CIRCLE_THR);
        set_nav_mode(CIRCLE_NAV);
        break;

    case LOITER:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(LOITER_YAW);
        set_roll_pitch_mode(LOITER_RP);
        set_throttle_mode(LOITER_THR);
        set_next_WP(&current_loc);
        set_nav_mode(LOITER_NAV);
        break;

    case POSITION:
    	ap.manual_throttle = true;
    	ap.manual_attitude = false;
        set_yaw_mode(POSITION_YAW);
        set_roll_pitch_mode(POSITION_RP);
        set_throttle_mode(POSITION_THR);
        set_next_WP(&current_loc);
        set_nav_mode(POSITION_NAV);
        break;

    case GUIDED:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(GUIDED_YAW);
        set_roll_pitch_mode(GUIDED_RP);
        set_throttle_mode(GUIDED_THR);
        set_nav_mode(GUIDED_NAV);
        wp_verify_byte = 0;
        set_next_WP(&guided_WP);
        break;

    case LAND:
        if( ap.home_is_set ) {
            // switch to loiter if we have gps
            ap.manual_attitude = false;
            set_yaw_mode(LOITER_YAW);
            set_roll_pitch_mode(LOITER_RP);
        }else{
            // otherwise remain with stabilize roll and pitch
            ap.manual_attitude = true;
            set_yaw_mode(YAW_HOLD);
            set_roll_pitch_mode(ROLL_PITCH_STABLE);
        }
    	ap.manual_throttle = false;
        do_land();
        break;

    case RTL:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        do_RTL();
        break;

    case OF_LOITER:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(OF_LOITER_YAW);
        set_roll_pitch_mode(OF_LOITER_RP);
        set_throttle_mode(OF_LOITER_THR);
        set_nav_mode(OF_LOITER_NAV);
        set_next_WP(&current_loc);
        break;

    // THOR
    // These are the flight modes for Toy mode
    // See the defines for the enumerated values
    case TOY_A:
    	ap.manual_throttle = false;
    	ap.manual_attitude = true;
        set_yaw_mode(YAW_TOY);
        set_roll_pitch_mode(ROLL_PITCH_TOY);
        set_throttle_mode(THROTTLE_AUTO);
        set_nav_mode(NAV_NONE);

        // save throttle for fast exit of Alt hold
        saved_toy_throttle = g.rc_3.control_in;
        break;

    case TOY_M:
    	ap.manual_throttle = false;
    	ap.manual_attitude = true;
        set_yaw_mode(YAW_TOY);
        set_roll_pitch_mode(ROLL_PITCH_TOY);
        set_nav_mode(NAV_NONE);
        set_throttle_mode(THROTTLE_HOLD);
        break;

    default:
        break;
    }

    if(ap.manual_attitude) {
        // We are under manual attitude control
        // remove the navigation from roll and pitch command
        reset_nav_params();
        // remove the wind compenstaion
        reset_wind_I();
    }

    Log_Write_Mode(control_mode);
}

static void
init_simple_bearing()
{
    initial_simple_bearing = ahrs.yaw_sensor;
    if (g.log_bitmask != 0) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, initial_simple_bearing);
    }
}

#if CLI_SLIDER_ENABLED == ENABLED && CLI_ENABLED == ENABLED
static bool check_startup_for_CLI()
{
    return (digitalReadFast(SLIDE_SWITCH_PIN) == 0);
}
#endif // CLI_ENABLED

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
    //cliSerial->println_P(PSTR("Invalid SERIAL3_BAUD"));
    return default_baud;
}

#if USB_MUX_PIN > 0
static void check_usb_mux(void)
{
    bool usb_check = !digitalReadFast(USB_MUX_PIN);
    if (usb_check == ap_system.usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    ap_system.usb_connected = usb_check;
    if (ap_system.usb_connected) {
        hal.uartA->begin(SERIAL0_BAUD);
    } else {
        hal.uartA->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD));
    }
}
#endif

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
    return board_vcc_analog_source->read_latest();
}

/*
  force a software reset of the APM
 */
static void reboot_apm(void) {
    hal.scheduler->reboot();
}

//
// print_flight_mode - prints flight mode to serial port.
//
static void
print_flight_mode(uint8_t mode)
{
    switch (mode) {
    case STABILIZE:
        cliSerial->print_P(PSTR("STABILIZE"));
        break;
    case ACRO:
        cliSerial->print_P(PSTR("ACRO"));
        break;
    case ALT_HOLD:
        cliSerial->print_P(PSTR("ALT_HOLD"));
        break;
    case AUTO:
        cliSerial->print_P(PSTR("AUTO"));
        break;
    case GUIDED:
        cliSerial->print_P(PSTR("GUIDED"));
        break;
    case LOITER:
        cliSerial->print_P(PSTR("LOITER"));
        break;
    case RTL:
        cliSerial->print_P(PSTR("RTL"));
        break;
    case CIRCLE:
        cliSerial->print_P(PSTR("CIRCLE"));
        break;
    case POSITION:
        cliSerial->print_P(PSTR("POSITION"));
        break;
    case LAND:
        cliSerial->print_P(PSTR("LAND"));
        break;
    case OF_LOITER:
        cliSerial->print_P(PSTR("OF_LOITER"));
        break;
    case TOY_M:
        cliSerial->print_P(PSTR("TOY_M"));
        break;
    case TOY_A:
        cliSerial->print_P(PSTR("TOY_A"));
        break;
    default:
        cliSerial->print_P(PSTR("---"));
        break;
    }
}
#line 1 "./Firmware/ACopter32/test.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t   test_radio_pwm(uint8_t argc,    const Menu::arg *argv);
static int8_t   test_radio(uint8_t argc,                const Menu::arg *argv);
//static int8_t	test_failsafe(uint8_t argc,     const Menu::arg *argv);
//static int8_t	test_stabilize(uint8_t argc,    const Menu::arg *argv);
static int8_t   test_gps(uint8_t argc,                  const Menu::arg *argv);
//static int8_t	test_tri(uint8_t argc,          const Menu::arg *argv);
//static int8_t	test_adc(uint8_t argc,          const Menu::arg *argv);
static int8_t   test_ins(uint8_t argc,                  const Menu::arg *argv);
//static int8_t	test_imu(uint8_t argc,          const Menu::arg *argv);
//static int8_t	test_dcm_eulers(uint8_t argc,   const Menu::arg *argv);
//static int8_t	test_dcm(uint8_t argc,          const Menu::arg *argv);
//static int8_t	test_omega(uint8_t argc,        const Menu::arg *argv);
//static int8_t	test_stab_d(uint8_t argc,       const Menu::arg *argv);
static int8_t   test_battery(uint8_t argc,              const Menu::arg *argv);
//static int8_t	test_toy(uint8_t argc,      const Menu::arg *argv);
static int8_t   test_wp_nav(uint8_t argc,               const Menu::arg *argv);
//static int8_t	test_reverse(uint8_t argc,      const Menu::arg *argv);
static int8_t   test_tuning(uint8_t argc,               const Menu::arg *argv);
static int8_t   test_relay(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_wp(uint8_t argc,                   const Menu::arg *argv);
#if HIL_MODE != HIL_MODE_ATTITUDE
static int8_t   test_baro(uint8_t argc,                 const Menu::arg *argv);
static int8_t   test_sonar(uint8_t argc,                const Menu::arg *argv);
#endif
static int8_t   test_mag(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_optflow(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_logging(uint8_t argc,              const Menu::arg *argv);
//static int8_t	test_xbee(uint8_t argc,         const Menu::arg *argv);
static int8_t   test_eedump(uint8_t argc,               const Menu::arg *argv);
static int8_t   test_rawgps(uint8_t argc,               const Menu::arg *argv);
//static int8_t	test_mission(uint8_t argc,      const Menu::arg *argv);

// this is declared here to remove compiler errors
extern void print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon);      // in Log.pde

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of printf that reads from flash memory
/*static int8_t	help_test(uint8_t argc,             const Menu::arg *argv)
 *  {
 *       cliSerial->printf_P(PSTR("\n"
 *                                                "Commands:\n"
 *                                                "  radio\n"
 *                                                "  servos\n"
 *                                                "  g_gps\n"
 *                                                "  imu\n"
 *                                                "  battery\n"
 *                                                "\n"));
 *  }*/

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command test_menu_commands[] PROGMEM = {
    {"pwm",                 test_radio_pwm},
    {"radio",               test_radio},
//	{"failsafe",	test_failsafe},
//	{"stabilize",	test_stabilize},
    {"gps",                 test_gps},
//	{"adc",         test_adc},
    {"ins",                 test_ins},
//	{"dcm",			test_dcm_eulers},
    //{"omega",		test_omega},
//	{"stab_d",		test_stab_d},
    {"battery",             test_battery},
    {"tune",                test_tuning},
    //{"tri",			test_tri},
    {"relay",               test_relay},
    {"wp",                  test_wp},
//	{"toy",			test_toy},
#if HIL_MODE != HIL_MODE_ATTITUDE
    {"altitude",    test_baro},
    {"sonar",               test_sonar},
#endif
    {"compass",             test_mag},
    {"optflow",             test_optflow},
    //{"xbee",		test_xbee},
    {"eedump",              test_eedump},
    {"logging",             test_logging},
//	{"rawgps",		test_rawgps},
//	{"mission",		test_mission},
    //{"reverse",		test_reverse},
    {"nav",                 test_wp_nav},
};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

static int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
    //cliSerial->printf_P(PSTR("Test Mode\n\n"));
    test_menu.run();
    return 0;
}

static int8_t
test_eedump(uint8_t argc, const Menu::arg *argv)
{

    // hexdump the EEPROM
    for (uint16_t i = 0; i < EEPROM_MAX_ADDR; i += 16) {
        cliSerial->printf_P(PSTR("%04x:"), i);
        for (uint16_t j = 0; j < 16; j++)  {
            int b = hal.storage->read_byte(i+j);
            cliSerial->printf_P(PSTR(" %02x"), b);
        }
        cliSerial->println();
    }
    return(0);
}


static int8_t
test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
#if defined( __AVR_ATmega1280__ )          // test disabled to save code size for 1280
    print_test_disabled();
    return (0);
#else
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);

        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        // servo Yaw
        //APM_RC.OutputCh(CH_7, g.rc_4.radio_out);

        cliSerial->printf_P(PSTR("IN: 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
                        g.rc_1.radio_in,
                        g.rc_2.radio_in,
                        g.rc_3.radio_in,
                        g.rc_4.radio_in,
                        g.rc_5.radio_in,
                        g.rc_6.radio_in,
                        g.rc_7.radio_in,
                        g.rc_8.radio_in);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
#endif
}

/*
 *  //static int8_t
 *  //test_tri(uint8_t argc, const Menu::arg *argv)
 *  {
 *       print_hit_enter();
 *       delay(1000);
 *
 *       while(1){
 *               delay(20);
 *
 *               // Filters radio input - adjust filters in the radio.pde file
 *               // ----------------------------------------------------------
 *               read_radio();
 *               g.rc_4.servo_out = g.rc_4.control_in;
 *               g.rc_4.calc_pwm();
 *
 *               cliSerial->printf_P(PSTR("input: %d\toutput%d\n"),
 *                                                       g.rc_4.control_in,
 *                                                       g.rc_4.radio_out);
 *
 *               APM_RC.OutputCh(CH_TRI_YAW, g.rc_4.radio_out);
 *
 *               if(cliSerial->available() > 0){
 *                       return (0);
 *               }
 *       }
 *  }*/


/*
//static int8_t
//test_toy(uint8_t argc, const Menu::arg *argv)
{
	set_alt_change(ASCENDING)

 	for(altitude_error = 2000; altitude_error > -100; altitude_error--){
 		int16_t temp = get_desired_climb_rate();
		cliSerial->printf("%ld, %d\n", altitude_error, temp);
	}
 	return 0;
}
{	wp_distance = 0;
	int16_t max_speed = 0;

 	for(int16_t i = 0; i < 200; i++){
	 	int32_t temp = 2 * 100 * (wp_distance - g.waypoint_radius * 100);
		max_speed = sqrtf((float)temp);
		max_speed = min(max_speed, g.waypoint_speed_max);
		cliSerial->printf("Zspeed: %ld, %d, %ld\n", temp, max_speed, wp_distance);
	 	wp_distance += 100;
	}
 	return 0;
 }
//*/

/*static int8_t
 *  //test_toy(uint8_t argc, const Menu::arg *argv)
 *  {
 *       int16_t yaw_rate;
 *       int16_t roll_rate;
 *       g.rc_1.control_in = -2500;
 *       g.rc_2.control_in = 2500;
 *
 *       g.toy_yaw_rate = 3;
 *       yaw_rate = g.rc_1.control_in / g.toy_yaw_rate;
 *       roll_rate = ((int32_t)g.rc_2.control_in * (yaw_rate/100)) /40;
 *       cliSerial->printf("yaw_rate, %d, roll_rate, %d\n", yaw_rate, roll_rate);
 *
 *       g.toy_yaw_rate = 2;
 *       yaw_rate = g.rc_1.control_in / g.toy_yaw_rate;
 *       roll_rate = ((int32_t)g.rc_2.control_in * (yaw_rate/100)) /40;
 *       cliSerial->printf("yaw_rate, %d, roll_rate, %d\n", yaw_rate, roll_rate);
 *
 *       g.toy_yaw_rate = 1;
 *       yaw_rate = g.rc_1.control_in / g.toy_yaw_rate;
 *       roll_rate = ((int32_t)g.rc_2.control_in * (yaw_rate/100)) /40;
 *       cliSerial->printf("yaw_rate, %d, roll_rate, %d\n", yaw_rate, roll_rate);
 *  }*/

static int8_t
test_radio(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);
        read_radio();


        cliSerial->printf_P(PSTR("IN  1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\n"),
                        g.rc_1.control_in,
                        g.rc_2.control_in,
                        g.rc_3.control_in,
                        g.rc_4.control_in,
                        g.rc_5.control_in,
                        g.rc_6.control_in,
                        g.rc_7.control_in);

        //cliSerial->printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d\n"), (g.rc_1.servo_out / 100), (g.rc_2.servo_out / 100), g.rc_3.servo_out, (g.rc_4.servo_out / 100));

        /*cliSerial->printf_P(PSTR(	"min: %d"
         *                                               "\t in: %d"
         *                                               "\t pwm_in: %d"
         *                                               "\t sout: %d"
         *                                               "\t pwm_out %d\n"),
         *                                               g.rc_3.radio_min,
         *                                               g.rc_3.control_in,
         *                                               g.rc_3.radio_in,
         *                                               g.rc_3.servo_out,
         *                                               g.rc_3.pwm_out
         *                                               );
         */
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

/*
 *  //static int8_t
 *  //test_failsafe(uint8_t argc, const Menu::arg *argv)
 *  {
 *
 * #if THROTTLE_FAILSAFE
 *       byte fail_test;
 *       print_hit_enter();
 *       for(int16_t i = 0; i < 50; i++){
 *               delay(20);
 *               read_radio();
 *       }
 *
 *       oldSwitchPosition = readSwitch();
 *
 *       cliSerial->printf_P(PSTR("Unplug battery, throttle in neutral, turn off radio.\n"));
 *       while(g.rc_3.control_in > 0){
 *               delay(20);
 *               read_radio();
 *       }
 *
 *       while(1){
 *               delay(20);
 *               read_radio();
 *
 *               if(g.rc_3.control_in > 0){
 *                       cliSerial->printf_P(PSTR("THROTTLE CHANGED %d \n"), g.rc_3.control_in);
 *                       fail_test++;
 *               }
 *
 *               if(oldSwitchPosition != readSwitch()){
 *                       cliSerial->printf_P(PSTR("CONTROL MODE CHANGED: "));
 *                       cliSerial->println(flight_mode_strings[readSwitch()]);
 *                       fail_test++;
 *               }
 *
 *               if(g.failsafe_throttle && g.rc_3.get_failsafe()){
 *                       cliSerial->printf_P(PSTR("THROTTLE FAILSAFE ACTIVATED: %d, "), g.rc_3.radio_in);
 *                       cliSerial->println(flight_mode_strings[readSwitch()]);
 *                       fail_test++;
 *               }
 *
 *               if(fail_test > 0){
 *                       return (0);
 *               }
 *               if(cliSerial->available() > 0){
 *                       cliSerial->printf_P(PSTR("LOS caused no change in ACM.\n"));
 *                       return (0);
 *               }
 *       }
 * #else
 *               return (0);
 * #endif
 *  }
 */

/*
 *  //static int8_t
 *  //test_stabilize(uint8_t argc, const Menu::arg *argv)
 *  {
 *       static byte ts_num;
 *
 *
 *       print_hit_enter();
 *       delay(1000);
 *
 *       // setup the radio
 *       // ---------------
 *       init_rc_in();
 *
 *       control_mode = STABILIZE;
 *       cliSerial->printf_P(PSTR("g.pi_stabilize_roll.kP: %4.4f\n"), g.pi_stabilize_roll.kP());
 *       cliSerial->printf_P(PSTR("max_stabilize_dampener:%d\n\n "), max_stabilize_dampener);
 *
 *       motors.auto_armed(false);
 *       motors.armed(true);
 *
 *       while(1){
 *               // 50 hz
 *               if (millis() - fast_loopTimer > 19) {
 *                       delta_ms_fast_loop     = millis() - fast_loopTimer;
 *                       fast_loopTimer		= millis();
 *                       G_Dt               = (float)delta_ms_fast_loop / 1000.f;
 *
 *                       if(g.compass_enabled){
 *                               medium_loopCounter++;
 *                               if(medium_loopCounter == 5){
 *                   Matrix3f m = dcm.get_dcm_matrix();
 *                                       compass.read();		                // Read magnetometer
 *                   compass.null_offsets();
 *                                       medium_loopCounter = 0;
 *                               }
 *                       }
 *
 *                       // for trim features
 *                       read_trim_switch();
 *
 *                       // Filters radio input - adjust filters in the radio.pde file
 *                       // ----------------------------------------------------------
 *                       read_radio();
 *
 *                       // IMU
 *                       // ---
 *                       read_AHRS();
 *
 *                       // allow us to zero out sensors with control switches
 *                       if(g.rc_5.control_in < 600){
 *                               dcm.roll_sensor = dcm.pitch_sensor = 0;
 *                       }
 *
 *                       // custom code/exceptions for flight modes
 *                       // ---------------------------------------
 *                       update_current_flight_mode();
 *
 *                       // write out the servo PWM values
 *                       // ------------------------------
 *                       set_servos_4();
 *
 *                       ts_num++;
 *                       if (ts_num > 10){
 *                               ts_num = 0;
 *                               cliSerial->printf_P(PSTR("r: %d, p:%d, rc1:%d, "),
 *                                       (int)(dcm.roll_sensor/100),
 *                                       (int)(dcm.pitch_sensor/100),
 *                                       g.rc_1.pwm_out);
 *
 *                               print_motor_out();
 *                       }
 *                       // R: 1417,  L: 1453  F: 1453  B: 1417
 *
 *                       //cliSerial->printf_P(PSTR("timer: %d, r: %d\tp: %d\t y: %d\n"), (int)delta_ms_fast_loop, ((int)dcm.roll_sensor/100), ((int)dcm.pitch_sensor/100), ((uint16_t)dcm.yaw_sensor/100));
 *                       //cliSerial->printf_P(PSTR("timer: %d, r: %d\tp: %d\t y: %d\n"), (int)delta_ms_fast_loop, ((int)dcm.roll_sensor/100), ((int)dcm.pitch_sensor/100), ((uint16_t)dcm.yaw_sensor/100));
 *
 *                       if(cliSerial->available() > 0){
 *                               if(g.compass_enabled){
 *                                       compass.save_offsets();
 *                                       report_compass();
 *                               }
 *                               return (0);
 *                       }
 *
 *               }
 *       }
 *  }
 */


/*
 * #if HIL_MODE != HIL_MODE_ATTITUDE && CONFIG_ADC == ENABLED
 *  //static int8_t
 *  //test_adc(uint8_t argc, const Menu::arg *argv)
 *  {
 *       print_hit_enter();
 *       cliSerial->printf_P(PSTR("ADC\n"));
 *       delay(1000);
 *
 *  adc.Init(&timer_scheduler);
 *
 *  delay(50);
 *
 *       while(1){
 *               for(int16_t i = 0; i < 9; i++){
 *                       cliSerial->printf_P(PSTR("%.1f,"),adc.Ch(i));
 *               }
 *               cliSerial->println();
 *               delay(20);
 *               if(cliSerial->available() > 0){
 *                       return (0);
 *               }
 *       }
 *  }
 * #endif
 */

static int8_t
test_ins(uint8_t argc, const Menu::arg *argv)
{
#if defined( __AVR_ATmega1280__ )          // test disabled to save code size for 1280
    print_test_disabled();
    return (0);
#else
    Vector3f gyro, accel;
    print_hit_enter();
    cliSerial->printf_P(PSTR("INS\n"));
    delay(1000);

    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate,
             flash_leds);

    delay(50);

    while(1) {
        ins.update();
        gyro = ins.get_gyro();
        accel = ins.get_accel();

        float test = accel.length() / GRAVITY_MSS;

        cliSerial->printf_P(PSTR("a %7.4f %7.4f %7.4f g %7.4f %7.4f %7.4f t %74f | %7.4f\n"),
            accel.x, accel.y, accel.z,
            gyro.x, gyro.y, gyro.z,
            test);

        delay(40);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
#endif
}

static int8_t
test_gps(uint8_t argc, const Menu::arg *argv)
{
    // test disabled to save code size for 1280
#if defined( __AVR_ATmega1280__ ) || HIL_MODE != HIL_MODE_DISABLED
    print_test_disabled();
    return (0);
#else
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(333);

        // Blink GPS LED if we don't have a fix
        // ------------------------------------
        update_GPS_light();

        g_gps->update();

        if (g_gps->new_data) {
            cliSerial->printf_P(PSTR("Lat: "));
            print_latlon(cliSerial, g_gps->latitude);
            cliSerial->printf_P(PSTR(", Lon "));
            print_latlon(cliSerial, g_gps->longitude);
            cliSerial->printf_P(PSTR(", Alt: %ldm, #sats: %d\n"),
                            g_gps->altitude/100,
                            g_gps->num_sats);
            g_gps->new_data = false;
        }else{
            cliSerial->print_P(PSTR("."));
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
#endif
}

/*
 *  //static int8_t
 *  //test_dcm(uint8_t argc, const Menu::arg *argv)
 *  {
 *       print_hit_enter();
 *       delay(1000);
 *       cliSerial->printf_P(PSTR("Gyro | Accel\n"));
 *       Vector3f   _cam_vector;
 *       Vector3f   _out_vector;
 *
 *       G_Dt = .02;
 *
 *       while(1){
 *               for(byte i = 0; i <= 50; i++){
 *                       delay(20);
 *                       // IMU
 *                       // ---
 *                       read_AHRS();
 *               }
 *
 *               Matrix3f temp = dcm.get_dcm_matrix();
 *               Matrix3f temp_t = dcm.get_dcm_transposed();
 *
 *               cliSerial->printf_P(PSTR("dcm\n"
 *                                                        "%4.4f \t %4.4f \t %4.4f \n"
 *                                                        "%4.4f \t %4.4f \t %4.4f \n"
 *                                                        "%4.4f \t %4.4f \t %4.4f \n\n"),
 *                                                       temp.a.x, temp.a.y, temp.a.z,
 *                                                       temp.b.x, temp.b.y, temp.b.z,
 *                                                       temp.c.x, temp.c.y, temp.c.z);
 *
 *               int16_t _pitch         = degrees(-asin(temp.c.x));
 *               int16_t _roll      = degrees(atan2f(temp.c.y, temp.c.z));
 *               int16_t _yaw       = degrees(atan2f(temp.b.x, temp.a.x));
 *               cliSerial->printf_P(PSTR(	"angles\n"
 *                                                               "%d \t %d \t %d\n\n"),
 *                                                               _pitch,
 *                                                               _roll,
 *                                                               _yaw);
 *
 *               //_out_vector = _cam_vector * temp;
 *               //cliSerial->printf_P(PSTR(	"cam\n"
 *               //						"%d \t %d \t %d\n\n"),
 *               //						(int)temp.a.x * 100, (int)temp.a.y * 100, (int)temp.a.x * 100);
 *
 *               if(cliSerial->available() > 0){
 *                       return (0);
 *               }
 *       }
 *  }
 */
/*
 *  //static int8_t
 *  //test_dcm(uint8_t argc, const Menu::arg *argv)
 *  {
 *       print_hit_enter();
 *       delay(1000);
 *       cliSerial->printf_P(PSTR("Gyro | Accel\n"));
 *       delay(1000);
 *
 *       while(1){
 *               Vector3f accels = dcm.get_accel();
 *               cliSerial->print("accels.z:");
 *               cliSerial->print(accels.z);
 *               cliSerial->print("omega.z:");
 *               cliSerial->print(omega.z);
 *               delay(100);
 *
 *               if(cliSerial->available() > 0){
 *                       return (0);
 *               }
 *       }
 *  }
 */

/*static int8_t
 *  //test_omega(uint8_t argc, const Menu::arg *argv)
 *  {
 *       static byte ts_num;
 *       float old_yaw;
 *
 *       print_hit_enter();
 *       delay(1000);
 *       cliSerial->printf_P(PSTR("Omega"));
 *       delay(1000);
 *
 *       G_Dt = .02;
 *
 *       while(1){
 *               delay(20);
 *               // IMU
 *               // ---
 *               read_AHRS();
 *
 *               float my_oz = (dcm.yaw - old_yaw) * 50;
 *
 *               old_yaw = dcm.yaw;
 *
 *               ts_num++;
 *               if (ts_num > 2){
 *                       ts_num = 0;
 *                       //cliSerial->printf_P(PSTR("R: %4.4f\tP: %4.4f\tY: %4.4f\tY: %4.4f\n"), omega.x, omega.y, omega.z, my_oz);
 *                       cliSerial->printf_P(PSTR(" Yaw: %ld\tY: %4.4f\tY: %4.4f\n"), dcm.yaw_sensor, omega.z, my_oz);
 *               }
 *
 *               if(cliSerial->available() > 0){
 *                       return (0);
 *               }
 *       }
 *       return (0);
 *  }
 *  //*/

static int8_t
test_tuning(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();

    while(1) {
        delay(200);
        read_radio();
        tuning();
        cliSerial->printf_P(PSTR("tune: %1.3f\n"), tuning_value);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_battery(uint8_t argc, const Menu::arg *argv)
{
#if defined( __AVR_ATmega1280__ )          // disable this test if we are using 1280
    print_test_disabled();
    return (0);
#else
    cliSerial->printf_P(PSTR("\nCareful! Motors will spin! Press Enter to start.\n"));
    while (cliSerial->read() != -1); /* flush */
    while(!cliSerial->available()) { /* wait for input */
        delay(100);
    }
    while (cliSerial->read() != -1); /* flush */
    print_hit_enter();

    // allow motors to spin
    motors.enable();
    motors.armed(true);

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
            cliSerial->printf_P(PSTR("V: %4.4f, A: %4.4f, Ah: %4.4f\n"),
                            battery_voltage1,
                            current_amps1,
                            current_total1);
        }
        motors.throttle_pass_through();

        if(cliSerial->available() > 0) {
            motors.armed(false);
            return (0);
        }
    }
    motors.armed(false);
    return (0);
#endif
}

static int8_t test_relay(uint8_t argc, const Menu::arg *argv)
{
#if defined( __AVR_ATmega1280__ )          // test disabled to save code size for 1280
    print_test_disabled();
    return (0);
#else

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
#endif
}


static int8_t
test_wp(uint8_t argc, const Menu::arg *argv)
{
    delay(1000);

    // save the alitude above home option
    cliSerial->printf_P(PSTR("Hold alt "));
    if(g.rtl_altitude < 0) {
        cliSerial->printf_P(PSTR("\n"));
    }else{
        cliSerial->printf_P(PSTR("of %dm\n"), (int)g.rtl_altitude / 100);
    }

    cliSerial->printf_P(PSTR("%d wp\n"), (int)g.command_total);
    cliSerial->printf_P(PSTR("Hit rad: %dm\n"), (int)g.waypoint_radius);
    //cliSerial->printf_P(PSTR("Loiter radius: %d\n\n"), (int)g.loiter_radius);

    report_wp();

    return (0);
}

//static int8_t test_rawgps(uint8_t argc, const Menu::arg *argv) {
/*
 *  print_hit_enter();
 *  delay(1000);
 *  while(1){
 *          if (Serial3.available()){
 *                          digitalWrite(B_LED_PIN, LED_ON); // Blink Yellow LED if we are sending data to GPS
 *                          Serial1.write(Serial3.read());
 *                          digitalWrite(B_LED_PIN, LED_OFF);
 *          }
 *          if (Serial1.available()){
 *                          digitalWrite(C_LED_PIN, LED_ON); // Blink Red LED if we are receiving data from GPS
 *                          Serial3.write(Serial1.read());
 *                          digitalWrite(C_LED_PIN, LED_OFF);
 *          }
 *          if(cliSerial->available() > 0){
 *                          return (0);
 *  }
 *  }
 */
//}

/*static int8_t
 *  //test_xbee(uint8_t argc, const Menu::arg *argv)
 *  {
 *       print_hit_enter();
 *       delay(1000);
 *       cliSerial->printf_P(PSTR("Begin XBee X-CTU Range and RSSI Test:\n"));
 *
 *       while(1){
 *               if (Serial3.available())
 *                       Serial3.write(Serial3.read());
 *
 *               if(cliSerial->available() > 0){
 *                       return (0);
 *               }
 *       }
 *  }
 */

#if HIL_MODE != HIL_MODE_ATTITUDE
static int8_t
test_baro(uint8_t argc, const Menu::arg *argv)
{
 #if defined( __AVR_ATmega1280__ )         // test disabled to save code size for 1280
    print_test_disabled();
    return (0);
 #else
    print_hit_enter();
    init_barometer();

    while(1) {
        delay(100);
        int32_t alt = read_barometer();                 // calls barometer.read()

        float pres = barometer.get_pressure();
        int16_t temp = barometer.get_temperature();
        int32_t raw_pres = barometer.get_raw_pressure();
        int32_t raw_temp = barometer.get_raw_temp();
        cliSerial->printf_P(PSTR("alt: %ldcm, pres: %fmbar, temp: %d/100degC,"
                             " raw pres: %ld, raw temp: %ld\n"),
                            (long)alt, pres, (int)temp, (long)raw_pres, (long)raw_temp);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
 #endif
}
#endif


static int8_t
test_mag(uint8_t argc, const Menu::arg *argv)
{
#if defined( __AVR_ATmega1280__ )          // test disabled to save code size for 1280
    print_test_disabled();
    return (0);
#else
    if(g.compass_enabled) {
        print_hit_enter();

        while(1) {
            delay(100);
            if (compass.read()) {
                float heading = compass.calculate_heading(ahrs.get_dcm_matrix());
                cliSerial->printf_P(PSTR("Heading: %ld, XYZ: %d, %d, %d\n"),
                                (wrap_360(ToDeg(heading) * 100)) /100,
                                compass.mag_x,
                                compass.mag_y,
                                compass.mag_z);
            } else {
                cliSerial->println_P(PSTR("not healthy"));
            }

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    } else {
        cliSerial->printf_P(PSTR("Compass: "));
        print_enabled(false);
        return (0);
    }
    return (0);
#endif
}

/*
 *  //static int8_t
 *  //test_reverse(uint8_t argc,        const Menu::arg *argv)
 *  {
 *       print_hit_enter();
 *       delay(1000);
 *
 *       while(1){
 *               delay(20);
 *
 *               // Filters radio input - adjust filters in the radio.pde file
 *               // ----------------------------------------------------------
 *               g.rc_4.set_reverse(0);
 *               g.rc_4.set_pwm(APM_RC.InputCh(CH_4));
 *               g.rc_4.servo_out = g.rc_4.control_in;
 *               g.rc_4.calc_pwm();
 *               cliSerial->printf_P(PSTR("PWM:%d input: %d\toutput%d "),
 *                                                       APM_RC.InputCh(CH_4),
 *                                                       g.rc_4.control_in,
 *                                                       g.rc_4.radio_out);
 *               APM_RC.OutputCh(CH_6, g.rc_4.radio_out);
 *
 *
 *               g.rc_4.set_reverse(1);
 *               g.rc_4.set_pwm(APM_RC.InputCh(CH_4));
 *               g.rc_4.servo_out = g.rc_4.control_in;
 *               g.rc_4.calc_pwm();
 *               cliSerial->printf_P(PSTR("\trev input: %d\toutput%d\n"),
 *                                                       g.rc_4.control_in,
 *                                                       g.rc_4.radio_out);
 *
 *               APM_RC.OutputCh(CH_7, g.rc_4.radio_out);
 *
 *               if(cliSerial->available() > 0){
 *                       g.rc_4.set_reverse(0);
 *                       return (0);
 *               }
 *       }
 *  }*/

#if HIL_MODE != HIL_MODE_ATTITUDE
/*
 *  test the sonar
 */
static int8_t
test_sonar(uint8_t argc, const Menu::arg *argv)
{
    if(g.sonar_enabled == false) {
        cliSerial->printf_P(PSTR("Sonar disabled\n"));
        return (0);
    }

    // make sure sonar is initialised
    init_sonar();

    print_hit_enter();
    while(1) {
        delay(100);

        cliSerial->printf_P(PSTR("Sonar: %d cm\n"), sonar->read());
        //cliSerial->printf_P(PSTR("Sonar, %d, %d\n"), sonar.read(), sonar.raw_value);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }

    return (0);
}
#endif


static int8_t
test_optflow(uint8_t argc, const Menu::arg *argv)
{
#if OPTFLOW == ENABLED
    if(g.optflow_enabled) {
        cliSerial->printf_P(PSTR("man id: %d\t"),optflow.read_register(ADNS3080_PRODUCT_ID));
        print_hit_enter();

        while(1) {
            delay(200);
            optflow.update(millis());
            Log_Write_Optflow();
            cliSerial->printf_P(PSTR("x/dx: %d/%d\t y/dy %d/%d\t squal:%d\n"),
                            optflow.x,
                            optflow.dx,
                            optflow.y,
                            optflow.dy,
                            optflow.surface_quality);

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    } else {
        cliSerial->printf_P(PSTR("OptFlow: "));
        print_enabled(false);
    }
    return (0);

#else
    print_test_disabled();
    return (0);
#endif      // OPTFLOW == ENABLED
}


static int8_t
test_wp_nav(uint8_t argc, const Menu::arg *argv)
{
    current_loc.lat = 389539260;
    current_loc.lng = -1199540200;

    next_WP.lat = 389538528;
    next_WP.lng = -1199541248;

    // got 23506;, should be 22800
    update_navigation();
    cliSerial->printf_P(PSTR("bear: %ld\n"), wp_bearing);
    return 0;
}

/*
 *  test the dataflash is working
 */

static int8_t
test_logging(uint8_t argc, const Menu::arg *argv)
{
#if defined( __AVR_ATmega1280__ )          // test disabled to save code size for 1280
    print_test_disabled();
    return (0);
#else
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
#endif
}


/*
 *  static int8_t
 *  //test_mission(uint8_t argc, const Menu::arg *argv)
 *  {
 *       //write out a basic mission to the EEPROM
 *
 *  //{
 *  //	uint8_t		id;					///< command id
 *  //	uint8_t		options;			///< options bitmask (1<<0 = relative altitude)
 *  //	uint8_t		p1;					///< param 1
 *  //	int32_t		alt;				///< param 2 - Altitude in centimeters (meters * 100)
 *  //	int32_t		lat;				///< param 3 - Lattitude * 10**7
 *  //	int32_t		lng;				///< param 4 - Longitude * 10**7
 *  //}
 *
 *       // clear home
 *       {Location t = {0,      0,	  0,        0,      0,          0};
 *       set_cmd_with_index(t,0);}
 *
 *       // CMD										opt						pitch       alt/cm
 *       {Location t = {MAV_CMD_NAV_TAKEOFF,        WP_OPTION_RELATIVE,	  0,        100,        0,          0};
 *       set_cmd_with_index(t,1);}
 *
 *       if (!strcmp_P(argv[1].str, PSTR("wp"))) {
 *
 *               // CMD											opt
 *               {Location t = {MAV_CMD_NAV_WAYPOINT,           WP_OPTION_RELATIVE,		15, 0, 0, 0};
 *               set_cmd_with_index(t,2);}
 *               // CMD											opt
 *               {Location t = {MAV_CMD_NAV_RETURN_TO_LAUNCH,       WP_OPTION_YAW,	  0,        0,      0,		0};
 *               set_cmd_with_index(t,3);}
 *
 *               // CMD											opt
 *               {Location t = {MAV_CMD_NAV_LAND,				0,	  0,        0,      0,		0};
 *               set_cmd_with_index(t,4);}
 *
 *       } else {
 *               //2250 = 25 meteres
 *               // CMD										opt		p1		//alt		//NS		//WE
 *               {Location t = {MAV_CMD_NAV_LOITER_TIME,    0,	  10,   0,          0,			0}; // 19
 *               set_cmd_with_index(t,2);}
 *
 *               // CMD										opt		dir		angle/deg	deg/s	relative
 *               {Location t = {MAV_CMD_CONDITION_YAW,		0,	  1,        360,        60,     1};
 *               set_cmd_with_index(t,3);}
 *
 *               // CMD										opt
 *               {Location t = {MAV_CMD_NAV_LAND,			0,	  0,        0,          0,      0};
 *               set_cmd_with_index(t,4);}
 *
 *       }
 *
 *       g.rtl_altitude.set_and_save(300);
 *       g.command_total.set_and_save(4);
 *       g.waypoint_radius.set_and_save(3);
 *
 *       test_wp(NULL, NULL);
 *       return (0);
 *  }
 */

static void print_hit_enter()
{
    cliSerial->printf_P(PSTR("Hit Enter to exit.\n\n"));
}

static void print_test_disabled()
{
    cliSerial->printf_P(PSTR("Sorry, not 1280 compat.\n"));
}

/*
 *  //static void fake_out_gps()
 *  {
 *       static float rads;
 *       g_gps->new_data    = true;
 *       g_gps->fix	    = true;
 *
 *       //int length = g.rc_6.control_in;
 *       rads += .05;
 *
 *       if (rads > 6.28){
 *               rads = 0;
 *       }
 *
 *       g_gps->latitude	= 377696000;	// Y
 *       g_gps->longitude	= -1224319000;	// X
 *       g_gps->altitude	= 9000;			// meters * 100
 *
 *       //next_WP.lng	    = home.lng - length * sin(rads);   // X
 *       //next_WP.lat  = home.lat + length * cos(rads);   // Y
 *  }
 *
 */
/*
 *  //static void print_motor_out(){
 *       cliSerial->printf("out: R: %d,  L: %d  F: %d  B: %d\n",
 *                               (motor_out[CH_1]   - g.rc_3.radio_min),
 *                               (motor_out[CH_2]   - g.rc_3.radio_min),
 *                               (motor_out[CH_3]   - g.rc_3.radio_min),
 *                               (motor_out[CH_4]   - g.rc_3.radio_min));
 *  }
 */
#endif // CLI_ENABLED
#line 1 "./Firmware/ACopter32/toy.pde"
////////////////////////////////////////////////////////////////////////////////
// Toy Mode - THOR
////////////////////////////////////////////////////////////////////////////////


//called at 10hz
void update_toy_throttle()
{
    /*
     *  // Disabled, now handled by TOY_A (Alt hold) and TOY_M (Manual throttle)
     *  if (false == CH6_toy_flag && g.rc_6.radio_in >= CH_6_PWM_TRIGGER){
     *       CH6_toy_flag = true;
     *       throttle_mode  = THROTTLE_MANUAL;
     *
     *  }else if (CH6_toy_flag && g.rc_6.radio_in < CH_6_PWM_TRIGGER){
     *       CH6_toy_flag = false;
     *       throttle_mode  = THROTTLE_AUTO;
     *       set_new_altitude(current_loc.alt);
     *       saved_toy_throttle = g.rc_3.control_in;
     *  }*/

    // look for a change in throttle position to exit throttle hold
    if(abs(g.rc_3.control_in - saved_toy_throttle) > 40) {
        throttle_mode   = THROTTLE_MANUAL;
    }
}

#define TOY_ALT_SMALL 25
#define TOY_ALT_LARGE 100

//called at 10hz
void update_toy_altitude()
{
    int16_t input = g.rc_3.radio_in;     // throttle
    //int16_t input = g.rc_7.radio_in;

    // Trigger upward alt change
    if(false == CH7_toy_flag && input > 1666) {
        CH7_toy_flag = true;
        // go up
        if(next_WP.alt >= 400) {
            force_new_altitude(next_WP.alt + TOY_ALT_LARGE);
        }else{
            force_new_altitude(next_WP.alt + TOY_ALT_SMALL);
        }

        // Trigger downward alt change
    }else if(false == CH7_toy_flag && input < 1333) {
        CH7_toy_flag = true;
        // go down
        if(next_WP.alt >= (400 + TOY_ALT_LARGE)) {
            force_new_altitude(next_WP.alt - TOY_ALT_LARGE);
        }else if(next_WP.alt >= TOY_ALT_SMALL) {
            force_new_altitude(next_WP.alt - TOY_ALT_SMALL);
        }else if(next_WP.alt < TOY_ALT_SMALL) {
            force_new_altitude(0);
        }

        // clear flag
    }else if (CH7_toy_flag && ((input < 1666) && (input > 1333))) {
        CH7_toy_flag = false;
    }
}

// called at 50 hz from all flight modes
#if TOY_EDF == ENABLED
void edf_toy()
{
    // EDF control:
    g.rc_8.radio_out = 1000 + ((abs(g.rc_2.control_in) << 1) / 9);
    if(g.rc_8.radio_out < 1050)
        g.rc_8.radio_out = 1000;

    // output throttle to EDF
    if(motors.armed()) {
        APM_RC.OutputCh(CH_8, g.rc_8.radio_out);
    }else{
        APM_RC.OutputCh(CH_8, 1000);
    }

    // output Servo direction
    if(g.rc_2.control_in > 0) {
        APM_RC.OutputCh(CH_6, 1000);         // 1000 : 2000
    }else{
        APM_RC.OutputCh(CH_6, 2000);         // less than 1450
    }
}
#endif

// The function call for managing the flight mode Toy
void roll_pitch_toy()
{
#if TOY_MIXER == TOY_LOOKUP_TABLE || TOY_MIXER == TOY_LINEAR_MIXER
    int16_t yaw_rate = g.rc_1.control_in / g.toy_yaw_rate;

    if(g.rc_1.control_in != 0) {    // roll
        get_acro_yaw(yaw_rate/2);
        ap_system.yaw_stopped = false;
        yaw_timer = 150;

    }else if (!ap_system.yaw_stopped) {
        get_acro_yaw(0);
        yaw_timer--;

        if((yaw_timer == 0) || (fabsf(omega.z) < 0.17f)) {
            ap_system.yaw_stopped = true;
            nav_yaw = ahrs.yaw_sensor;
        }
    }else{
        if(motors.armed() == false || g.rc_3.control_in == 0)
            nav_yaw = ahrs.yaw_sensor;

        get_stabilize_yaw(nav_yaw);
    }
#endif

    // roll_rate is the outcome of the linear equation or lookup table
    // based on speed and Yaw rate
    int16_t roll_rate = 0;

#if TOY_MIXER == TOY_LOOKUP_TABLE
    uint8_t xx, yy;
    // Lookup value
    //xx	= g_gps->ground_speed / 200;
    xx      = abs(g.rc_2.control_in / 1000);
    yy      = abs(yaw_rate / 500);

    // constrain to lookup Array range
    xx = constrain(xx, 0, 3);
    yy = constrain(yy, 0, 8);

    roll_rate = toy_lookup[yy * 4 + xx];

    if(yaw_rate == 0) {
        roll_rate = 0;
    }else if(yaw_rate < 0) {
        roll_rate = -roll_rate;
    }

    int16_t roll_limit = 4500 / g.toy_yaw_rate;
    roll_rate = constrain(roll_rate, -roll_limit, roll_limit);

#elif TOY_MIXER == TOY_LINEAR_MIXER
    roll_rate = -((int32_t)g.rc_2.control_in * (yaw_rate/100)) /30;
    //cliSerial->printf("roll_rate: %d\n",roll_rate);
    roll_rate = constrain(roll_rate, -2000, 2000);

#elif TOY_MIXER == TOY_EXTERNAL_MIXER
    // JKR update to allow external roll/yaw mixing
    roll_rate = g.rc_1.control_in;
#endif

#if TOY_EDF == ENABLED
    // Output the attitude
    //g.rc_1.servo_out = get_stabilize_roll(roll_rate);
    //g.rc_2.servo_out = get_stabilize_pitch(g.rc_6.control_in);             // use CH6 to trim pitch
    get_stabilize_roll(roll_rate);
    get_stabilize_pitch(g.rc_6.control_in);             // use CH6 to trim pitch

#else
    // Output the attitude
    //g.rc_1.servo_out = get_stabilize_roll(roll_rate);
    //g.rc_2.servo_out = get_stabilize_pitch(g.rc_2.control_in);
    get_stabilize_roll(roll_rate);
    get_stabilize_pitch(g.rc_2.control_in);
#endif

}
