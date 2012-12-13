/** AP_InertialSensor_VRIMU
 This class is developed by Roberto Navoni a member of Arducopter DevTeam
 This class is compatible with VR NAVI FULL Sensor Board.
 It's use for retrival the data from Gyro and Acc Sensor.
**/
#ifndef __AP_INERTIAL_SENSOR_VRIMU_H__
#define __AP_INERTIAL_SENSOR_VRIMU_H__

#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include <wirish.h>
#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"
#include "../AP_Math/AP_Math.h"
#include "AP_InertialSensor.h"

#define SPI2_CS		77 // GPIO9  PE10
#define SPI2_BUSY	99 // GPIO18 PD10
#define SPI2_CLK	3
#define SPI2_MISO	4
#define SPI2_MOSI	5


class AP_InertialSensor_VRIMU : public AP_InertialSensor
{
public:

  AP_InertialSensor_VRIMU(uint8_t cs_pin, HardwareSPI *spi_dev, FastSerial *ser_port);

  uint16_t init( AP_PeriodicProcess * scheduler );

  /* Concrete implementation of AP_InertialSensor functions: */
  /// Update the data from the Sensor and apply the filter on read data
  bool update();
  //is there new data available?
  bool new_data_available();
  /// Get matrix of X,Y,Z accelerometer data in one call  
  void get_accels( float * );
  /// Get sensor data ax,ay,az,gx,gy,gz accelerometer data in one call  
  void get_sensors( float * );
  /// Get Temperature from sensor
  float temperature();
  /// Get Sample Time
  uint32_t sample_time();
  /// Reset Sample Time
  void reset_sample_time();
    float get_gyro_drift_rate();
	
    // get number of samples read from the sensors
    uint16_t        num_samples_available();
 // static const struct AP_Param::GroupInfo var_info[];
  
  /// Read the data from sensor and put it in sum[n] matrix.
  void read();
  /// Check if the automatic Read using the timer is active or not.
  bool automaticRead();


  static HardwareSPI *_SPIx;
  static void read(uint32_t time);
private:

  static int16_t spi_transfer_16(void);
  
  static void data_interrupt(void);
  static uint8_t register_read( uint8_t reg );
  static void register_write( uint8_t reg, uint8_t val );
  static void hardware_init();

  AP_PeriodicProcess * _scheduler;

  Vector3f _gyro;
  Vector3f _accel;
  float _temp;

  uint32_t _last_sample_micros;

  float _temp_to_celsius( uint16_t );

  static const float _accel_scale;
  static const float _gyro_scale;

  static const uint8_t _gyro_data_index[3];
  static const  int8_t _gyro_data_sign[3];

  static const uint8_t _accel_data_index[3];
  static const  int8_t _accel_data_sign[3];

  static const uint8_t _temp_data_index;

  static int16_t _data[20];

  /* TODO deprecate _cs_pin */
  static uint8_t _cs_pin;

  // ensure we can't initialise twice
  unsigned _initialised:1;

  float _gyro_apply_std_offset( uint16_t adc_value );
  float _accel_apply_std_offset( uint16_t adc_value );
};

#endif // __AP_INERTIAL_SENSOR_VRIMU_H__
