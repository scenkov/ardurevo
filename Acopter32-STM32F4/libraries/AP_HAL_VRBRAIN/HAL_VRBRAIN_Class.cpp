
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include <AP_HAL_VRBRAIN.h>
#include "AP_HAL_VRBRAIN_Namespace.h"
#include "HAL_VRBRAIN_Class.h"
#include "AP_HAL_VRBRAIN_Private.h"
#include "Util.h"
#include <AP_HAL_Empty.h>
#include <AP_HAL_Empty_Private.h>
#include <pwm_in.h>
#include <usart.h>
#include <i2c.h>
#include <AP_Compass.h>

using namespace VRBRAIN;

//_USART1 PIN 3 AND 4 OF THE INPUT RAIL
//_USART2 INTERNAL SERIAL PORT
//_USART3 PIN 1 AND 2 OF THE INPUT RAIL
//_USART6 PIN 5 AND 6 on the INPUT RAIL


// XXX make sure these are assigned correctly
static VRBRAINUARTDriver uartADriver(_USART1,1);
static VRBRAINUARTDriver uartBDriver(_USART3,0);
static VRBRAINUARTDriver uartCDriver(_USART2,0);
static VRBRAINUARTDriver uartDDriver(_USART6,0);
static VRBRAINSemaphore  i2cSemaphore;
static VRBRAINSemaphore  i2c2Semaphore;
static VRBRAINI2CDriver  i2cDriver(_I2C2,&i2cSemaphore);
static VRBRAINI2CDriver  i2c2Driver(_I2C1,&i2c2Semaphore);
static VRBRAINSPIDeviceManager spiDeviceManager;
static VRBRAINAnalogIn analogIn;
static VRBRAINStorage storageDriver(_I2C2);
static VRBRAINGPIO gpioDriver;
static VRBRAINRCInput rcinDriver;
static VRBRAINRCOutput rcoutDriver;
static VRBRAINScheduler schedulerInstance;
static VRBRAINUtil utilInstance;

uint8_t g_ext_mag_detect;

HAL_VRBRAIN::HAL_VRBRAIN() :
    AP_HAL::HAL(
      &uartADriver,
      &uartBDriver,
      &uartCDriver,
      &uartDDriver,
	  NULL,
      &i2cDriver,
      &i2c2Driver,
      &spiDeviceManager,
      &analogIn,
      &storageDriver,
      &uartADriver,
      &gpioDriver,
      &rcinDriver,
      &rcoutDriver,
      &schedulerInstance,
      &utilInstance
	  )
{}

extern const AP_HAL::HAL& hal;

/*Returns true if an external mag on I2C2 port has been detected*/
static void detect_compass(void){

    AP_Compass_HMC5843_EXT compass_ext;
    AP_Compass_HMC5843 compass;

    hal.scheduler->delay(1000);

    if(compass_ext.init()){
	hal.console->printf_P(PSTR("External Compass found!"));
	g_ext_mag_detect = 1;
	return;
    }
    if(compass.init()){
	hal.console->printf_P(PSTR("Internal Compass found!"));
	g_ext_mag_detect = 0;
	return;
	}

    //No compass found
    g_ext_mag_detect = 3;
}

void HAL_VRBRAIN::init(int argc,char* const argv[]) const
{
  /* initialize all drivers and private members here.
   * up to the programmer to do this in the correct order.
   * Scheduler should likely come first. */
  //delay_us(2000000);
  scheduler->init(NULL);

  uartA->begin(57600);
  //uartC->begin(57600);

  //uartC->set_blocking_writes(true);

  g_ext_mag_detect = 0;

  //_member->init();
  i2c->begin();
  i2c2->begin();

  spi->init(NULL);

  detect_compass();

  analogin->init(NULL);
  storage->init(NULL);
  rcin->init(NULL);
  rcout->init(NULL);

}

const HAL_VRBRAIN AP_HAL_VRBRAIN;

#endif
