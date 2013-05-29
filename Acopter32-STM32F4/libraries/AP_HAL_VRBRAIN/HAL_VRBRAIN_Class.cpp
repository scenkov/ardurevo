
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include "HAL_VRBRAIN_Class.h"
#include "AP_HAL_VRBRAIN_Private.h"
#include <AP_HAL_Empty.h>
#include <AP_HAL_Empty_Private.h>
#include <usart.h>
#include <i2c.h>

using namespace VRBRAIN;

// XXX make sure these are assigned correctly
static VRBRAINUARTDriver uartADriver(_USART1,1);
static VRBRAINUARTDriver uartBDriver(_USART3,0);
//static Empty::EmptyUARTDriver uartBDriver;
//static Empty::EmptyUARTDriver uartCDriver;
static VRBRAINUARTDriver uartCDriver(_USART2,0);
static VRBRAINSemaphore  i2cSemaphore;
static VRBRAINI2CDriver  i2cDriver(_I2C2,&i2cSemaphore);
static VRBRAINSPIDeviceManager spiDeviceManager;
static VRBRAINAnalogIn analogIn;
static VRBRAINStorage storageDriver;
static VRBRAINConsoleDriver consoleDriver;
static VRBRAINGPIO gpioDriver;
static VRBRAINRCInput rcinDriver;
static VRBRAINRCOutput rcoutDriver;
static VRBRAINScheduler schedulerInstance;
static VRBRAINUtil utilInstance;



HAL_VRBRAIN::HAL_VRBRAIN() :
    AP_HAL::HAL(
      &uartADriver,
      &uartBDriver,
      &uartCDriver,
      &i2cDriver,
      &spiDeviceManager,
      &analogIn,
      &storageDriver,
      &consoleDriver,
      &gpioDriver,
      &rcinDriver,
      &rcoutDriver,
      &schedulerInstance,
      &utilInstance)//,
    //_member(new VRBRAINPrivateMember(123))
{}

extern const AP_HAL::HAL& hal;

void HAL_VRBRAIN::init(int argc,char* const argv[]) const
{
  /* initialize all drivers and private members here.
   * up to the programmer to do this in the correct order.
   * Scheduler should likely come first. */
  scheduler->init(NULL);
  //uartA->begin(115200);

  hal.uartA->begin(115200);
  hal.uartB->begin(38400);
  hal.uartC->begin(57600);

  console->init((void *)hal.uartA);
  //_member->init();
  i2c->begin();
  spi->init(NULL);
  storage->init(NULL);
  rcin->init(NULL);
  rcout->init(NULL);

}

const HAL_VRBRAIN AP_HAL_VRBRAIN;

#endif
