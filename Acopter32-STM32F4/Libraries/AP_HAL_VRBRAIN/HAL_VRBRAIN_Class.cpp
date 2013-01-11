
#include <AP_HAL.h>

#include "HAL_VRBRAIN_Class.h"
#include "AP_HAL_VRBRAIN_Private.h"

using namespace VRBRAIN;

// XXX make sure these are assigned correctly
static VRBRAINUARTDriver uartADriver(usart1);
static VRBRAINUARTDriver uartBDriver(usart2);
static VRBRAINUARTDriver uartCDriver(usart3);

static VRBRAINI2CDriver  i2cDriver;
static VRBRAINSPIDeviceManager spiDeviceManager;
static VRBRAINAnalogIn analogIn;
static VRBRAINStorage storageDriver;
static VRBRAINConsoleDriver consoleDriver(&uartADriver);
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
      &utilInstance)
{
}

void HAL_VRBRAIN::init(int argc,char* const argv[]) const
{
  /* initialize all drivers and private members here.
   * up to the programmer to do this in the correct order.
   * Scheduler should likely come first. */
  scheduler->init(NULL);
  uartA->begin(115200);
  console->init(uartA);
  i2c->begin();
  spi->init(NULL);
  storage->init(NULL);
}

const HAL_VRBRAIN AP_HAL_VRBRAIN;

