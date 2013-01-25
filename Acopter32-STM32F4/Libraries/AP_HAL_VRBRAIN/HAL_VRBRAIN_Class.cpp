
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include "HAL_VRBRAIN_Class.h"
#include "AP_HAL_VRBRAIN_Private.h"
#include <usart.h>

using namespace VRBRAIN;

// XXX make sure these are assigned correctly
static VRBRAINUARTDriver uartADriver(_USART3);
static VRBRAINUARTDriver uartBDriver(_USART2);
static VRBRAINUARTDriver uartCDriver(_UART4);
static VRBRAINSemaphore  i2cSemaphore;
static VRBRAINI2CDriver  i2cDriver(&i2cSemaphore);
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
        &utilInstance),
    _member(new VRBRAINPrivateMember(123))
{}

void HAL_VRBRAIN::init(int argc,char* const argv[]) const
{
  /* initialize all drivers and private members here.
   * up to the programmer to do this in the correct order.
   * Scheduler should likely come first. */
  scheduler->init(NULL);
  uartA->begin(115200);
    _member->init();
}

const HAL_VRBRAIN AP_HAL_VRBRAIN;

#endif
