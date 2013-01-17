
#include "SPIDriver.h"

using namespace VRBRAIN;

VRBRAINSPIDeviceDriver::VRBRAINSPIDeviceDriver()
{}

void VRBRAINSPIDeviceDriver::init()
{}

AP_HAL::Semaphore* VRBRAINSPIDeviceDriver::get_semaphore()
{
    return &_semaphore;
}

void VRBRAINSPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{}


void VRBRAINSPIDeviceDriver::cs_assert()
{}

void VRBRAINSPIDeviceDriver::cs_release()
{}

uint8_t VRBRAINSPIDeviceDriver::transfer (uint8_t data)
{
    return 0;
}

VRBRAINSPIDeviceManager::VRBRAINSPIDeviceManager()
{}

void VRBRAINSPIDeviceManager::init(void *)
{}

AP_HAL::SPIDeviceDriver* VRBRAINSPIDeviceManager::device(enum AP_HAL::SPIDevice)
{
    return &_device;
}

