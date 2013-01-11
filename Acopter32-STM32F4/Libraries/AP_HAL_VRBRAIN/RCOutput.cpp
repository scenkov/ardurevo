
#include "RCOutput.h"

using namespace VRBRAIN;

void VRBRAINRCOutput::init(void* machtnichts) {}

void VRBRAINRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t VRBRAINRCOutput::get_freq(uint8_t ch) {
    return 50;
}

void VRBRAINRCOutput::enable_ch(uint8_t ch)
{}

void VRBRAINRCOutput::enable_mask(uint32_t chmask)
{}

void VRBRAINRCOutput::disable_ch(uint8_t ch)
{}

void VRBRAINRCOutput::disable_mask(uint32_t chmask)
{}

void VRBRAINRCOutput::write(uint8_t ch, uint16_t period_us)
{}

void VRBRAINRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{}

uint16_t VRBRAINRCOutput::read(uint8_t ch) {
    return 900;
}

void VRBRAINRCOutput::read(uint16_t* period_us, uint8_t len)
{}

