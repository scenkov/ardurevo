
#include <string.h>
#include "Storage.h"

using namespace VRBRAIN;

VRBRAINStorage::VRBRAINStorage()
{}

void VRBRAINStorage::init(void*)
{}

uint8_t VRBRAINStorage::read_byte(uint16_t loc){
    return 0;
}

uint16_t VRBRAINStorage::read_word(uint16_t loc){
    return 0;
}

uint32_t VRBRAINStorage::read_dword(uint16_t loc){
    return 0;
}

void VRBRAINStorage::read_block(void* dst, uint16_t src, size_t n) {
    memset(dst, 0, n);
}

void VRBRAINStorage::write_byte(uint16_t loc, uint8_t value)
{}

void VRBRAINStorage::write_word(uint16_t loc, uint16_t value)
{}

void VRBRAINStorage::write_dword(uint16_t loc, uint32_t value)
{}

void VRBRAINStorage::write_block(uint16_t loc, void* src, size_t n)
{}

