/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>


#include <AP_HAL.h>
#include "SPIDevices.h"
#include "GPIO.h"
#include "Semaphores.h"
#include <spi.h>

using namespace VRBRAIN;

extern const AP_HAL::HAL& hal;

VRBRAINSemaphore VRBRAINSPI2DeviceDriver::_semaphore;

void VRBRAINSPI2DeviceDriver::init() {
    //_dev = _SPI2;

    SPIFrequency freq = SPI_1_125MHZ;
    spi_baud_rate baud = determine_baud_rate(freq);

    bool as_master = true;

    configure_gpios(_dev, as_master);
    if (as_master) {
        spi_master_enable(_dev, baud, (spi_mode)0, MSBFIRST);
    } else {
        spi_slave_enable(_dev, (spi_mode)0, MSBFIRST);
    }

    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);

}

AP_HAL::Semaphore* VRBRAINSPI2DeviceDriver::get_semaphore() {
    return &_semaphore;
}

inline void VRBRAINSPI2DeviceDriver::_cs_assert() {
    digitalWrite(_cs_pin, LOW);
}

inline void VRBRAINSPI2DeviceDriver::_cs_release() {
    digitalWrite(_cs_pin, HIGH);
}

inline uint8_t VRBRAINSPI2DeviceDriver::_transfer(uint8_t data) {
    uint8_t buf[1];

    //write 1byte
    spi_tx(this->_dev, data, 1);

    //read one byte
    while (!spi_is_rx_nonempty(this->_dev))
            ;
    buf[0] = (uint8_t)spi_rx_reg(this->_dev);
    return buf[0];
}

void VRBRAINSPI2DeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len) {

    _cs_assert();
    if (rx == NULL) {
        for (uint16_t i = 0; i < len; i++) {
            _transfer(tx[i]);
        }
    } else {
        for (uint16_t i = 0; i < len; i++) {
            rx[i] = _transfer(tx[i]);
        }
    }
    _cs_release();
}

uint8_t VRBRAINSPI2DeviceDriver::transfer(uint8_t data) {
    return _transfer(data);
}

void VRBRAINSPI2DeviceDriver::transfer(const uint8_t *tx, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
            _transfer(tx[i]);
    }
}
void VRBRAINSPI2DeviceDriver::cs_assert() {
    _cs_assert();
}

void VRBRAINSPI2DeviceDriver::cs_release() {
    _cs_release();
}



const spi_pins* VRBRAINSPI2DeviceDriver::dev_to_spi_pins(spi_dev *dev) {
    if (_dev->SPIx == SPI1)
       return board_spi_pins;
    else if (dev->SPIx == SPI2)
       return board_spi_pins + 1;
#ifdef STM32_HIGH_DENSITY
    else if (_dev->SPIx == SPI3)
	  return board_spi_pins + 2;
#endif
	else
	{
	  assert_param(0);
	  return NULL;
	}
}

void VRBRAINSPI2DeviceDriver::configure_gpios(spi_dev *dev, bool as_master) {
    const spi_pins *pins = dev_to_spi_pins(dev);

    if (!pins) {
        return;
    }

    const stm32_pin_info *nssi = &PIN_MAP[pins->nss];
    const stm32_pin_info *scki = &PIN_MAP[pins->sck];
    const stm32_pin_info *misoi = &PIN_MAP[pins->miso];
    const stm32_pin_info *mosii = &PIN_MAP[pins->mosi];

    spi_gpio_cfg(dev,
		as_master,
                nssi->gpio_device,
                nssi->gpio_bit,
                scki->gpio_device,
                scki->gpio_bit,
                misoi->gpio_bit,
                mosii->gpio_bit);
}

const spi_baud_rate VRBRAINSPI2DeviceDriver::determine_baud_rate(SPIFrequency freq)
{

	spi_baud_rate rate;

	switch(freq)
	{
		case SPI_18MHZ:
			rate = SPI_BAUD_PCLK_DIV_2;
			break;
		case SPI_9MHZ:
			rate = SPI_BAUD_PCLK_DIV_4;
			break;
		case SPI_4_5MHZ:
			rate = SPI_BAUD_PCLK_DIV_8;
			break;
		case SPI_2_25MHZ:
			rate = SPI_BAUD_PCLK_DIV_16;
			break;
		case SPI_1_125MHZ:
			rate = SPI_BAUD_PCLK_DIV_32;
			break;
		case SPI_562_500KHZ:
			rate = SPI_BAUD_PCLK_DIV_64;
			break;
		case SPI_281_250KHZ:
			rate = SPI_BAUD_PCLK_DIV_128;
			break;
		case SPI_140_625KHZ:
			rate = SPI_BAUD_PCLK_DIV_256;
			break;
		default:
			rate = SPI_BAUD_PCLK_DIV_32;
			break;

	}
	return rate;
}
