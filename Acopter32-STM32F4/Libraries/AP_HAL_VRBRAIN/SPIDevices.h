
#ifndef __AP_HAL_VRBRAIN_SPI_DEVICES_H__
#define __AP_HAL_VRBRAIN_SPI_DEVICES_H__

#include <AP_HAL.h>
#include "AP_HAL_VRBRAIN_Namespace.h"
#include <spi.h>
#include <laserlab_MP32V1F4.h>

typedef enum SPIFrequency {
    SPI_18MHZ       = 0, /**< 18 MHz */
    SPI_9MHZ        = 1, /**< 9 MHz */
    SPI_4_5MHZ      = 2, /**< 4.5 MHz */
    SPI_2_25MHZ     = 3, /**< 2.25 MHz */
    SPI_1_125MHZ    = 4, /**< 1.125 MHz */
    SPI_562_500KHZ  = 5, /**< 562.500 KHz */
    SPI_281_250KHZ  = 6, /**< 281.250 KHz */
    SPI_140_625KHZ  = 7, /**< 140.625 KHz */
} SPIFrequency;

static const spi_baud_rate baud_rates[8] __FLASH__ = {
    SPI_BAUD_PCLK_DIV_2,
    SPI_BAUD_PCLK_DIV_4,
    SPI_BAUD_PCLK_DIV_8,
    SPI_BAUD_PCLK_DIV_16,
    SPI_BAUD_PCLK_DIV_32,
    SPI_BAUD_PCLK_DIV_64,
    SPI_BAUD_PCLK_DIV_128,
    SPI_BAUD_PCLK_DIV_256,
};

struct spi_pins {
    uint8_t nss;
    uint8_t sck;
    uint8_t miso;
    uint8_t mosi;
};

static const spi_pins board_spi_pins[] __FLASH__ = {
    {BOARD_SPI1_NSS_PIN,
     BOARD_SPI1_SCK_PIN,
     BOARD_SPI1_MISO_PIN,
     BOARD_SPI1_MOSI_PIN},
    {BOARD_SPI2_NSS_PIN,
     BOARD_SPI2_SCK_PIN,
     BOARD_SPI2_MISO_PIN,
     BOARD_SPI2_MOSI_PIN},
#ifdef STM32_HIGH_DENSITY
    {BOARD_SPI3_NSS_PIN,
     BOARD_SPI3_SCK_PIN,
     BOARD_SPI3_MISO_PIN,
     BOARD_SPI3_MOSI_PIN},
#endif
};

class VRBRAIN::VRBRAINSPI1DeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    VRBRAINSPI1DeviceDriver()
    :
        _dev(_SPI2)
    {}

    void init();
    AP_HAL::Semaphore* get_semaphore();

    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);
    void transfer(const uint8_t *data, uint16_t len);

private:
    void _cs_assert();
    void _cs_release();
    uint8_t _transfer(uint8_t data);
    const spi_pins* dev_to_spi_pins(spi_dev *dev);
    void configure_gpios(spi_dev *dev, bool as_master);
    const spi_baud_rate determine_baud_rate(SPIFrequency freq);

    static VRBRAIN::VRBRAINSemaphore _semaphore;

    //uint8_t _device_num;
    spi_dev *_dev;

};


class VRBRAIN::VRBRAINSPI2DeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    VRBRAINSPI2DeviceDriver()
    :
        _dev(_SPI2)
    {}

    void init();
    AP_HAL::Semaphore* get_semaphore();

    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);
    void transfer(const uint8_t *data, uint16_t len);


private:
    void _cs_assert();
    void _cs_release();
    uint8_t _transfer(uint8_t data);
    const spi_pins* dev_to_spi_pins(spi_dev *dev);
    void configure_gpios(spi_dev *dev, bool as_master);
    const spi_baud_rate determine_baud_rate(SPIFrequency freq);

    static VRBRAIN::VRBRAINSemaphore _semaphore;

    //uint8_t _device_num;
    spi_dev *_dev;
};


#endif // __AP_HAL_VRBRAIN_SPI_DEVICES_H__
