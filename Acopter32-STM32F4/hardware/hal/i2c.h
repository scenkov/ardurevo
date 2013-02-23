#ifndef _I2C_H
#define _I2C_H

#include <stm32f4xx.h>
#include <hal.h>

#define I2C_100KHz_SPEED                        100000
#define I2C_400KHz_SPEED                        400000

/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will 
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   

#define I2C_TIMEOUT         ((uint32_t)0x1000)
#define I2C_LONG_TIMEOUT    ((uint32_t)(10 * I2C_TIMEOUT))


/* Maximum number of trials for sEE_WaitEepromStandbyState() function */
#define sEE_MAX_TRIALS_NUMBER     300

/**
 * @brief I2C device type.
 */
typedef struct i2c_dev {
    I2C_TypeDef* I2Cx;          
    gpio_dev *gpio_port;        
    uint8_t sda_pin;             
    uint8_t scl_pin;             
    uint32_t clk;          
    uint8_t gpio_af;     
    IRQn_Type ev_nvic_line;  /* Event IRQ number */
    IRQn_Type er_nvic_line;  /* Error IRQ number */        
} i2c_dev;

#ifdef __cplusplus
  extern "C" {
#endif
 

void i2c_init(i2c_dev *dev, uint16_t address, uint32_t speed);
void i2c_deinit(i2c_dev *dev);
/*
uint8_t i2c_Write(i2c_dev *dev, uint16_t dev_addr, uint8_t len, uint8_t *data);
uint8_t i2c_write(i2c_dev *dev, uint16_t eerpm_addr, uint16_t addr, uint8_t data);
uint8_t i2c_8bitaddr_write(i2c_dev *dev, uint16_t dev_addr, uint8_t addr, uint8_t data);
uint8_t i2c_read(i2c_dev *dev, uint16_t eerpm_addr, uint16_t addr, uint8_t *data);
uint8_t i2c_buffer_read(i2c_dev *dev, uint16_t eerpm_addr, uint16_t addr, uint8_t len, uint8_t *buf);
uint8_t i2c_8bitaddr_buffer_read(i2c_dev *dev, uint16_t dev_addr, uint8_t addr, uint8_t len, uint8_t *buf);


uint8_t i2c_send(i2c_dev *dev, uint8_t data);
uint8_t i2c_readack(i2c_dev *dev, uint8_t *data);
uint8_t i2c_stop(i2c_dev *dev);
uint8_t i2c_start(i2c_dev *dev, uint8_t Address);
uint8_t i2c_start_wait(i2c_dev *dev, uint8_t Address);
*/

uint32_t i2c_write(i2c_dev *dev, uint8_t addr, uint8_t *tx_buffer, uint8_t len);
uint32_t i2c_read(i2c_dev *dev, uint8_t addr, uint8_t *tx_buffer, uint8_t txlen, uint8_t *rx_buffer, uint8_t rxlen);
uint8_t i2c_is_busy();
uint32_t sEE_WaitEepromStandbyState(i2c_dev *dev, uint8_t addr);

extern i2c_dev* const _I2C1;
extern i2c_dev* const _I2C2;

#ifdef __cplusplus
  }
#endif
 

#endif
