#include "HardwareI2C.h"

#define I2CDELAY 50
//#define DELAYI2C

HardwareI2C::HardwareI2C()
{
}

HardwareI2C::HardwareI2C(uint32_t i2c_num)
{
    switch (i2c_num) {
    case 1:
        this->i2c_d = _I2C1;
        this->begin();
        break;
    case 2:
        this->i2c_d = _I2C2;
        this->begin();
        break;
    default:
        assert_param(0);
        break;
    }
}

////////////// Public Methods ////////////////////////////////////////

void HardwareI2C::begin()
{
	/* set as master */
	i2c_init(this->i2c_d, 0, I2C_400KHz_SPEED);
	delay(I2CDELAY);
}

void HardwareI2C::end()
{
}

void HardwareI2C::setSpeed(boolean _fast)
{

}

/* WRITE ******************************************/
int8_t HardwareI2C::write(uint8_t address, uint8_t len, uint8_t *tx_buffer)
{
	uint8_t numbytes = len;

	uint32_t ret = i2c_write(this->i2c_d, address, tx_buffer, &numbytes);
	//sEE_WaitEepromStandby(this->i2c_d, address);

	return ret;
}

int8_t HardwareI2C::write(uint8_t address, uint8_t registerAddress, uint8_t databyte)
{
	uint8_t ibuff[2];

	ibuff[0] = registerAddress;
	ibuff[1] = databyte;
	uint8_t numbytes = 2;

	uint8_t ret = i2c_write(this->i2c_d, address, ibuff, &numbytes);
	//sEE_WaitEepromStandby(this->i2c_d, address);

	return ret;
}


/* READ *******************************************/

int8_t HardwareI2C::read(uint8_t address, uint8_t registerAddress, uint8_t numberBytes, uint8_t *dataBuffer)
{
	uint8_t ibuff[1];
	ibuff[0] = registerAddress;
	uint8_t ret = i2c_read(this->i2c_d, address, ibuff, 1, dataBuffer, &numberBytes);
	while(numberBytes > 0);

	return ret;
}


uint8_t HardwareI2C::WaitEepromStandbyState(uint8_t address){
    uint32_t ret = sEE_WaitEepromStandbyState();
    return (uint8_t)ret;
}
