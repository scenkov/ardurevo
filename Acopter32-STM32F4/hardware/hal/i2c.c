#include "i2c.h"
#include "gpiopins.h"
#include "systick.h"
#include "stm32f4xx_i2c.h"

#define TIMEOUT 500

static i2c_dev i2c_dev1 = {
    .I2Cx         = I2C1,
    .gpio_port    = &gpiob,
    .sda_pin      = 9,
    .scl_pin      = 8,
    .clk       	  = RCC_APB1Periph_I2C1,
    .gpio_af	  = GPIO_AF_I2C1,
    .ev_nvic_line = I2C1_EV_IRQn,
    .er_nvic_line = I2C1_ER_IRQn    
};
/** I2C1 device */
i2c_dev* const _I2C1 = &i2c_dev1;

static i2c_dev i2c_dev2 = {
    .I2Cx         = I2C2,
    .gpio_port    = &gpiob,
    .sda_pin      = 11,
    .scl_pin      = 10,
    .clk       	  = RCC_APB1Periph_I2C2,
    .gpio_af	  = GPIO_AF_I2C2,
    .ev_nvic_line = I2C2_EV_IRQn,
    .er_nvic_line = I2C2_ER_IRQn        
};
/** I2C2 device */
i2c_dev* const _I2C2 = &i2c_dev2;


typedef enum {TX = 0, RX = 1, TXREG = 2} I2C_Dir;

__IO I2C_Dir I2C_DIR;
__IO uint8_t I2C_BLOCKED = 0;
__IO uint8_t I2CADDRESS = 0;
__IO uint8_t rx_buffer_len = 0;
__IO uint8_t tx_buffer_len = 0;
__IO uint8_t tx_buffer_ix = 0;
__IO uint8_t rx_buffer_ix = 0;
__IO uint8_t i2c_reg_len = 0;
#define I2C_BUF_SIZE 16
__IO uint8_t tx_buffer[I2C_BUF_SIZE];
__IO uint8_t *rx_buffer_ptr;

__IO uint32_t  sEETimeout = I2C_LONG_TIMEOUT;
__IO uint16_t* sEEDataReadPointer;
__IO uint8_t*  sEEDataWritePointer;
__IO uint8_t   sEEDataNum;

/**
  * @brief  DeInitializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
static void i2c_lowLevel_deinit(i2c_dev *dev)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
   
  /* sEE_I2C Peripheral Disable */
  I2C_Cmd(dev->I2Cx, DISABLE);
 
  /* sEE_I2C DeInit */
  I2C_DeInit(dev->I2Cx);

  /*!< sEE_I2C Periph clock disable */
  RCC_APB1PeriphClockCmd(dev->clk, DISABLE);
    
  /*!< GPIO configuration */  
  /*!< Configure I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = BIT(dev->scl_pin);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(dev->gpio_port->GPIOx, &GPIO_InitStructure);

  /*!< Configure I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = BIT(dev->sda_pin);
  GPIO_Init(dev->gpio_port->GPIOx, &GPIO_InitStructure);
}

/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
static void i2c_lowLevel_init(i2c_dev *dev)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
   
  /* Enable the i2c */
  RCC_APB1PeriphClockCmd(dev->clk, ENABLE);
  
  /* Reset the Peripheral */
  RCC_APB1PeriphResetCmd(dev->clk, ENABLE);
  RCC_APB1PeriphResetCmd(dev->clk, DISABLE);
  
  /* Enable the GPIOs for the SCL/SDA Pins */
  RCC_AHB1PeriphClockCmd(dev->gpio_port->clk, ENABLE);
     
  /* GPIO configuration */  
  /* Configure SCL */   
  GPIO_InitStructure.GPIO_Pin = BIT(dev->scl_pin);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(dev->gpio_port->GPIOx, &GPIO_InitStructure);

  /* Configure SDA */
  GPIO_InitStructure.GPIO_Pin = BIT(dev->sda_pin);
  GPIO_Init(dev->gpio_port->GPIOx, &GPIO_InitStructure);

  /* Connect GPIO pins to peripheral */
  GPIO_PinAFConfig(dev->gpio_port->GPIOx, dev->scl_pin, dev->gpio_af);
  GPIO_PinAFConfig(dev->gpio_port->GPIOx, dev->sda_pin, dev->gpio_af);      
}

void i2c_init(i2c_dev *dev, uint16_t address, uint32_t speed)
{ 
  I2C_InitTypeDef  I2C_InitStructure;
  
  i2c_lowLevel_init(dev);
  
  /* I2C configuration */
  I2C_StructInit(&I2C_InitStructure);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = address;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = speed;
  /* Apply I2C configuration after enabling it */
  I2C_Init(dev->I2Cx, &I2C_InitStructure);    

  NVIC_InitTypeDef        NVIC_InitStructure;
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
	  
  I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
      
  /* I2C Peripheral Enable */
  I2C_Cmd(dev->I2Cx, ENABLE);
}

/**
  * @brief  DeInitializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void i2c_deinit(i2c_dev *dev)
{
  i2c_lowLevel_deinit(dev); 
}

/*
void i2c_set_speed(i2c_dev *dev, uint32_t speed)
{
	I2C_InitTypeDef  I2C_InitStructure;
	I2C_InitStructure.I2C_ClockSpeed = speed;  
	I2C_Init(dev->I2Cx, &I2C_InitStructure);   
}
*/






void I2C_Serve(I2C_TypeDef *I2Cx)
{
        /*
         * state machine for the whole I2C events
         */
        switch (I2C_GetLastEvent(I2Cx))
        {
                /*
                 * Master Mode
                 */
                case I2C_EVENT_MASTER_MODE_SELECT:
                        /*
                         * we want to transmit a single byte
                         */
                        if(I2C_DIR == TX)
                        {
                                I2C_Send7bitAddress(I2Cx, I2CADDRESS, I2C_Direction_Transmitter);
                        /*
                         * we want to read a register value
                         * first step is TRANSMITTING the register address then RECEIVE the data
                         */
                        }else if(I2C_DIR == TXREG)
                        {
                                I2C_Send7bitAddress(I2Cx, I2CADDRESS, I2C_Direction_Transmitter);
                        /*
                         * we want to receive something
                         */
                        }else if(I2C_DIR == RX)
                        {
                                I2C_Send7bitAddress(I2Cx, I2CADDRESS, I2C_Direction_Receiver);
                        }
                        // address sent, if no byte should be sent: request NAK now!
						if( tx_buffer_len == 0 ) {
							// request NAK
							I2C_AcknowledgeConfig(I2Cx, DISABLE);
							// request stop condition
							I2C_GenerateSTOP(I2Cx, ENABLE);
						}                        
                break;

                        /*
                         * we get here after transmitting address + write bit
                         */
                case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
                    break;
                case I2C_EVENT_MASTER_BYTE_TRANSMITTING:  
						if( tx_buffer_ix < tx_buffer_len ) {
							I2C_SendData(I2Cx, tx_buffer[tx_buffer_ix++]);
						}  
						else
						{						
							if(I2C_DIR == TX)
							{
								// send stop condition
								I2C_GenerateSTOP(I2Cx, ENABLE);
								I2C_BLOCKED = 0;
							}
							else if(I2C_DIR == TXREG)
							{
								/*
								* Generate second start and switch to receiver mode
								*/
                                I2C_DIR = RX;
                                I2C_GenerateSTART(I2Cx,ENABLE);
							}
                        }
				break;
				  
                case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
                break;

                case I2C_EVENT_MASTER_MODE_ADDRESS10:
                break;

               /*
                * we received a byte...
                */
                case I2C_EVENT_MASTER_BYTE_RECEIVED:
						/* Store I2C received data */
						rx_buffer_ptr[rx_buffer_ix++] = I2C_ReceiveData(I2Cx);
						/* Request NACK and send I2C1 STOP condition before receiving the last data */
						if (rx_buffer_ix == rx_buffer_len - 1)
						{
							/* Request ACK and send I2C STOP condition before receiving the last data */
							/* Request NACK */
							I2C_AcknowledgeConfig(I2Cx, DISABLE);
							/* Send I2C STOP Condition */
							I2C_GenerateSTOP(I2Cx, ENABLE);						
						}
						if (rx_buffer_ix == rx_buffer_len)
						{
							I2C_BLOCKED = 0;
						}
                break;
                
                /*
                 * we switched the mode
                 */
                case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
						if(rx_buffer_len == 1)
						{
							/* Request NACK */
							I2C_AcknowledgeConfig(I2Cx, DISABLE);
							/* Send I2C STOP Condition */
							I2C_GenerateSTOP(I2Cx,ENABLE);
 						}                        
                break;

        }
}

void I2C1_EV_IRQHandler()
{
	I2C_Serve(I2C1);
}

void I2C2_EV_IRQHandler()
{
	I2C_Serve(I2C2);
}

/*
 * dummey irq handler
 */
void I2C2_ER_IRQHandler(void)
{
  if (I2C_GetITStatus(I2C2, I2C_IT_AF))
  {
    I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
  }
}

/*
 * dummey irq handler
 */
void I2C1_ER_IRQHandler(void)
{
  if (I2C_GetITStatus(I2C1, I2C_IT_AF))
  {
    I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
  }
}



uint8_t i2c_write(i2c_dev *dev, uint8_t addr, uint8_t *buffer, uint8_t len)
{
	assert_param(len <= I2C_BUF_SIZE && len > 0);
	
	// disable interrupts
	I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
	
    /*
     * check if I2C interace is in use
     */
	if(I2C_BLOCKED == 0)
    {
        I2CADDRESS = addr;
        I2C_DIR = TX;
		// set buffer length and start index
		tx_buffer_len = len;
		tx_buffer_ix = 0;
		rx_buffer_len = 0;
		rx_buffer_ix = 0;
		
		__IO uint8_t *tmp_buffer_ptr = tx_buffer;
		uint8_t i;
		for(i=0; i<len; ++i) {
			*tmp_buffer_ptr++ = *buffer++; // copies faster than using indexed arrays
		}
                    
		// start with ACK
		I2C_AcknowledgeConfig(dev->I2Cx, ENABLE);

		// enable I2V2 event, buffer and error interrupt
		I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);

		// notify that transfer has started
		I2C_BLOCKED = 1; 

		// send start condition
		I2C_GenerateSTART(dev->I2Cx, ENABLE);
	} 
	else
	{
		//errno_r = EBUSY;
		return I2C_ERROR;
	}
	
    /*
     * wait till finished
     */
    //while(I2C_BLOCKED == 1);
    
	uint32_t startime =  systick_uptime();
    while(I2C_BLOCKED == 1)
    {
       if ((systick_uptime() - startime) > TIMEOUT)
          break;
    }
    return I2C_OK;
}

uint8_t i2c_read(i2c_dev *dev, uint8_t addr, uint8_t *tx_buf, uint8_t txlen, uint8_t *rx_buffer, uint8_t rxlen)
{
	assert_param(len <= I2C_BUF_SIZE && rxlen > 0);
	assert_param(txlen <= 2);
	// disable interrupts
	I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
		
    /*
     * check if I2C interface is in use
     */
    if(I2C_BLOCKED == 0)
    {
        I2CADDRESS = addr;
        I2C_DIR = TXREG;
        rx_buffer_len = rxlen;
        rx_buffer_ix = 0;        
   		rx_buffer_ptr = rx_buffer;
		tx_buffer_ix = 0;
		tx_buffer_len = txlen;

		__IO uint8_t *tmp_buffer_ptr = tx_buffer;
		uint8_t i;
		for(i=0; i<txlen; ++i) {
			*tmp_buffer_ptr++ = *tx_buf++; // copies faster than using indexed arrays
		}
		        
		// start with ACK
		I2C_AcknowledgeConfig(dev->I2Cx, ENABLE);

		// enable I2V2 event, buffer and error interrupt
		I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);

		// notify that transfer has started
		I2C_BLOCKED = 1; 

		// send start condition
		I2C_GenerateSTART(dev->I2Cx, ENABLE);
    }
	else 
	{
		//errno_r = EBUSY;
		return I2C_ERROR;
	}

    /*
     * wait till finished
    */
	uint32_t startime =  systick_uptime();

    while(I2C_BLOCKED == 1)
    {
       if ((systick_uptime() - startime) > TIMEOUT)
          break;
    }
  
    return I2C_OK;
}

uint8_t i2c_is_busy()
{
        return I2C_BLOCKED;
}


/**
  * @brief  Wait for EEPROM Standby state.
  *
  * @note  This function allows to wait and check that EEPROM has finished the
  *        last operation. It is mostly used after Write operation: after receiving
  *        the buffer to be written, the EEPROM may need additional time to actually
  *        perform the write operation. During this time, it doesn't answer to
  *        I2C packets addressed to it. Once the write operation is complete
  *        the EEPROM responds to its address.
  *
  * @param  None
  * @retval sEE_OK (0) if operation is correctly performed, else return value
  *         different from sEE_OK (0) or the timeout user callback.
  */
uint32_t sEE_WaitEepromStandbyState(i2c_dev *dev, uint8_t addr)
{
  __IO uint16_t tmpSR1 = 0;
  __IO uint32_t sEETrials = 0;

  I2CADDRESS = addr;

  /*!< While the bus is busy */
  sEETimeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(dev->I2Cx, I2C_FLAG_BUSY))
  {
    if((sEETimeout--) == 0) return I2C_ERROR;
  }

  /* Keep looping till the slave acknowledge his address or maximum number
     of trials is reached (this number is defined by sEE_MAX_TRIALS_NUMBER define
     in STM324x7I_eval_i2c_ee.h file) */
  while (1)
  {
    //Disable interrupts
      I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);

    /*!< Send START condition */
    I2C_GenerateSTART(dev->I2Cx, ENABLE);

    /*!< Test on EV5 and clear it */
    sEETimeout = I2C_TIMEOUT;
    while(!I2C_CheckEvent(dev->I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
    {

    }
      if((sEETimeout--) == 0)
	  {
	  //I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
	  return I2C_ERROR;
	  }

    /*!< Send EEPROM address for write */
    I2C_Send7bitAddress(dev->I2Cx, I2CADDRESS, I2C_Direction_Transmitter);

    /* Wait for ADDR flag to be set (Slave acknowledged his address) */
    sEETimeout = I2C_LONG_TIMEOUT;
    do
    {
      /* Get the current value of the SR1 register */
      tmpSR1 = dev->I2Cx->SR1;

      /* Update the timeout value and exit if it reach 0 */
      if((sEETimeout--) == 0)
	  {
	  //I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
	  return I2C_ERROR;
	  }
    }
    /* Keep looping till the Address is acknowledged or the AF flag is
       set (address not acknowledged at time) */
    while((tmpSR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) == 0);

    /* Check if the ADDR flag has been set */
    if (tmpSR1 & I2C_SR1_ADDR)
    {
      /* Clear ADDR Flag by reading SR1 then SR2 registers (SR1 have already
         been read) */
      (void)dev->I2Cx->SR2;

      /*!< STOP condition */
      I2C_GenerateSTOP(dev->I2Cx, ENABLE);

      //I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
      /* Exit the function */
      return I2C_OK;
    }
    else
    {
      /*!< Clear AF flag */
      I2C_ClearFlag(dev->I2Cx, I2C_FLAG_AF);
    }

    /* Check if the maximum allowed number of trials has bee reached */
    if (sEETrials++ == sEE_MAX_TRIALS_NUMBER)
    {
	//I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
      /* If the maximum number of trials has been reached, exit the function */
      return I2C_ERROR;
    }
  }
}

