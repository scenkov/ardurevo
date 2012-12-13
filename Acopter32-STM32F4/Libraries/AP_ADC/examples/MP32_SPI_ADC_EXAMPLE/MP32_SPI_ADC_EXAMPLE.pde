/*
  Example of APM_ADC library.
  Code by Jordi Muï¿½oz and Jose Julio Port to Multipilot32 by Roberto Navoni. DIYDrones.com virtualrobotix.com
*/
# define SERIAL0_BAUD			115200
#include <FastSerial.h>
#include <AP_ADC.h> // ArduPilot Mega ADC Library

unsigned long timer;

FastSerialPort2(Serial);        // FTDI/console

//#define USART3 2
//HardwareSerial Serial1(USART3, 2250000UL, GPIOD_BASE, 8,9, TIMER_INVALID, 0);



AP_ADC_ADS7844 adc;

void setup()
{  

  adc.Init();   // APM ADC initialization
  Serial.begin(SERIAL0_BAUD, 128, 128);

  //Serial1.begin(115200);
  Serial.println("ArduPilot Arm ADC library test");
  delay(1000);
  timer = millis();
}

void loop()
{
  int ch;
  
  if((millis()- timer) > 20)
    {
    adc.Update(); // Call the spi update function.
    timer = millis();
    for (ch=0;ch<8;ch++)
      {
      Serial.print(adc.Ch(ch));
      Serial.print(",");
      }
    Serial.println();
    }
}

