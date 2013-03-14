/*
  Example of APM_ADC library.
  Code by Jordi Muï¿½oz and Jose Julio. DIYDrones.com
*/

#include <AP_ADC.h> // ArduPilot Mega ADC Library

unsigned long timer;

//AP_ADC_ADS7844 adc;
AP_ADC adc;

void setup()
{  
  adc.Init();   // APM ADC initialization
  Serial1.begin(115200);
  Serial1.println("ArduPilot Arm ADC library test");
  delay(1000);
  timer = millis();
}

void loop()
{
  int ch;
  
  if((millis()- timer) > 20)
    {
    timer = millis();
    for (ch=0;ch<7;ch++)
      {
      Serial1.print(adc.Ch(ch));
      Serial1.print(",");
      }
    Serial1.println();
    }
}

