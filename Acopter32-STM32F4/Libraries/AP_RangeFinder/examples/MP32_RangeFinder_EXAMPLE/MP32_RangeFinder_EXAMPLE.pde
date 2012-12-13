/*
  AP_RangeFinder_test
  Code by DIYDrones.com
*/
#define SERIAL0_BAUD			115200

#include <FastSerial.h>
#include "wirish.h"
#include <AP_ADC.h>
#include <ModeFilter.h>
#include <AP_RangeFinder.h>        // Range finder library
	// ArduPilot Mega Analog to Digital Converter Library



#define RF_PIN 7 // the far back-right pin on the oilpan (near the CLI switch)
//#define RF_PIN A5 // A5 is the far back-right pin on the oilpan (near the CLI switch)

FastSerialPort2(Serial);  

// declare global instances for reading pitot tube
AP_ADC_ADS7844	adc;

ModeFilter sonar_mode_filter;
// create the range finder object
//AP_RangeFinder_SharpGP2Y aRF;
AP_RangeFinder_MaxsonarXL aRF(&adc, &sonar_mode_filter);
//AP_RangeFinder_MaxsonarLV aRF;

void setup()
{
  Serial.begin(115200);
  Serial.println("Range Finder Test v1.0");
  adc.Init();            // APM ADC library initialization
  //aRF.init(RF_PIN, &adc);
}

void loop()
{   
    //int i = 0;
    adc.Update();
    Serial.print("dist:");
    Serial.print(aRF.read());
    Serial.print("\traw:");
    Serial.print(aRF.raw_value); 
    Serial.println();
    delay(20); 
}

