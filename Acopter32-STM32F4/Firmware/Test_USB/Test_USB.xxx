// Libraries
#include <FastSerial.h>
#include "WProgram.h"

FastSerialPort0(Serial);        // FTDI/console
FastSerial SerialUSB;

void setup() 
{
 	Serial.begin(115200);
   	SerialUSB.configure(99);
    SerialUSB.begin(115200);
   	
   	Serial.println("Seriale OK");
   	
    }

uint8_t txbuf[] = { 't', 'e', 's','t','\n'};
int ret;
uint8_t ch;

void loop()
{  	
	/*
	if (usb_data_available())
	{
		ch = usb_getc();
		Serial.print(ch);
		usb_putc(ch);
	}
	*/
	Serial.printf("ciao\n");
	SerialUSB.printf("Fast Serial USB : Ciao da USB\n");
	delay(1000);
	
}

