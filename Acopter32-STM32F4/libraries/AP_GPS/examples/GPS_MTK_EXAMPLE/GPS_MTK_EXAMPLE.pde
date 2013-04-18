/*
	Example of GPS MTK library.
	Code by Jordi Munoz and Jose Julio. DIYDrones.com

	Works with Ardupilot Mega Hardware (GPS on Serial Port1)
*/

//#include <FastSerial.h>
#include <AP_GPS_MTK16.h>
#include <stdio.h>

//FastSerialPort0(Serial);
//FastSerialPort1(Serial1);

AP_GPS_MTK16 gps;
#define T6 1000000
#define T7 10000000
char tmp[200];
void setup()
{
	Serial4.begin(38400);
	Serial1.begin(38400);
	stderr = stdout;
	gps.print_errors = true;

	Serial1.println("GPS MTK library test");
	gps.init();	 // GPS Initialization
	delay(1000);
}
void loop()
{
	delay(20);
	gps.update();
	if (gps.new_data){
		sprintf(tmp,"Lat %f , Lon %f , Alt %f , GSP %f , COG %f , SAT %d , FIX %d , TIME %d\n\r",(float)gps.latitude/7,(float)gps.longitude / T7,(float)gps.altitude / 100.0,gps.ground_speed / 100.0,gps.ground_course / 100.0,gps.num_sats,gps.fix,gps.time);
		Serial1.print(tmp);
		gps.new_data = 0; // We have readed the data
		}
}

