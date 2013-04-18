/*
	Example of APM_RC library.
	Code by Jordi MuÒoz and Jose Julio , Porting on Multipilot32 by Roberto Navoni . DIYDrones.com Virtualrobotix.com

	Print Input values and send Output to the servos
	(Works with last PPM_encoder firmware)
*/

#include <APM_RC.h> // ArduPilot Mega RC Library

void setup()
{
	Serial3.begin(115200);
        Serial3.println("ArduPilot 32 RC library test");	
        APM_RC.Init();	 // APM Radio initialization
	delay(1000);
}

void loop()
{
	// New radio frame? (we could use also if((millis()- timer) > 20)
	if (APM_RC.GetState() == 1){
		Serial3.print("CH:");
		for(int i = 0; i < 8; i++){
			Serial3.print(APM_RC.InputCh(i));	// Print channel values
			Serial3.print(",");
			APM_RC.OutputCh(i, APM_RC.InputCh(i)); // Copy input to Servos
		}
		Serial3.println();
	}
}
