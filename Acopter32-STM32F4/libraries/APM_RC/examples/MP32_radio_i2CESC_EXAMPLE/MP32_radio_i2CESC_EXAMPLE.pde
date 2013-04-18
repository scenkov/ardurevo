/*
	Example of APM_RC library.
	Code by Jordi MuÃ’oz and Jose Julio , Porting on Multipilot32 by Roberto Navoni . DIYDrones.com Virtualrobotix.com

	Print Input values and send Output to the servos
	(Works with last PPM_encoder firmware)
*/
#define ESCI2C
#define AUTOTEST

#include "APM_RC.h"

void setup()
{
	Serial3.begin(115200);
        Serial3.println("ArduPilot 32 RC library test");	
        delay(1000);

        APM_RC.Init();	 // APM Radio initialization
        delay(1000);

#ifdef ESCI2C
        APM_RC.InitI2C(); // APM I2C Init
#endif
        delay(5000);
}

void loop()
{
	// New radio frame? (we could use also if((millis()- timer) > 20)
#ifdef AUTOTEST
	for(int out=1000;out<1200;out++)
{
//                Serial3.print("CH:");
		for(int i = 0; i < 4; i++){          
//			Serial3.print(i);	// Print channel values
//			Serial3.print(":");
  //                      Serial3.print(out);
			//APM_RC.OutputCh(i, out); // Copy input to Servos
                        #ifdef ESCI2C
                        APM_RC.OutputChI2C(i, out);
                        APM_RC.Force_OutI2C(i);
                        //APM_RC.Force_OutI2C();

                       // APM_RC.OutputChI2C(i, 0);
                       // APM_RC.Force_OutI2C(i);
                       // delay(1000);
                        #endif
		}
                //delay(1);                
                #ifdef ESCI2C       
                //APM_RC.Force_OutI2C(255);
                //delay(100);
                #endif
//		Serial3.println();
}

#else

	if (APM_RC.GetState() == 1){
		Serial3.print("CH:");
		for(int i = 0; i < 4; i++){
			Serial3.print(APM_RC.InputCh(i));	// Print channel values
			Serial3.print(",");
			APM_RC.OutputCh(i, APM_RC.InputCh(i)); // Copy input to Servos
                        #ifdef ESCI2C
                        APM_RC.OutputChI2C(i, APM_RC.InputCh(i));
                        #endif
		}
                
                #ifdef ESCI2C       
                APM_RC.Force_OutI2C();
                #endif
		Serial3.println();
	}
#endif
}

