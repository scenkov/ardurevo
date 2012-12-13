#ifndef APM_RC_MP32_h
#define APM_RC_MP32_h

#include <AP_Common.h>
#include "../AP_Math/AP_Math.h"
#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"
#include <inttypes.h>


#define NUM_CHANNELS 8
#define MIN_PULSEWIDTH 900
#define MAX_PULSEWIDTH 2100


#define MOTORID1 0		
#define MOTORID2 1		
#define MOTORID3 2		
#define MOTORID4 3		
#define MOTORID5 4		
#define MOTORID6 5		
#define MOTORID7 6		
#define MOTORID8 7
#define MOTORID9 8		
#define MOTORID10 9		
#define MOTORID11 10		
#define MOTORID12 11
		
#define PPMSUM_IN 60 // IN CHANNEL 8
//#define PPMSUMACTIVE

// MP32V1F1
//#define PPM_IN_CH1 22
//#define PPM_IN_CH2 23  // PA8
//#define PPM_IN_CH3 24  //
//#define PPM_IN_CH4 89
//#define PPM_IN_CH5 59
//#define PPM_IN_CH6 62
//#define PPM_IN_CH7 0
//#define PPM_IN_CH8 60


// MP32F4
#define PPM_IN_CH1 22
#define PPM_IN_CH2 63  // PA8
#define PPM_IN_CH3 66  //
#define PPM_IN_CH4 89
#define PPM_IN_CH5 59
#define PPM_IN_CH6 62
#define PPM_IN_CH7 0 //57
#define PPM_IN_CH8 60


#define PPM_OUT_CH1 48
#define PPM_OUT_CH2 49
#define PPM_OUT_CH3 50
#define PPM_OUT_CH4 36
#define PPM_OUT_CH5 46
#define PPM_OUT_CH6 45
#define PPM_OUT_CH7 301
#define PPM_OUT_CH8 225



	
#include "APM_RC.h"
#include "../Arduino_Mega_ISR_Registry/Arduino_Mega_ISR_Registry.h"

#ifndef MOTOR_PWM_FREQ

  #define MOTOR_PWM_FREQ 60

#endif

//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
#define MINTHROTTLE 1200 // for Giancod 10A
//#define MINTHROTTLE 1120 // for Super Simple ESCs 10A

//#define SERIAL_SUM_PPM  // for specific receiver with only one PPM sum signal, on digital PIN 2
#define PPM_ORDER         PITCH,YAW,THROTTLE,ROLL,MODE,AUX1,AUX2,AUX3 //For Graupner/Spektrum
#define PPM_ORDER         PITCH,YAW,THROTTLE,ROLL,MODE,AUX1,AUX2,AUX3 //For JETI



// *************************
// motor and servo functions
// *************************

#define MINCOMMAND 1000
#define MAXCOMMAND 1850



//static byte receiverPin[12] = {PPM_IN_CH1,PPM_IN_CH2,PPM_IN_CH3,PPM_IN_CH4,PPM_IN_CH5,PPM_IN_CH6,PPM_IN_CH7,PPM_IN_CH8};


#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075


//#define MINOFFWIDTH 12000
//#define MAXOFFWIDTH 24000

// PATCH FOR FAILSAFE AND FRSKY
#define MINOFFWIDTH 1000
#define MAXOFFWIDTH 30000



//PIN assignment
#define THROTTLEPIN 2
#define ROLLPIN 4
#define PITCHPIN 5
#define YAWPIN 6
#define AUX1PIN 7
// alias for RC
#define ROLL 4
#define PITCH 2
#define YAW 3
#define THROTTLE 1
#define MODE 5
#define AUX1 6
#define AUX2 7
#define AUX3 8

#include <inttypes.h>

#define MAX_MOTORS 8


class APM_RC_MP32 : public APM_RC_Class
{
private:
  public:
	/// Constructor
	///
	/// @param key      EEPROM storage key for the channel trim parameters.
	/// @param name     Optional name for the group.
	///
	
  public:
	unsigned int ppm_sum_channel;
	unsigned int input_channel_ch1;
	unsigned int input_channel_ch2;
	unsigned int input_channel_ch3;
	unsigned int input_channel_ch4;
	unsigned int input_channel_ch5;
	unsigned int input_channel_ch6;
	unsigned int input_channel_ch7;
	unsigned int input_channel_ch8;
	unsigned int input_channel_ch9;
	unsigned int input_channel_ch10;
	unsigned int input_channel_ch11;
	unsigned int input_channel_ch12;
	
	
    unsigned int output_channel_ch1;
	unsigned int output_channel_ch2;
	unsigned int output_channel_ch3;
	unsigned int output_channel_ch4;
	unsigned int output_channel_ch5;
	unsigned int output_channel_ch6;
	unsigned int output_channel_ch7;
	unsigned int output_channel_ch8;
	unsigned int output_channel_ch9;
	unsigned int output_channel_ch10;
	unsigned int output_channel_ch11;
	unsigned int output_channel_ch12;

	
	public:
	
	unsigned char motors;
	unsigned char motorread,MissingMotor;
	unsigned char motor_rx[16],motor_rx2[16];
	unsigned char MotorPresent[8];
	unsigned char MotorError[8];
	unsigned char radio_mode;
	unsigned char iboard;




	//unsigned int                     g_u16Pulse[NUM_RCI_CH];                // RCInput pulse widths
	//volatile RCInputFlags            g_RCIFlags[NUM_RCI_CH];        // RCInput channel flags
	//DiagnosticsFlags               g_DiagnosticsFlags;
   

	APM_RC_MP32();
	
	void Init( char board,Arduino_Mega_ISR_Registry * isr_reg , FastSerial * _serial);
	
	void InitDefaultPPM(char board);
	void InitDefaultPPMSUM(char board);
	void InitPPM(void);
	void InitDefaultPWM(void);
	void InitPPMSUM(void);
	void InitPWM(FastSerial * _serial); 
	
	void OutputCh(unsigned char ch, uint16_t pwm);
    uint16_t                OutputCh_current(uint8_t ch);
	uint16_t InputCh(unsigned char ch);
	unsigned char GetState();
	bool setHIL(int16_t v[NUM_CHANNELS]);
	void clearOverride(void);
	void Force_Out(void);
	void SetFastOutputChannels(uint32_t chmask, uint16_t speed_hz = 400);

  void enable_out(uint8_t);
  void disable_out(uint8_t);

	void Force_Out0_Out1(void);
	void Force_Out2_Out3(void);
	void Force_Out6_Out7(void);
	
	unsigned short GetTimerReloadValue(unsigned short uFreq);
	void InitFQUpdate(unsigned char channel, FastSerial * _serial);

  private:
    static volatile uint16_t _PWM_RAW[NUM_CHANNELS];
    static volatile uint8_t _radio_status;
    int16_t _HIL_override[NUM_CHANNELS];

};


#endif
