
/*
  Cart.h - Library for the mechanical Cart
  Author         Date          Note
  _____________  __________    __________________________________
  Michael Baird  5/5/2014      Initial creation
*/

#ifndef Cart_h
#define Cart_h
//libraries 
#include "Arduino.h"
#include "Timer.h"
#include "Encoder.h"
#include "RoboUtility.h"


#define _gunsDisabled 9
#define _gun1 10
#define _gun2 11
#define _gun3 12

#define _cartSideWheelsDisabled 51
#define _cartSideWheelsUp 49
#define _cartSideWheelsDown 47
#define _encoderPinA 2
#define _encoderPinB 3


#define _speedInputPWM 5                    //Green
#define _startStop 6                       //Black
#define _runBrake 7                         //White
#define _clockwiseCounter 8                 //Gray
//#define _alarmReset 7                       //Purple
//#define _alarmOutput 9                      //Brown
//#define _intVRext 10                        //LightBlue
#define _speedOutput 18                      //Red (was on pin 20 before 10/4/2014)



class Cart
{
  public:
    Cart();
	// for use by ISR glue routines
	volatile long _verticalPulses; //Store pulses of the rotary encoder 
	volatile long _horizontalPulses; //Store pulses of the rotary encoder
	float currentDistanceVertical; //Measures the horizontal distance 
	float currentDistanceHorizontal; //Measures the vertical distance
	float targetDistanceVertical; //The target position of the machine from the start position
	float targetDistanceHorizontal;//The target position of the machine from the start position
	int speedPWM; //value of 0 to 255 for speed of trolley

	boolean left;
    long readVerticalPulses();
	long readHorizontalPulses();
    float readVerticalInches();
	float readHorizontalInches();
	void horizontalLeft (String distanceToMove);
	void horizontalRight (String distanceToMove);
	void setSpeed(String speed);
    void sideWheelsUp(String time);
    void sideWheelsDown(String time);
	void calculateDistance();
	void stopMoving();
	void calculateCurrentDistanceAndStopAtTarget();
    void fireNailGun(String gunNumber);
    void update();
	void begin(); 
	void speedOutput(); 
  private:
	static Cart * instance0_;
	static void isr0();
	//static RoboUtility robo;
	//Rotary Encoder to measure vertical movement
	//Encoder _verticalEncoder(_encoderPinA,_encoderPinB);
};
#endif

