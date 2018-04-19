
/*
  RoboStepper.h - Library for the controlling the HSS86 stepper motor
   We could NOT use typical logic to move the motor because it was
   not a constant frequency.  This led to the stepper rattling and
   making a jerking motion when the arduino was interrupted for 
   other tasks like i2c communication. 

   This library is tested with the HSS86 stepper motor controller
   from RATM motor. you can find on ebay 8N.m model. 48vDC power supply
  Author         Date          Note
  _____________  __________    __________________________________
  Michael Baird  12/31/2017      Initial creation
*/

#ifndef RoboStepper_h
#define RoboStepper_h
//libraries 
#include "Arduino.h"




class RoboStepper
{
  public:
    RoboStepper(); 
    void setup(); 
    void run();
    void stop();
    bool isStopped();
    void start();
    void move(float xAxisDistanceInches, float yAxisDistanceInches);
    float getHorizontalDistance();
    float getVerticalDistance();
    float getHorizontalDistanceToGo();
    float getVerticalDistanceToGo();
    float getUpAvailableInches();
    float getDownAvailableInches();
    float getLeftAvailableInches();
    float getRightAvailableInches();
    void enableVerticalStepper(int number);
    void disableVerticalStepper(int number);
    bool isCalibrated();
    void calibrate();
    void printCalibrationInfo();
    void RoboStepper::setCalibrationValue(String name, String value);
    bool isMoving();
    void saveToEEPROM();
    void getEEPROM();

  private: 
    /*unsigned char h2int(char c);*/
    void checkLimitSwitches();
	  void debounceLimitSwitch(int index, int pin);
    void accelerateMotor();
    void calculateDistance();
    void stopHorizontalMovement();
    void stopVerticalMovement();
    void stopVerticalMovement(int verticalLegNumber);
};
#endif

