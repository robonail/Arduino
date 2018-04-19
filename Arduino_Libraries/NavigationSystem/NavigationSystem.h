
/*
  Cart.h - Library for the mechanical Cart
  Author         Date          Note
  _____________  __________    __________________________________
  Michael Baird  5/5/2014      Initial creation
*/

#ifndef NavigationSystem_h
#define NavigationSystem_h
//libraries 
#include "Arduino.h"
#include "Timer.h"
#include "Encoder.h"
#include "RoboUtility.h"
#include "AccelStepper.h"
#include "Sabertooth.h"

#define _stepPin5V 2   // Blue  - Pul +
#define _stepPin 3 //Red   - Pul -
#define _dirPin5V 4    // Green - Dir +
#define _dirPin 5 // White - Dir - 


#define _drillRelay1and3Pin A10   // Blue  +5v Drills 1 & 3
#define _drillRelay2and4Pin A11 //Red   Drills +5V 2 & 4
#define _drillRelay1and3NegPin A12    // Green -5V Drills 1 & 3
#define _drillRelay2and4NegPin A13 // White -5V Drills 2 & 4
//#define _pulseDelay 350 // about 200RPM, anything below 300 stalls the motor
//#define _stepsPerRotation 400 //smalest the DQ860MA will allow


#define _rotations  3 //full rotations 

class NavigationSystem
{
  public:
    NavigationSystem();
  	// for use by ISR glue routines
    void setup(); 
    void update();
  	void speedOutput();
    void move(float xAxisDistanceInches, float yAxisDistanceInches);
  	/*void moveRight(float distanceInches);
    void moveLeft(float distanceInches);
    void moveUp(float distanceInches);
    void moveDown(float distanceInches);
    */
    static void moveDrill(int drillNumber, int direction);
    static void stopDrill(int drillNumber);
    static void finishBrakingDrill();
    void stop();
    static bool isMoving();
    void switchDrillSet(char drillSet);
    static void finishSwitchingDrillSet();

    static float distanceLeftToMoveXAxis();
    static float distanceLeftToMoveYAxis();

  private:
    //static bool drillDirectionForward[4] = {false,false,false,false};
  	static NavigationSystem * instance0_;
  	static void isr0();
    static void rampDrills();
    static void sendInfo();
    void moveSteppers();
    static void checkLimitSwitches();
    static void saveToEEPROM();
    static void getEEPROM();

    // defines pins numbers
    const int stepPin5V = 2;   // Blue  - Pul +
    const int stepPin = 3; //Red   - Pul -
    const int dirPin5V = 4;    // Green - Dir +
    const int dirPin = 5; // White - Dir - 
    const int pulseDelay = 700; // about 200RPM, anything below 600 stalls the motor
    const int rotations = 3; //full rotations 
    const int stepsPerRotation = 400; //smalest the DQ860MA will allow

};
#endif

