
/*
  RoboStepper.h - Library for the controlling the HSS86 stepper motor
   We could NOT use typical logic to move the motor because it was
   not a constant frequency.  This led to the stepper rattling and
   making a jerking motion when the arduino was interrupted for 
   other tasks like i2c communication. 

   This library is tested with the HSS86 stepper motor controller
   from RATM motor. you can find on ebay 8N.m model. 48vDC power supply

   The Motor 86HSE8N Closed Loop Servo motor (8N.m, 116mm Length, 6A)
   torque curve shows it SLOWLY loses torque between 1Khz and 8Khz. 
   Then, the torque drops quickly. We can adjust the parameter
   calibration.stepperPwmTargetPeriod to increase/decrease speed if we 
   need more torque.

  Author         Date          Note
  _____________  __________    __________________________________
  Michael Baird  12/31/2017      Initial creation
*/


#include "Arduino.h"
#include "RoboStepper.h"
#include "TimerOne.h"   //Vertical steppers (pins 12 and 13) - library interrupt for arduino mega 2650 pins 11, 12 and 13 
#include "TimerThree.h" //Horizontal stepper (pin 2) - library interrupt for arduino mega 2650 pins 2, 3, and 5
#include "EEPROM.h"
#include "RoboUtility.h"

#define HORIZONTAL_LEFT 0
#define HORIZONTAL_RIGHT 1
#define VERTICAL_1_UP 2
#define VERTICAL_1_DOWN 3
#define VERTICAL_2_UP 4
#define VERTICAL_2_DOWN 5


//Stepper motor and limit switches
const int _stepperRelay = 12;

const int _horizontalLeftLimit = 35; //was 29
const int _horizontalRightLimit = 40; //was 34

const int _horizontalStepperEnabled = 43;
const int _horizontalStepperDirection = 44; //45; //46; //42;
const int _horizontalStepperPulse = 13; //PWM Timer`
const int _horizontalStepperAlarm = 46;
const int _horizontalStepperPending = 47;
/*
const int _horizontalStepperEnabled = 39;
const int _horizontalStepperDirection = 38;
const int _horizontalStepperPulse = 13; //PWM Timer`
const int _horizontalStepperAlarm = 36;
const int _horizontalStepperPending = 35;
*/
const int _vertical1UpLimit = 31; //24; //40;
const int _vertical1DownLimit = 32; //25; //41;
const int _vertical1StepperEnabled = A15; //A14; //26; //42;
const int _vertical1StepperDirection = A14; // A15; //27; //43;
const int _vertical1StepperPulse = 3; //PWM Timer3
const int _vertical1StepperAlarm = 51; //28; //44;
const int _vertical1StepperPending = 50; //29; //45;

const int _vertical2UpLimit = 36; //30; //46;
const int _vertical2DownLimit = 37; //32; //47;
const int _vertical2StepperEnabled = 48;
const int _vertical2StepperDirection = 49;
const int _vertical2StepperPulse = 2; //PWM Timer3
const int _vertical2StepperAlarm = 50;
const int _vertical2StepperPending = 51;

struct StepperCalibration {

  int stepsPerRotation = 1600; //steps per rotation the HSS86 driver is set to
  unsigned long stepperAccelerationPeriodMicros = 300000; //how fast to ramp up/down the motor when starting/stopping
  unsigned long stepperPwmTargetPeriod = 200; //125; // 100 us = 10Khz which gives 360rpm @ 1600 steps/revolution, 125us=8Khz. 250 microseconds is 0.000250 sec  or 4Khz

  float stepperVerticalAdjustment = 0.5; //meaning 0.5 motor turns spins the rod 1 time (40 tooth to 20 tooth ratio on pulleys) 
  float stepperHorizontalAdjustment = 0.5; //meaning 0.5 motor turns spins the rod 1 time (40 tooth to 20 tooth ratio on pulleys) 

  float xToleranceInches=0.05f; //within 1/32nd of an inch 
  float yToleranceInches=0.05f; //within 1/32nd of an inch
  float xLimitOfThreadedRod = 30.000f; //needs calibrated() - how far can we move before we reach the end limit switch
  float yLimitOfThreadedRod = 99.999f; //needs calibrated() - how far can we move before we reach the end limit switch 


  float xTotalMovement = 0.000f;  
  float yTotalMovement = 0.000f;  
  float xTotalLostMovement = 0.000f;  
  float yTotalLostMovement = 0.000f;  

  unsigned long debounceCount = 3;    // the debounce time counter in millis(); increase if the noise/electro-magnetic-interference occurs with the limit switches
};

StepperCalibration stepperCalibration;


RoboUtility stepperUtility;

unsigned long stepperHorizontalStartedMicros=micros();
unsigned long stepperVerticalStartedMicros=micros();

unsigned long stepperHorizontalPwmCurrentPeriod = 0;
unsigned long stepperHorizontalPwmCurrentPeriodStartedMicros = 0;


unsigned long stepperVerticalPwmCurrentPeriod = 0;
unsigned long stepperVerticalPwmCurrentPeriodStartedMicros = 0;

bool movingLeft = false;
bool movingRight = false;
bool movingUp = false;
bool movingDown = false;


// we debounce the limit switches to avoid flickering/interference from 
//  triggering them incorrectly. 
unsigned long lastDebounceTime[6]= {0,0,0,0,0,0};  // the last time we read the switch
int debounceCounter[6]= {0,0,0,0,0,0};  // count the milliseconds for each reading
bool atLimitSwitch[6] = {false,false,false,false,false,false}; //whether the stepper is at limit switch
unsigned long atLimitSwitchCount[6] = {0,0,0,0,0,0};  // count the number of times we hit each limit switch 

float xTargetInches = 0.0f; //when moving, stores the target on the threaded rod
float yTargetInches = 0.0f; //when moving, stores the target on the threaded rod
float xCurrentPositionInches = 0.0f; //stores position while currently in motion on PWM() Timer
float yCurrentPositionInches = 0.0f; //stores position while currently in motion on PWM() Timer
float xPreviousPositionInches = 55.555f; //needs calibrated() - stores position while after completing motion. this is the position on the threaded rod. Note: we initialize to 555.5 so it can "think" we are in the center. calibrate() will calculate the correct number
float yPreviousPositionInches = 55.555f; //needs calibrated() - stores position while after completing motion. this is the position on the threaded rod. Note: we initialize to 555.5 so it can "think" we are in the center. calibrate() will calculate the correct number
float xMovingDistance = 0.0f; //stores how far the previous call to Move() asked us to travel
float yMovingDistance = 0.0f; //stores how far the previous call to Move() asked us to travel

bool y1Enabled = true; // in case we want to stop 1 of the two vertical steppers
bool y2Enabled = true; // in case we want to stop 1 of the two vertical steppers



bool calibrated = true; //we calibrate x/yCurrentPositionInches to the limit switches by moving to them on system start 

RoboStepper::RoboStepper()
{
//The constructor runs before certain arduino components are loaded including interrupts needed for the delay() and attachInterrup() functions.
// Thus, we added method setup() to handle constructor setup 
}
void RoboStepper::setup()
{  

  //Stepper motor and limit switches
  pinMode(_horizontalLeftLimit, INPUT_PULLUP);
  pinMode(_horizontalRightLimit, INPUT_PULLUP); 
  pinMode(_horizontalStepperDirection, OUTPUT); 
  pinMode(_horizontalStepperPulse, OUTPUT); 

  pinMode(_vertical1UpLimit, INPUT_PULLUP);
  pinMode(_vertical1DownLimit, INPUT_PULLUP); 
  pinMode(_vertical1StepperDirection, OUTPUT); 
  pinMode(_vertical1StepperPulse, OUTPUT); 

  pinMode(_vertical2UpLimit, INPUT_PULLUP);
  pinMode(_vertical2DownLimit, INPUT_PULLUP); 
  pinMode(_vertical2StepperDirection, OUTPUT); 
  pinMode(_vertical2StepperPulse, OUTPUT); 

  pinMode(_stepperRelay, OUTPUT); 
  digitalWrite(_stepperRelay, LOW); //Relay for 48V stepper motor controller power

  xCurrentPositionInches = 0.0f; //needs calibrate() - we assume the horizontal stepper is all the way to the left on system start
  yCurrentPositionInches = 0.0f; //needs calibrate() - we assume the vertical stepper is all the way to the bottom on system start

  Timer1.initialize(stepperCalibration.stepperPwmTargetPeriod); // 100 us = 10Khz (360rpm @ 1600 steps/revolution), 125=8Khz. set a timer of length 250 microseconds (or 0.000250 sec - or 4Khz)
  Timer3.initialize(stepperCalibration.stepperPwmTargetPeriod); // 100 us = 10Khz (360rpm @ 1600 steps/revolution), 125=8Khz. set a timer of length 250 microseconds (or 0.000250 sec - or 4Khz)
  printCalibrationInfo();

}

void RoboStepper::move(float xAxisDistanceInches, float yAxisDistanceInches)
{

  printCalibrationInfo();
  if(digitalRead(_stepperRelay)==HIGH)
  {
    Serial.println("RoboStepper::move() ERROR - stepper controller relay OFF. call RoboStepper.start() first");
      return;
  }

 if(xAxisDistanceInches>0){
    if(atLimitSwitch[HORIZONTAL_RIGHT]) //we are at the limit and cannot move left
    {
      Serial.println("RoboStepper::move() ERROR - unable to move right because limit switch reached");
      return;
    }

    /*
    if(xAxisDistanceInches  > stepperCalibration.xLimitOfThreadedRod + stepperCalibration.xToleranceInches - xPreviousPositionInches) // xPositionOnThreadedRod)
    { 
      Serial.println("RoboStepper::move() ERROR - right distance is past limit switch");
      return;
    }*/

    Serial.println("moving right "+String(xAxisDistanceInches));
    movingRight=true;
    digitalWrite(_horizontalStepperDirection,HIGH);
  }
  else if(xAxisDistanceInches<0){
    if(atLimitSwitch[HORIZONTAL_LEFT]) //we are at the limit and cannot move left
    {
      Serial.println("RoboStepper::move() ERROR - unable to move left because limit switch reached");
      return;
    }
    
    /*
    if(abs(xAxisDistanceInches) > xPreviousPositionInches + stepperCalibration.xToleranceInches) //xPositionOnThreadedRod)
    { 
      Serial.println("RoboStepper::move() ERROR - left distance is past limit switch");
      return;
    }*/

    Serial.println("moving left " +String(xAxisDistanceInches));
    movingLeft=true;
    digitalWrite(_horizontalStepperDirection,LOW);
  }


  if(yAxisDistanceInches>0){
    if((y1Enabled && atLimitSwitch[VERTICAL_1_UP]) || 
      (y2Enabled && atLimitSwitch[VERTICAL_2_UP])) //we are at the limit and cannot move left
    {
      Serial.println("RoboStepper::move() ERROR - unable to move up because limit switch reached");
      // TODO: put back in once connected 
      return;
    }
    /*
    if(isCalibrated() &&
      (yAxisDistanceInches > stepperCalibration.yLimitOfThreadedRod - yPreviousPositionInches)) 
    { 
      Serial.println("RoboStepper::move() ERROR - up distance is past limit switch");
      return;
    }*/

    Serial.println("moving up "+String(yAxisDistanceInches));
    movingUp=true;
    digitalWrite(_vertical1StepperDirection,HIGH);
    digitalWrite(_vertical2StepperDirection,HIGH);
  }
  else if(yAxisDistanceInches<0){
    if((y1Enabled && atLimitSwitch[VERTICAL_1_DOWN]) || 
      (y2Enabled && atLimitSwitch[VERTICAL_2_DOWN]))  
    {
      Serial.println("RoboStepper::move() ERROR - unable to move down because limit switch reached");
      Serial.println("true="+String(true)+"   false="+String(false));
      Serial.println("Down status: " + String(y1Enabled) + ":" + String(atLimitSwitch[VERTICAL_1_DOWN]) + "   /   " +
        String(y2Enabled) + ":" + String(atLimitSwitch[VERTICAL_2_DOWN]));
      // TODO: put back in once connected 
      return;
    }

    /*
    if(isCalibrated() &&
      (abs(yAxisDistanceInches) > yPreviousPositionInches))  
    { 
      Serial.println("RoboStepper::move() ERROR - down distance is past limit switch");
      // TODO: put back in once connected 
       return;
    }*/
    Serial.println("moving down " +String(yAxisDistanceInches));
    movingDown=true;
    digitalWrite(_vertical1StepperDirection,LOW);
    digitalWrite(_vertical2StepperDirection,LOW);
  }

  xMovingDistance = xAxisDistanceInches;
  yMovingDistance = yAxisDistanceInches;
  xTargetInches = xAxisDistanceInches + getHorizontalDistance();
  yTargetInches = yAxisDistanceInches + getVerticalDistance();

  //xPreviousPositionInches = 0.0f;
  //yPreviousPositionInches = 0.0f;
  if (xAxisDistanceInches!=0.0f)
    stepperHorizontalPwmCurrentPeriod = stepperCalibration.stepperPwmTargetPeriod * long(pow(1.15,20-1)); //exponentially increase speed. start off at 1.2 to the power of 20 target speed
  if (yAxisDistanceInches!=0.0f)
    stepperVerticalPwmCurrentPeriod = stepperCalibration.stepperPwmTargetPeriod; //TODO: Put back in to accelerate vertical  * long(pow(1.15,20-1)); //exponentially increase speed. start off at 1.2 to the power of 20 target speed
  

  int duty = (5.0f/float(stepperHorizontalPwmCurrentPeriod))*1024;  // 5 millisecond pulse/duty. 

  Timer1.pwm(_horizontalStepperPulse, duty, stepperHorizontalPwmCurrentPeriod); // 5 millisecond pulse/duty. 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency
  
  duty = (5.0f/float(stepperVerticalPwmCurrentPeriod))*1024;  // 5 millisecond pulse/duty. 
  if(y1Enabled)
    Timer3.pwm(_vertical1StepperPulse, duty, stepperVerticalPwmCurrentPeriod); // 5 millisecond pulse/duty. 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency
  if(y2Enabled)
    Timer3.pwm(_vertical2StepperPulse, duty, stepperVerticalPwmCurrentPeriod); // 5 millisecond pulse/duty. 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency
 
  stepperHorizontalStartedMicros=micros();
  stepperHorizontalPwmCurrentPeriodStartedMicros=micros(); 
  stepperVerticalStartedMicros=micros();
  stepperVerticalPwmCurrentPeriodStartedMicros=micros(); 

}
 
/*
float RoboStepper::distanceLeftToMoveXAxis(){
  return 0.0f; //TODO TEST PWM stepperHorizontal.distanceToGo()/(2.0f*float(horizontalMicrosteps)*stepperCalibration.stepperHorizontalAdjustment);
} 
*/
void RoboStepper::run(){

  checkLimitSwitches(); //debounces limit switches and STOPS motion if past limit  
  calculateDistance(); //calculates distance based on time at current speed (speed is PWM duty cycle and frequency )
  accelerateMotor(); 

  if((movingLeft && getHorizontalDistanceToGo()+stepperCalibration.xToleranceInches>0)
    ||(movingRight && getHorizontalDistanceToGo()-stepperCalibration.xToleranceInches<0))
  { 
    Serial.println("moving horizontal reached target distance-to-go:"+String(getHorizontalDistanceToGo()));
    stopHorizontalMovement();
  }

  if((movingUp && getVerticalDistanceToGo()-stepperCalibration.yToleranceInches<0)
    ||(movingDown && getVerticalDistanceToGo()+stepperCalibration.yToleranceInches>0))
  { 
    Serial.println("moving vertical reached target distance-to-go:"+String(getVerticalDistanceToGo()));
    stopVerticalMovement();
  }

}


void RoboStepper::accelerateMotor()
{

  //HORIZONTAL  X-AXIS  LOGIC 
  if(stepperHorizontalPwmCurrentPeriod!=0) //x axis/horizontal motor is moving, lets accelerate/decelerate if needed 
  {


    //ACCELERATE/SPEED UP MOTOR
    int period = 20 - (micros()-stepperHorizontalStartedMicros) / (stepperCalibration.stepperAccelerationPeriodMicros/20); 
    if(period < 1) //We've been moving for longer than the stepper Acceleration Period. so lets just move full speed
      period = 1; 

    unsigned long pulsesToReachTarget = 2.0f*float(stepperCalibration.stepsPerRotation)*stepperCalibration.stepperHorizontalAdjustment*abs(getHorizontalDistanceToGo());
    unsigned long newPwmPeriod = stepperCalibration.stepperPwmTargetPeriod * pow(1.15, period-1); 
    unsigned long timeToReachTarget = pulsesToReachTarget*newPwmPeriod; 


    //SLOW DOWN MOTOR IF WE ARE CLOSE TO THE TARGET
    if(timeToReachTarget < stepperCalibration.stepperAccelerationPeriodMicros) //we are close to the target position. lets decelerate
    { 


      int slowDownPeriod = 20 - (timeToReachTarget) / ((stepperCalibration.stepperAccelerationPeriodMicros)/20);
      slowDownPeriod=slowDownPeriod-4; // we slow down much slower than we accelerate

      if(slowDownPeriod <= period) //we are still accelerating. this occurs in short distance movements 
        period = period; 
      else
        period=slowDownPeriod; 

      newPwmPeriod = stepperCalibration.stepperPwmTargetPeriod * pow(1.15, period-1);  // we slow down much slower than we accelerate. notice the 1.10 exponent  
    } 

switch (period) {
    case 1:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+0;
      break;
    case 2:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+5;
      break;
    case 3:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+10;
      break;
    case 4:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+20;
      break;
    case 5:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+30;
      break;
    case 6:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+40;
      break;
    case 7:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+55;
      break;
    case 8:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+70;
      break;
    case 9:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+85;
      break;
    case 10:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+100;
    case 11:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+125;
      break;
    case 12:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+155;
      break;
    case 13:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+190;
      break;
    case 14:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+225;
      break;
    case 15:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+300;
      break;
    case 16:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+375;
      break;
    case 17:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+500;
      break;
    case 18:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+700;
      break;
    case 19:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+1000;
      break;
      break;
    case 20:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+2000;
      break;
  }


    //CHANGE SPEED 
    if(stepperHorizontalPwmCurrentPeriod != newPwmPeriod)  // The motor speed was changed by either the SPEED UP or SLOW code blocks above
    {
      //digitalWrite(_horizontalStepperPulse,LOW);
      //delayMicroseconds(stepperCalibration.stepperPwmTargetPeriod);
      //delay(1);

      stepperHorizontalPwmCurrentPeriod= newPwmPeriod; // exponentially increase speed. //new frequency to set the motor speed to
      stepperHorizontalPwmCurrentPeriodStartedMicros=micros(); //start a new ramping period 
      xPreviousPositionInches+=xCurrentPositionInches; //store the total distance we moved
      xCurrentPositionInches = 0.0f; //zero out the current period distance 
      
      int duty = (5.0f/float(stepperHorizontalPwmCurrentPeriod))*1024; //5 millisecond pulse/duty. 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency
      Timer1.stop();
      Timer1.pwm(_horizontalStepperPulse, duty, stepperHorizontalPwmCurrentPeriod); // 5 millisecond pulse/duty. 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency
      Timer1.start();
      
      /*
      Serial.print("RoboStepper::accelerateMotor() X period:");
      Serial.print(stepperHorizontalPwmCurrentPeriod);
      Serial.print("     at:");
      Serial.print(stepperHorizontalPwmCurrentPeriodStartedMicros);
      Serial.print("     timeToReachTarget:");
      Serial.print(timeToReachTarget);
      Serial.print("     period:");
      Serial.print(period);
      Serial.println(); 
      */
      
      
    }
  }

/*
////Vertical  Y-AXIS  LOGIC 
  if(stepperVerticalPwmCurrentPeriod!=0) //y axis/horizontal motor is moving, lets accelerate/decelerate if needed 
  {


    //ACCELERATE/SPEED UP MOTOR
    int period = 20 - (micros()-stepperVerticalStartedMicros) / (stepperCalibration.stepperAccelerationPeriodMicros/20); 
    if(period < 1) //We've been moving for longer than the stepper Acceleration Period. so lets just move full speed
      period = 1; 

    unsigned long pulsesToReachTarget = 2.0f*float(stepperCalibration.stepsPerRotation)*stepperCalibration.stepperVerticalAdjustment*abs(getVerticalDistanceToGo());
    unsigned long newPwmPeriod = stepperCalibration.stepperPwmTargetPeriod * pow(1.15, period-1); 
    unsigned long timeToReachTarget = pulsesToReachTarget*newPwmPeriod; 


    //SLOW DOWN MOTOR IF WE ARE CLOSE TO THE TARGET
    if(timeToReachTarget < stepperCalibration.stepperAccelerationPeriodMicros) //we are close to the target position. lets decelerate
    { 
 
      int slowDownPeriod = 20 - (timeToReachTarget) / ((stepperCalibration.stepperAccelerationPeriodMicros)/20);
      slowDownPeriod=slowDownPeriod-4; // we slow down much slower than we accelerate

      if(slowDownPeriod <= period) //we are still accelerating. this occurs in short distance movements 
        period = period; 
      else
        period=slowDownPeriod; 

      newPwmPeriod = stepperCalibration.stepperPwmTargetPeriod * pow(1.15, period-1);  // we slow down much slower than we accelerate. notice the 1.10 exponent  
    } 


switch (period) {
    case 1:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+0;
      break;
    case 2:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+5;
      break;
    case 3:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+10;
      break;
    case 4:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+20;
      break;
    case 5:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+30;
      break;
    case 6:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+40;
      break;
    case 7:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+55;
      break;
    case 8:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+70;
      break;
    case 9:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+85;
      break;
    case 10:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+100;
    case 11:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+125;
      break;
    case 12:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+155;
      break;
    case 13:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+190;
      break;
    case 14:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+225;
      break;
    case 15:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+300;
      break;
    case 16:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+375;
      break;
    case 17:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+500;
      break;
    case 18:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+700;
      break;
    case 19:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+1000;
      break;
      break;
    case 20:
      newPwmPeriod=stepperCalibration.stepperPwmTargetPeriod+2000;
      break;
  }
    //CHANGE SPEED 
    if(stepperVerticalPwmCurrentPeriod != newPwmPeriod)  // The motor speed was changed by either the SPEED UP or SLOW code blocks above
    {
      stepperVerticalPwmCurrentPeriod= newPwmPeriod; // exponentially increase speed. //new frequency to set the motor speed to
      stepperVerticalPwmCurrentPeriodStartedMicros=micros(); //start a new ramping period 
      yPreviousPositionInches+=yCurrentPositionInches; //store the total distance we moved
      yCurrentPositionInches = 0.0f; //zero out the current period distance 
      
      int duty = (5.0f/float(stepperVerticalPwmCurrentPeriod))*1024; //5 millisecond pulse/duty. 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency

     
      if(y1Enabled)
        Timer3.stop();
        Timer3.pwm(_vertical1StepperPulse, duty, stepperVerticalPwmCurrentPeriod); // 5 millisecond pulse/duty. 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency
        Timer3.start();
      if(y2Enabled)
        Timer3.stop();
        Timer3.pwm(_vertical2StepperPulse, duty, stepperVerticalPwmCurrentPeriod); // 5 millisecond pulse/duty. 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency
        Timer3.start();
    }
  }
  */
 
 
/*
  if(stepperHorizontalPwmCurrentPeriod!=0) //motor is moving, lets accelerate/decelerate if needed 
  {
    unsigned long pulsesToReachTarget = 2.0f*float(stepperCalibration.stepsPerRotation)*stepperCalibration.stepperHorizontalAdjustment*abs(getHorizontalDistanceToGo());
    unsigned long timeToReachTarget = pulsesToReachTarget*stepperHorizontalPwmCurrentPeriod;
    
    int period = 1; //EXP stepperHorizontalPwmCurrentPeriod/stepperCalibration.stepperPwmTargetPeriod;  //we ramp up in periods 1/20th of stepperCalibration.stepperAccelerationPeriodMicros. 
    
    for (int i=1; i<=20; i++)
    { 
      if (stepperHorizontalPwmCurrentPeriod==long(stepperCalibration.stepperPwmTargetPeriod*long(pow(1.15,i-1)))) //EXP
      {
        period = i; 
      }
    }
    //DECELERATE/SLOW MOTOR
    //you'll notice we divide the acceleration time by 4. This is because as we slow
    // down the spinning the time to target increases. Thus, we end up darn close
    // to the desired acceleration time by dividing by 4
    if(timeToReachTarget < stepperCalibration.stepperAccelerationPeriodMicros/4) //we are close to the target position. lets decelerate
    {
      period = period+1; //EXP (stepperHorizontalPwmCurrentPeriod/stepperCalibration.stepperPwmTargetPeriod) +1; 
      //we ramp up in periods 1/20th of stepperCalibration.stepperAccelerationPeriodMicros 
      if(period>20 || period ==1)
        return; //we are already moving at the desired RPM  
    } 

    //ACCELERATE/SPEED UP MOTOR
    if ((stepperHorizontalPwmCurrentPeriod>stepperCalibration.stepperPwmTargetPeriod &&  //we are below the targetSpeed of the motor  
      timeToReachTarget > stepperCalibration.stepperAccelerationPeriodMicros)){ //we are not decelerating, so lets accelerate

      if(micros()-stepperHorizontalPwmCurrentPeriodStartedMicros < stepperCalibration.stepperAccelerationPeriodMicros/20)
        return; // it has not been a full 1/20th period yet. no need to accelerate 
    
      period--; //EXP = 20-int((float(micros()-stepperHorizontalStartedMicros)/float(stepperCalibration.stepperAccelerationPeriodMicros)) *20);   //we ramp up in periods 1/20th of stepperCalibration.stepperAccelerationPeriodMicros. 
      
      if(period<1) //This occurs when it is longer than the acceleration period. typically something interrupted the processing (e.g. i2c communication)
        period=1; //lets go full speed since we've exceeded the acceleration period 

    }


    if(stepperHorizontalPwmCurrentPeriod != stepperCalibration.stepperPwmTargetPeriod * long(pow(1.15, period-1)))  //EXP The motor speed was changed by one of the code blocks above
    {

      if(period >= 20)
        period=20; //20x is the slownest we want 
       

      stepperHorizontalPwmCurrentPeriod= stepperCalibration.stepperPwmTargetPeriod * long(pow(1.15,period-1)); //exponentially increase speed. //new frequency to set the motor speed to
      stepperHorizontalPwmCurrentPeriodStartedMicros=micros(); //start a new ramping period 
      xPreviousPositionInches+=xCurrentPositionInches; //store the total distance we moved
      xCurrentPositionInches = 0.0f; //zero out the current period distance 
      
      int duty = (5.0f/float(stepperHorizontalPwmCurrentPeriod))*1024; //5 millisecond pulse/duty. 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency

      Timer1.pwm(_horizontalStepperPulse, duty, stepperHorizontalPwmCurrentPeriod); // 5 millisecond pulse/duty. 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency
      
      Serial.print("RoboStepper::accelerateMotor() X period:");
      Serial.print(stepperHorizontalPwmCurrentPeriod);
      Serial.print("     at:");
      Serial.print(stepperHorizontalPwmCurrentPeriodStartedMicros);
      Serial.print("     timeToReachTarget:");
      Serial.print(timeToReachTarget);
      Serial.print("     period:");
      Serial.print(period);
      Serial.println(); 
      
    }
  }

  if(stepperVerticalPwmCurrentPeriod!=0) //motor is moving, lets accelerate/decelerate if needed 
  {

    unsigned long pulsesToReachTarget = 2.0f*float(stepperCalibration.stepsPerRotation)*stepperCalibration.stepperVerticalAdjustment*abs(getVerticalDistanceToGo());
    unsigned long timeToReachTarget = pulsesToReachTarget*stepperVerticalPwmCurrentPeriod;
    int period = 1; stepperVerticalPwmCurrentPeriod/stepperCalibration.stepperPwmTargetPeriod;  //we ramp up in periods 1/20th of stepperCalibration.stepperAccelerationPeriodMicros. 
    
    for (int i=1; i<=20; i++)
    {
      if (stepperHorizontalPwmCurrentPeriod==stepperCalibration.stepperPwmTargetPeriod * long(pow(1.15,i-1))) //EXP
        period = i;
    }
    //DECELERATE/SLOW MOTOR
    //you'll notice we divide the acceleration time by 4. This is because as we slow
    // down the spinning the time to target increases. Thus, we end up darn close
    // to the desired acceleration time by dividing by 4
    if(timeToReachTarget < stepperCalibration.stepperAccelerationPeriodMicros/4) //we are close to the target position. lets decelerate
    {
      period++; // = (stepperVerticalPwmCurrentPeriod/stepperCalibration.stepperPwmTargetPeriod) +1; 
      //we ramp up in periods 1/20th of stepperCalibration.stepperAccelerationPeriodMicros
      if(period>20 || period ==1)
        return; //we are already moving at the desired RPM  
    } 

    //ACCELERATE/SPEED UP MOTOR
    if ((stepperVerticalPwmCurrentPeriod>stepperCalibration.stepperPwmTargetPeriod &&  //we are below the targetSpeed of the motor  
      timeToReachTarget > stepperCalibration.stepperAccelerationPeriodMicros)){ //we are not decelerating, so lets accelerate

      if(micros()-stepperVerticalPwmCurrentPeriodStartedMicros < stepperCalibration.stepperAccelerationPeriodMicros/20) //20
        return; // it has not been a full 1/20th period yet. no need to accelerate 
    
      period = period-1; //EXP 20-int((float(micros()-stepperVerticalStartedMicros)/float(stepperCalibration.stepperAccelerationPeriodMicros)) * 20);   //we ramp up in periods 1/20th of stepperCalibration.stepperAccelerationPeriodMicros. 
      
      if(period<1) //This occurs when it is longer than the acceleration period. typically something interrupted the processing (e.g. i2c communication)
        period=1; //lets go full speed since we've exceeded the acceleration period 
    }


    if(stepperVerticalPwmCurrentPeriod != stepperCalibration.stepperPwmTargetPeriod * long(pow(1.15, period-1)))  //EXP The motor speed was changed by one of the code blocks above
    {

      if(period >= 20)
        period=20; //20x is the slownest we want 

      stepperVerticalPwmCurrentPeriod= stepperCalibration.stepperPwmTargetPeriod* long(pow(1.15,period-1)); //EXP  //new frequency to set the motor speed to
      stepperVerticalPwmCurrentPeriodStartedMicros=micros(); //start a new ramping period 
      yPreviousPositionInches+=yCurrentPositionInches; //store the total distance we moved
      yCurrentPositionInches = 0.0f; //zero out the current period distance 
      
      int duty = (5.0f/float(stepperVerticalPwmCurrentPeriod))*1024; //5 millisecond pulse/duty. 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency

      if(y1Enabled)
        Timer3.pwm(_vertical1StepperPulse, duty, stepperVerticalPwmCurrentPeriod); // 5 millisecond pulse/duty. 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency
      if(y2Enabled)
        Timer3.pwm(_vertical2StepperPulse, duty, stepperVerticalPwmCurrentPeriod); // 5 millisecond pulse/duty. 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency
      
      Serial.print("RoboStepper::accelerateMotor() Y period:");
      Serial.print(stepperVerticalPwmCurrentPeriod);
      Serial.print("     at:");
      Serial.print(stepperVerticalPwmCurrentPeriodStartedMicros);
      Serial.print("     timeToReachTarget:");
      Serial.print(timeToReachTarget);
      Serial.print("     period:");
      Serial.print(period);
      Serial.println(); 
      
    } 
  }
*/







}

void RoboStepper::calculateDistance()
{
  unsigned long timeAtCurrentSpeed = 0;
  float pulses = 0.0f;

  //Horizontal
  if(movingLeft || movingRight)
  {
    timeAtCurrentSpeed = micros()-stepperHorizontalPwmCurrentPeriodStartedMicros;
    pulses = timeAtCurrentSpeed/stepperHorizontalPwmCurrentPeriod;

    xCurrentPositionInches = (2.0f*float(stepperCalibration.stepperHorizontalAdjustment))*(pulses/float(stepperCalibration.stepsPerRotation));
    if(movingLeft)
      xCurrentPositionInches = -1 * xCurrentPositionInches; 


    //if(abs(xCurrentPositionInches) > 100000)
    //{
    //  Serial.println("*************************************************************************");
    //  Serial.print("xCurrentPositionInches: ");
    //  Serial.println(xCurrentPositionInches);
    //  Serial.print("micros(): ");
    //  Serial.println(micros());
    //  Serial.print("stepperHorizontalPwmCurrentPeriodStartedMicros: ");
    //  Serial.println(stepperHorizontalPwmCurrentPeriodStartedMicros);
    //  Serial.print("timeAtCurrentSpeed: ");
    //  Serial.println(timeAtCurrentSpeed);
    //  Serial.print("pulses: ");
    //  Serial.println(pulses);
    //  Serial.print("stepperHorizontalPwmCurrentPeriod: ");
    //  Serial.println(stepperHorizontalPwmCurrentPeriod);
    //  Serial.println("*************************************************************************");
    //  delay(500);
    //}

  }
  
  if(movingUp || movingDown)
  {
    //Vertical
    timeAtCurrentSpeed = micros()-stepperVerticalPwmCurrentPeriodStartedMicros;
    pulses = timeAtCurrentSpeed/stepperVerticalPwmCurrentPeriod;

    yCurrentPositionInches = (2.0f*float(stepperCalibration.stepperVerticalAdjustment))*(pulses/float(stepperCalibration.stepsPerRotation));

    if(movingDown)
      yCurrentPositionInches = -1 * yCurrentPositionInches;
  }

  
}

float RoboStepper::getHorizontalDistanceToGo()
{
  return xTargetInches - getHorizontalDistance();
}

float RoboStepper::getVerticalDistanceToGo()
{
  return yTargetInches - getVerticalDistance();
}

float RoboStepper::getHorizontalDistance()
{
  return xCurrentPositionInches + xPreviousPositionInches;
}

float RoboStepper::getVerticalDistance()
{
  return yCurrentPositionInches + yPreviousPositionInches;
}


float RoboStepper::getUpAvailableInches()
{
  return stepperCalibration.yLimitOfThreadedRod - getVerticalDistance() ;
}

float RoboStepper::getDownAvailableInches()
{
  return getVerticalDistance();
}

float RoboStepper::getLeftAvailableInches()
{
  return getHorizontalDistance();
}

float RoboStepper::getRightAvailableInches()
{
  return stepperCalibration.xLimitOfThreadedRod - getHorizontalDistance();
}

void RoboStepper::stop()
{
  digitalWrite(_stepperRelay, HIGH); //Relay for 48V stepper motor controller power
  stopHorizontalMovement();
  stopVerticalMovement();
}
bool RoboStepper::isStopped()
{
  if(digitalRead(_stepperRelay)==HIGH)
    return true;
  else
    return false;
}
void RoboStepper::start()
{
  digitalWrite(_stepperRelay, LOW); //Relay for 48V stepper motor controller power
}

void RoboStepper::stopHorizontalMovement()
{
   Serial.print("Stopping X axis actual/target ");
    Serial.print(getHorizontalDistance());
    Serial.print(" / ");
    Serial.print(xTargetInches);
    Serial.print(" inches after moving ");
    Serial.print(xMovingDistance);
    Serial.print(" in ");
    Serial.print((micros()-stepperHorizontalStartedMicros)/1000);
    Serial.print(" micros - ");
    Serial.println(micros());

    stepperCalibration.xTotalMovement += abs(xMovingDistance);
  //xPositionOnThreadedRod += xCurrentPositionInches + xPreviousPositionInches;

  //xPreviousPositionInches=0.0f;
  xCurrentPositionInches=0.0f;
  stepperHorizontalPwmCurrentPeriod=0;

  Timer1.setPwmDuty(_horizontalStepperPulse, 0);
  movingLeft=false;
  movingRight=false; 
}

void RoboStepper::stopVerticalMovement()
{

  stepperCalibration.yTotalMovement += abs(yMovingDistance);

  stopVerticalMovement(1);
  stopVerticalMovement(2);
  yCurrentPositionInches=0.0f;
  stepperVerticalPwmCurrentPeriod=0;  
  movingUp=false;
  movingDown=false;
  /*
    Serial.print("Stopping Y axis actual/target ");
    Serial.print(getVerticalDistance());
    Serial.print(" / ");
    Serial.print(yTargetInches);
    Serial.print(" inches after moving ");
    Serial.print(yMovingDistance);
    Serial.print(" in ");
    Serial.print((micros()-stepperVerticalStartedMicros)/1000);
    Serial.print(" micros - ");
    Serial.println(micros()); 

  stepperCalibration.yTotalMovement += abs(yMovingDistance);

  if(isCalibrated()) //once we are calibrated we stop/start both vertical steppers together
  {
    //if(y1Enabled)
    //  y1PositionOnThreadedRod += yCurrentPositionInches + yPreviousPositionInches;
    //if(y2Enabled)
    //    y2PositionOnThreadedRod += yCurrentPositionInches + yPreviousPositionInches;

    //yPreviousPositionInches=0.0f;
    yCurrentPositionInches=0.0f;
    stepperVerticalPwmCurrentPeriod=0; 
    Timer3.setPwmDuty(_vertical1StepperPulse, 0);
    Timer3.setPwmDuty(_vertical2StepperPulse, 0);
    movingUp=false;
    movingDown=false;
  } else  //we are not calibrated, so lets just stop the enabled stepper 
    if((y1Enabled && (atLimitSwitch[VERTICAL_1_UP] || atLimitSwitch[VERTICAL_1_DOWN]))
      || (y2Enabled && (atLimitSwitch[VERTICAL_2_UP] || atLimitSwitch[VERTICAL_2_DOWN])))
      {
        Timer3.setPwmDuty(_vertical1StepperPulse, 0); 
        Timer3.setPwmDuty(_vertical2StepperPulse, 0); 
        //yPreviousPositionInches=0.0f;
        yCurrentPositionInches=0.0f;
        stepperVerticalPwmCurrentPeriod=0;  
        movingUp=false;
        movingDown=false;
      } 
      */
}
  

void RoboStepper::stopVerticalMovement(int verticalLegNumber)
{
  if(stepperUtility.DEBUG_MODE){
    Serial.print("Stopping vertical leg "+String(verticalLegNumber) + " Y axis actual/target ");
    Serial.print(getVerticalDistance());
    Serial.print(" / ");
    Serial.print(yTargetInches);
    Serial.print(" inches after moving ");
    Serial.print(yMovingDistance);
    Serial.print(" in ");
    Serial.print((micros()-stepperVerticalStartedMicros)/1000);
    Serial.print(" micros - ");
    Serial.println(micros()); 
  }


  if(verticalLegNumber==1)
    Timer3.setPwmDuty(_vertical1StepperPulse, 0);

  if(verticalLegNumber==2)
    Timer3.setPwmDuty(_vertical2StepperPulse, 0);

  if(atLimitSwitch[VERTICAL_1_DOWN] && atLimitSwitch[VERTICAL_2_DOWN]){ //all the way to the bottom on both sides. stop moving
    yCurrentPositionInches=0.0f;
    stepperVerticalPwmCurrentPeriod=0;  
    movingUp=false;
    movingDown=false;
  }
}

void RoboStepper::checkLimitSwitches()
{ 
  debounceLimitSwitch(HORIZONTAL_LEFT, _horizontalLeftLimit);
  if(atLimitSwitch[HORIZONTAL_LEFT] && movingLeft)
  {
      Serial.println("Hit Left limit");
      //xPositionOnThreadedRod = 0.0f - stepperCalibration.xToleranceInches; //zero out to recalibrate our position 
      stepperCalibration.xTotalLostMovement+=abs(getHorizontalDistance());

      xPreviousPositionInches = -stepperCalibration.xToleranceInches; //zero out to recalibrate our position 
      xCurrentPositionInches = 0.0f; //zero out to recalibrate our position 
      stepperCalibration.xLimitOfThreadedRod = 29.788f;
      stopHorizontalMovement();
  }

  debounceLimitSwitch(HORIZONTAL_RIGHT, _horizontalRightLimit); 
  if(atLimitSwitch[HORIZONTAL_RIGHT] && movingRight)
  { 

      Serial.println("Hit Right limit");
      stepperCalibration.xTotalLostMovement+=abs(stepperCalibration.xLimitOfThreadedRod-getHorizontalDistance());
      //xPreviousPositionInches +=  xCurrentPositionInches; //Do this becaise we are going to stop immediatly 
      //stepperCalibration.xLimitOfThreadedRod = xPositionOnThreadedRod - stepperCalibration.xToleranceInches; //calibrate the length of the rod til we hit the limit
      stepperCalibration.xLimitOfThreadedRod = 29.788f; //getHorizontalDistance(); //calibrate the length of the rod til we hit the limit
      xPreviousPositionInches = stepperCalibration.xLimitOfThreadedRod+stepperCalibration.xToleranceInches; //currently at the limit
      stopHorizontalMovement();
  }


  // TODO: put back in once connected  
  debounceLimitSwitch(VERTICAL_1_DOWN, _vertical1DownLimit);
  if((atLimitSwitch[VERTICAL_1_DOWN])
    && movingDown)
  {
      Serial.println("Hit Down limit 1");
      //y1PositionOnThreadedRod = 0.0f - stepperCalibration.yToleranceInches; //zero out to recalibrate our position 
      yPreviousPositionInches = 0.0f - stepperCalibration.yToleranceInches; //zero out to recalibrate our position 
      yCurrentPositionInches = 0.0f; //zero out to recalibrate our position 
      stopVerticalMovement(1); 
  }

  debounceLimitSwitch(VERTICAL_1_UP, _vertical1UpLimit);
  if((atLimitSwitch[VERTICAL_1_UP])
     && movingUp)
  {
      Serial.println("Hit Up limit 1");
      //y1LimitOfThreadedRod = y1PositionOnThreadedRod - stepperCalibration.yToleranceInches; //calibrate the length of the rod til we hit the limit
      stepperCalibration.yLimitOfThreadedRod = getVerticalDistance() - stepperCalibration.yToleranceInches; //calibrate the length of the rod til we hit the limit
      stopVerticalMovement();
  }




  debounceLimitSwitch(VERTICAL_2_DOWN, _vertical2DownLimit);
  if((atLimitSwitch[VERTICAL_2_DOWN])
    && movingDown)
  {
      Serial.println("Hit Down limit 2");
      //y2PositionOnThreadedRod = 0.0f - stepperCalibration.yToleranceInches; //zero out to recalibrate our position 
      yPreviousPositionInches = 0.0f - stepperCalibration.yToleranceInches; //zero out to recalibrate our position 
      yCurrentPositionInches = 0.0f; //zero out to recalibrate our position 
      stopVerticalMovement(2);
  }

  debounceLimitSwitch(VERTICAL_2_UP, _vertical2UpLimit);
  if((atLimitSwitch[VERTICAL_2_UP])
     && movingUp)
  {
      Serial.println("Hit Up limit 2");
      //y2LimitOfThreadedRod = y2PositionOnThreadedRod - stepperCalibration.yToleranceInches; //calibrate the length of the rod til we hit the limit
      stepperCalibration.yLimitOfThreadedRod = getVerticalDistance() - stepperCalibration.yToleranceInches; //calibrate the length of the rod til we hit the limit
      stopVerticalMovement();
  }
  
}

void RoboStepper::enableVerticalStepper(int number){
  if(number=1)
    y1Enabled = true; // in case we want to stop 1 of the two vertical steppers
  if(number=2)
    y2Enabled = true; // in case we want to stop 1 of the two vertical steppers
}

void RoboStepper::disableVerticalStepper(int number){
  if(number=1)
    y1Enabled = false; // in case we want to stop 1 of the two vertical steppers
  if(number=2)
    y2Enabled = false; // in case we want to stop 1 of the two vertical steppers
}


bool RoboStepper::isMoving()
{
  if(movingUp || movingDown || movingLeft || movingRight)
    return true;
  else 
    return false; 
}

void RoboStepper::debounceLimitSwitch(int index, int pin)
{ 
  // BEGIN 
  // TODO
  //  Use this only for debugging when NOT connected to the robot 
  //return;
  // END

  // If we have gone on to the next millisecond
  if(millis() != lastDebounceTime[index])
  {
    int reading = digitalRead(pin);

    //TODO: Fix this, the limits are wired WRONG so LOW is tripped. HIGH should be at limit switch
    if(index == VERTICAL_1_DOWN || index == VERTICAL_1_UP ||
      index == VERTICAL_2_DOWN || index == VERTICAL_2_UP)
      if(reading==HIGH)
        reading=LOW;
      else
        reading=HIGH;

    if((reading == HIGH && !atLimitSwitch[index]) //we hit the limit
      || (reading == LOW && atLimitSwitch[index])) //we moved away from the limit 
    {
      if(debounceCounter[index] < stepperCalibration.debounceCount) //we don't want to increment to infinity because it will take too long to switch back
        debounceCounter[index]++;
      else{
        atLimitSwitch[index] = !atLimitSwitch[index];
        if(atLimitSwitch[index])
          atLimitSwitchCount[index]++; // count the number of times we hit the limit switch 
        debounceCounter[index] = 0;

        if(stepperUtility.DEBUG_MODE){
          Serial.println("Stepper limit [index, pin] : value = [" + String(index)+","+String(pin)+"] : "+String(atLimitSwitch[index]));
        }

      }

    }
    lastDebounceTime[index] = millis();
  } 
}


bool RoboStepper::isCalibrated()
{
  return calibrated;
}

void RoboStepper::calibrate()
{
  calibrated=false;
  
  //TODO: comment line below in when we finally fix the calibration issue.
  //      we can NOT have the robot move randomly. need to add a screen or prompt to the mobile app
  return;

  //first calibrate both axis by moving all the way to both limit switches
  move(-100.0, -100.0f); 
  while(movingLeft || movingDown)
    run();

  move(100.0,100.0f);
  while(movingRight || movingUp)
    run();

  //next, lets calibrate the each of the steppers on the vertical axis by moving them up to the limit switch
  y1Enabled=false;
  move(0.0f,100.0f);
  while(movingUp)
    run();

  y1Enabled=true;
  y2Enabled=false;
  move(0.0f,100.0f);
  while(movingUp)
    run();

  y2Enabled=true;
  calibrated=true;
  printCalibrationInfo();

}

void RoboStepper::printCalibrationInfo()
{

  Serial.println("RoboStepper::printCalibrationInfo().................................BEGIN");
  if(isCalibrated())
    Serial.println("....CALIBRATED.........");
  else
    Serial.println("****NOT CALIBRATED NOT CALIBRATED *******");

  Serial.print("x/y position: ");
  Serial.print(getHorizontalDistance());
  Serial.print(" / ");
  Serial.println(getVerticalDistance());

  Serial.print("x/y current, previous : ");
  Serial.print( xCurrentPositionInches);
  Serial.print(" / ");
  Serial.print( yCurrentPositionInches);
  Serial.print(" , ");
  Serial.print( xPreviousPositionInches);
  Serial.print(" / ");
  Serial.println( yPreviousPositionInches); 

  Serial.print("x/y distance left to move: ");
  Serial.print(getHorizontalDistanceToGo());
  Serial.print(" / ");
  Serial.println(getVerticalDistanceToGo());


  Serial.print(" x/y total moved: ");
  Serial.print(stepperCalibration.xTotalMovement);
  Serial.print(" / ");
  Serial.println(stepperCalibration.yTotalMovement);

  Serial.print(" x/y total lost: ");
  Serial.print(stepperCalibration.xTotalLostMovement);
  Serial.print(" / ");
  Serial.println(stepperCalibration.yTotalLostMovement);

  Serial.print("# times limit switch actuated (Left, Right, Up1, Down1, Up2, Down2): "); 
  for(int i=0; i<6; i++)
    {
      Serial.print(atLimitSwitchCount[i]);
      Serial.print(" , ");
    }
  Serial.println();


  Serial.print("x / y limit switch on threaded rod: ");
  Serial.print(stepperCalibration.xLimitOfThreadedRod);
  Serial.print(" / ");
  Serial.println(stepperCalibration.yLimitOfThreadedRod); 

  Serial.println("RoboStepper::printCalibrationInfo().................................END"); 
}

void RoboStepper::setCalibrationValue(String name, String value)
{
  if(name=="s_steps")
    stepperCalibration.stepsPerRotation = value.toInt();

  if(name=="s_acceleration")
    stepperCalibration.stepperAccelerationPeriodMicros = atol(value.c_str());

  if(name=="s_period")
    stepperCalibration.stepperPwmTargetPeriod =  value.toInt();

  if(name=="s_xAdjust")
    stepperCalibration.stepperHorizontalAdjustment = value.toFloat();

  if(name=="s_yAdjust")
    stepperCalibration.stepperVerticalAdjustment = value.toFloat();

  if(name=="s_xTolerance")
    stepperCalibration.xToleranceInches=value.toFloat();

  if(name=="s_yTolerance")
    stepperCalibration.yToleranceInches=value.toFloat();

  if(name=="s_xLength")
    stepperCalibration.xLimitOfThreadedRod = value.toFloat();

  if(name=="s_yLength")
    stepperCalibration.yLimitOfThreadedRod = value.toFloat();

  if(name=="s_debounce")
    stepperCalibration.debounceCount = value.toInt();  

  saveToEEPROM();
  //getEEPROM();
}

void RoboStepper::saveToEEPROM()
{
  
  EEPROM.put(500,stepperCalibration); //500 is the memory address out of 4,000 for the RoboStepper 
  //Serial.print(stepperCalibration.stepperAccelerationPeriodMicros);
  Serial.println(" Saved to EEPROM()");  
}


void RoboStepper::getEEPROM()
{
  EEPROM.get(500,stepperCalibration); //500 is the memory address out of 4,000 for the RoboStepper 
  //Serial.print(stepperCalibration.stepperAccelerationPeriodMicros);
  Serial.println(" Got from EEPROM()"); 
}
