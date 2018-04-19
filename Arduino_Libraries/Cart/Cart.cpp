
/*
  Cart.h - Library for the mechanical Cart
  Author         Date          Note
  _____________  __________    __________________________________
  Michael Baird  5/5/2014      Initial creation
*/

#include "Arduino.h"
#include "Timer.h"
#include "Encoder.h"
#include "Cart.h"
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


#define _speedPWM 5                    //Green
#define _startStop 6                       //Black
#define _runBrake 7                         //White
#define _clockwiseCounter 8                 //Gray
//#define _alarmReset 7                       //Purple
//#define _alarmOutput 9                      //Brown
//#define _intVRext 10                        //LightBlue
#define _speedOutput 18                     //Red (was on pin 20 before 10/4/2014)


volatile long _verticalPulses; //Store pulses of the rotary encoder 
volatile long _horizontalPulses; //Store pulses of the rotary encoder
float currentDistanceVertical=0.0; //Measures the horizontal distance 
float currentDistanceHorizontal=0.0; //Measures the vertical distance
float targetDistanceVertical=0.00; //The target position of the machine from the start position
float targetDistanceHorizontal=0.00;//The target position of the machine from the start position
int speedPWM = 255; //value of 0 to 255 for speed of trolley

boolean left = false;
//Rotary Encoder to measure vertical movement
Encoder _verticalEncoder(_encoderPinA,_encoderPinB);
Timer _timer;
RoboUtility _robo;

Cart::Cart()
{
	//The constructor runs before certain arduino components are loaded including interrupts needed for the delay() and attachInterrup() functions. Thus, we added method begin() to handle constructor setup 
	
}
void Cart::begin(){

  //set up the side wheel linear actuators/relays
	_verticalEncoder.write(0);
  pinMode(_cartSideWheelsUp, OUTPUT);
  pinMode(_cartSideWheelsDown, OUTPUT); 
  pinMode(_cartSideWheelsDisabled, OUTPUT);  
  digitalWrite(_cartSideWheelsDisabled,HIGH); 
  
  
  //Set up nail guns
  
  pinMode(_gun1, OUTPUT);
  pinMode(_gun2, OUTPUT); 
  pinMode(_gun3, OUTPUT);
  pinMode(_gunsDisabled, OUTPUT);  
  digitalWrite(_gunsDisabled,LOW); //High disables the guns
  
  //set up the motor/chain to move horizontally
  //Inputs from motor/chain  
  pinMode(_startStop, OUTPUT);  
  digitalWrite(_startStop,LOW); // 6 Black  
  //delay(10); NOTE: NEVER EVERE put a delay in a constructor. it causes the whole arduino to stop. see: http://forum.arduino.cc/index.php/topic,46517.0.html
  pinMode(_speedPWM, OUTPUT);  
  //digitalWrite(_speedPWM,LOW); // 8 Gray
  analogWrite(_speedPWM,255);
  pinMode(_clockwiseCounter, OUTPUT);  
  pinMode(_runBrake, OUTPUT);  
  digitalWrite(_runBrake,HIGH); // 7 White 
  
  //interrupt using the internal pullup resistor for horizontal movement
  pinMode(_speedOutput, INPUT_PULLUP); // 20 Red
  attachInterrupt(5, isr0, CHANGE); //Interrupt 5 is pin 18 (we were using interrupt 3 on pin 20 before 10/4/2014) 
  instance0_=this;
 //attachMyInterrupt();
}

void Cart::update()
{
	_timer.update();
	calculateCurrentDistanceAndStopAtTarget();
}

long Cart::readVerticalPulses()
{
	_verticalPulses = _verticalEncoder.read();
	return _verticalPulses;
}

long Cart::readHorizontalPulses()
{
	return _horizontalPulses;
}

float Cart::readVerticalInches()
{
	/*(_verticalPulses = _verticalEncoder.read();
  float inchesPerPulse=0.0026327978; // measured this
  float distance = inchesPerPulse*(float)_verticalPulses;
  return distance;*/
  calculateDistance();
  return currentDistanceVertical;
}

float Cart::readHorizontalInches()
{
  /*float inchesPerPulse=0.0046887210; // measured this
  float distance = inchesPerPulse*(float)_horizontalPulses;
  return distance;*/
  calculateDistance();
  return currentDistanceHorizontal;
}
void Cart::calculateDistance()
{
	//first calculate vertical
	_verticalPulses = _verticalEncoder.read();
	float inchesPerPulse=0.0026327978; // measured this
	currentDistanceVertical = inchesPerPulse*(float)_verticalPulses;
	
	//now calculate horizontal
	inchesPerPulse=0.0046887210; // measured this
	currentDistanceHorizontal = inchesPerPulse*(float)_horizontalPulses;
}


//used to Glue to instance of class when calling an attachInterrupt ISR within a library
void Cart::isr0(){
  instance0_->speedOutput(); 
}

// for use by ISR glue routines
Cart * Cart::instance0_;

void Cart::speedOutput()
{
	if(left){
	_horizontalPulses-=1;
	}
	else{
	_horizontalPulses+=1;
	}
}


void Cart::horizontalLeft (String distanceToMove){
	float moveInches = _robo.stringToFloat(distanceToMove);
	Serial.print("Running horizontal wheels left inches:");
	Serial.println(moveInches);
	left = true;
	digitalWrite(_clockwiseCounter,LOW); // 5 Green
	//_timer.pulseImmediate(_speedPWM, moveInches, HIGH);  
	targetDistanceHorizontal=targetDistanceHorizontal-moveInches;
	//_timer.pulseImmediate(_runBrake, moveInches, LOW);
	digitalWrite(_runBrake,LOW); //Should this be high?
	speedPWM=255;
	analogWrite(_speedPWM,speedPWM); 
	//Serial.println("Moving left");
}


void Cart::horizontalRight (String distanceToMove){
	float moveInches = _robo.stringToFloat(distanceToMove);
	Serial.print("Running horizontal wheels right inches:");
	Serial.println(moveInches);
	left = false;
	targetDistanceHorizontal=targetDistanceHorizontal+moveInches;
	digitalWrite(_clockwiseCounter,HIGH); // 5 Green
	//_timer.pulseImmediate(_speedPWM, moveInches, HIGH);
	digitalWrite(_runBrake,LOW); //Should this be high?
	//_timer.pulseImmediate(_runBrake, moveInches, LOW);
	speedPWM=255;
	analogWrite(_speedPWM,speedPWM); 
}

void Cart:: setSpeed(String speed){ //input parameter is a value between 1 and 100
      int speedPWM = (int) ((speed.toInt()) * (255/100.0));
      analogWrite(_speedPWM,speedPWM);  //Green PWM Speed (VRM) 
}

void Cart::sideWheelsUp (String time){
  float delayMilliseconds = _robo.stringToFloat(time)*1000;
  Serial.print("Running side wheels for:");
  Serial.println(delayMilliseconds); 
  _timer.pulseImmediate(_cartSideWheelsDisabled, delayMilliseconds, LOW);
}
	
void Cart::sideWheelsDown (String time){
  float delayMilliseconds = _robo.stringToFloat(time)*1000;
  Serial.print("Running side wheels for:");
  Serial.println(delayMilliseconds);
  _timer.pulseImmediate(_cartSideWheelsDown, delayMilliseconds, HIGH);
  _timer.pulseImmediate(_cartSideWheelsUp, delayMilliseconds, HIGH);
  _timer.pulseImmediate(_cartSideWheelsDisabled, delayMilliseconds, LOW);
}

/* 
  This routine stops the cart from moving
*/
void Cart::stopMoving(){
  digitalWrite(_runBrake,HIGH); //White Run/Brake
}
/*This routine calculates how far the cart has moved horizontally based on the
  number of pulses from the motor encoder.
  */
void Cart::calculateCurrentDistanceAndStopAtTarget(){
  //currentDistanceHorizontal=
  calculateDistance();
  float stopTolerance=0.05; //the distance we can begin stopping
  Serial.print("currentDistanceHorizontal=");
  Serial.print(currentDistanceHorizontal);
  Serial.print(" targetDistanceHorizontal=");
  Serial.print(targetDistanceHorizontal);
  Serial.print(" speedPWM=");
  Serial.println(speedPWM);
  if(left && currentDistanceHorizontal < (targetDistanceHorizontal+stopTolerance)) //we are moving left and past our target position
  {
    digitalWrite(_runBrake,HIGH); //White Run/Brake
    //TODO: Once the vertical motion works, call stop moving 
    //  currently stopMoving() will end the scissor because we are at the target distance
  }
  if(!left && currentDistanceHorizontal > (targetDistanceHorizontal-stopTolerance)) //we are moving right and past our target position
  {
    digitalWrite(_runBrake,HIGH); //White Run/Brake
    //TODO: Once the vertical motion works, call stop moving 
    //  currently stopMoving() will end the scissor because we are at the target distance
  }
  if(abs(currentDistanceHorizontal - targetDistanceHorizontal) <4.0 && speedPWM > 75 ) //we are within two inches of the target so we slow down
  {
	speedPWM=50;
    analogWrite(_speedPWM,speedPWM);  //Green PWM Speed (VRM) 
  }
  /*else
  {    
    analogWrite(_speedPWM,speedPWM);  //Green PWM Speed (VRM) 
  }*/
  
}

void Cart::fireNailGun (String gunNumber){
  float delayMilliseconds = 200;
  Serial.print("Firing Gun #"+gunNumber+"\n");
  gunNumber.trim();
  if(gunNumber.equals("1"))
	_timer.pulseImmediate(_gun1, delayMilliseconds, HIGH);
  if(gunNumber=="2")
	_timer.pulseImmediate(_gun2, delayMilliseconds, HIGH);
  if(gunNumber=="3")
	_timer.pulseImmediate(_gun3, delayMilliseconds, HIGH);
	
  //_timer.pulseImmediate(_gunsDisabled, delayMilliseconds, LOW);
}


