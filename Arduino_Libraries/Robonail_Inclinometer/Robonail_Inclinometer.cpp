
/*
  Cart.h - Library for the mechanical Cart
  Author         Date          Note
  _____________  __________    __________________________________
  Michael Baird  5/5/2014      Initial creation
*/

#include "Arduino.h"
#include "Timer.h"
#include "Encoder.h"
#include "NavigationSystem.h"
#include "RoboUtility.h"
#include "Sabertooth.h"
#include "EEPROM.h"

#define _stepHorizontalPin5V 2   // Blue  - Pul +
#define _stepHorizontalPin 3 //Red   - Pul -
#define _dirHorizontalPin5V 4    // Green - Dir +
#define _dirHorizontalPin 5 // White - Dir - 
#define _stepVerticalRightPin5V 39   // Blue  - Pul +
#define _stepVerticalRightPin 40 //Red   - Pul -
#define _dirVerticalRightPin5V 41    // Green - Dir +
#define _dirVerticalRightPin 42 // White - Dir - 
#define _stepVerticalLeftPin5V 43   // Blue  - Pul +
#define _stepVerticalLeftPin 44 //Red   - Pul -
#define _dirVerticalLeftPin5V 45    // Green - Dir +
#define _dirVerticalLeftPin 46 // White - Dir - 


#define _drillRelay1and3Pin A10   // Blue  +5v Drills 1 & 3
#define _drillRelay2and4Pin A11 //Red   Drills +5V 2 & 4
#define _drillRelay1and3NegPin A12    // Green -5V Drills 1 & 3
#define _drillRelay2and4NegPin A13 // White -5V Drills 2 & 4

// 26 //used for a 10k ohm pullup resister. the internal arduino INPUT_PULLUP get too much interference

#define _drill1LimitSwitchPin 37 // black wire #3 tells us when the drill is screwed all the way in or out.  Same pin is used for up/down. 
#define _drill2LimitSwitchPin 38 // black wire #3 tells us when the drill is screwed all the way in or out.  Same pin is used for up/down. 
#define _drill3LimitSwitchPin 33 // black wire #3 tells us when the drill is screwed all the way in or out.  Same pin is used for up/down. 
#define _drill4LimitSwitchPin 34 // black wire #3 tells us when the drill is screwed all the way in or out.  Same pin is used for up/down. 

//#define _limitSwitchHorizontalPin 19   // Black limit switch will read change when horizontal drill assembly moves all the way left or right
#define _limitSwitchHorizontalRightPin 48 //Black limit switch will read 5v when horizontal drill assembly moves all the way right 
#define _limitSwitchHorizontalLeftPin 49 //Black limit switch will read 5v when horizontal drill assembly moves all the way left 
#define _limitSwitchVerticalLeftUpPin 29 //BAD Actually connected to 31 but there is a problem with BOTH uppper limit switches //BAD //Black limit switch will read 5v when horizontal drill assembly moves all the way right 
#define _limitSwitchVerticalLeftDownPin 35 //Black limit switch will read 5v when horizontal drill assembly moves all the way left 
#define _limitSwitchVerticalRightUpPin 30 //BAD Actually connected to 31 but there is a problem with BOTH uppper limit switches //BAD //Black limit switch will read 5v when horizontal drill assembly moves all the way right 
#define _limitSwitchVerticalRightDownPin 36 //Black limit switch will read 5v when horizontal drill assembly moves all the way left 

#define _rotaryEncoderHorizontalPin 19 //interrupt 
//#define _rotaryEncoderVerticalLeftPin xx //interrupt 
//#define _rotaryEncoderVerticalRightPin xx //interrupt 

#define _pulseDelay 400 // about 200RPM, anything below 600 stalls the motor
#define _stepsPerRotation 400 //smalest the DQ860MA will allow
#define _rotations  3 //full rotations 

#define _drillRampingMaxPower  65 //95 //maximum power for Sabertooth (range 0 to 127). 100 is 19 volts, 95 is 18 volts, 127 is 24 volts
#define _drillRampingMidPower 35 //starting power for Sabertooth ramp up.
#define _drillRampingStartPower 5 //starting power for Sabertooth ramp up.
#define _drillRampingPowerIncrement 5 // every powerIncrementFrequency we will increase power by this amount. Must be a multiple to _maxPower
#define _drillRampingIncrementFrequency 60 // 200 milliseconds we increase power when we ramp
#define _drillOscillateFrequency 75 // 200 milliseconds we increase power when we ramp
#define _drillBrakePower 30 //reverse polirity to brake immediatly when we hit the limit switch
#define _drillBrakeDuration 3 //reverse polirity to brake immediatly when we hit the limit switch

//Encoder _verticalEncoder(_encoderPinA,_encoderPinB);
Timer _timer;
//RoboUtility _robo;

Sabertooth ST(128); // The Sabertooth is on address 128. We'll name its object ST.
                    // If you've set up your Sabertooth on a different address, of course change
                    // that here. For how to configure address, etc. see the DIP Switch Wizard for
                    //   Sabertooth - http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
                    //   SyRen      - http://www.dimensionengineering.com/datasheets/SyrenDIPWizard/start.htm
                    // Be sure to select Packetized Serial Mode for use with this library.
                    //
                    // On that note, you can use this library for SyRen just as easily.
                    // The diff-drive commands (drive, turn) do not work on a SyRen, of course, but it will respond correctly
                    // if you command motor 1 to do something (ST.motor(1, ...)), just like a Sabertooth.
                    //
                    // In this sample, hardware serial TX connects to S1.
                    // See the SoftwareSerial example in 3.Advanced for how to use other pins.
                            

AccelStepper stepperHorizontal(AccelStepper::DRIVER, _stepHorizontalPin, _dirHorizontalPin);
AccelStepper stepperVerticalLeft(AccelStepper::DRIVER, _stepVerticalLeftPin, _dirVerticalLeftPin);
AccelStepper stepperVerticalRight(AccelStepper::DRIVER, _stepVerticalRightPin, _dirVerticalRightPin);

int drillForwardOscillateCount=2; //we reverse and screw back in (oscillate) a couple times to ensure the screw grips the wood
int drillDirectionForwardOscillateCount[4]={0,0,0,0}; //we reverse and screw back in (oscillate) a couple times to ensure the screw grips the wood
//bool drillDirectionForward[4] = {false,false,false,false};
bool drillAtLimit[4]={true,true,true,true};//we use this to know if we are stopped at a limit.  If we are all the way down, it is ok to drill up even though
												// we are at the limit.  We set this to false once we are passed the "all the way down" limit.  

int drillPower[4] = {0,0,0,0};
bool drills1And2 = true; // On robot startup drills 1 and 2 are engaged with the relays.  We can switch to drills 3 and 4
bool drillBraking[4]= {false,false,false,false};


int counter[10] = {0,0,0,0,0,0,0,0,0,0};      // how many times we have seen new value
int reading[10] = {0,0,0,0,0,0,0,0,0,0};   // reading for each of the drills limit switches
// the following variable is a long because the time, measured in milliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was sampled
int debounce_count = 4; // number of millis/samples to consider before declaring a debounced input
int stepper_debounce_count = 4; //steppers have more chatter and needed a higher debounce count
struct EEPROMVariables {
	bool drillDirectionForward[4] = {false,false,false,false};
	bool switchingDrillSet=false; //whether we a switching drillsets 
	char activeDrillSet = '0'; //0 is for neither drillsets fastened because the system just started, 'H' is for horizontal drills 1 and 2, 'V' is for vertical drills 3 and 4

	bool isMoving = false;
	char temp = ' ';
	float xAxisPositionInInches = 0.0f;
	float yAxisPositionInInches = 0.0f;
	float xAxisDistanceFromLeftInInches = 0.0f;
	float yAxisDistanceFromBottomInInches = 0.0f;
	float xAxisDistanceLeftToMoveInches = 0.0f;
	float yAxisDistanceLeftToMoveInches = 0.0f;
	
};
float yAxisMaximumDistanceFromLimit = 13.0f;

EEPROMVariables eepromVariables;

NavigationSystem::NavigationSystem()
{
	//The constructor runs before certain arduino components are loaded including interrupts needed for the delay() and attachInterrup() functions. 
	// Thus, we added method setup() to handle constructor setup 

 
}
void NavigationSystem::setup(){
 // Sets the two pins as Outputs

  pinMode(_stepHorizontalPin5V,OUTPUT);
  pinMode(_dirHorizontalPin5V,OUTPUT);
  pinMode(_stepVerticalRightPin5V,OUTPUT);
  pinMode(_dirVerticalRightPin5V ,OUTPUT);
  pinMode(_stepVerticalLeftPin5V,OUTPUT);
  pinMode(_dirVerticalLeftPin5V,OUTPUT);

  pinMode(_drillRelay1and3Pin,OUTPUT);
  pinMode(_drillRelay2and4Pin,OUTPUT);
  pinMode(_drillRelay1and3NegPin,OUTPUT);
  pinMode(_drillRelay2and4NegPin,OUTPUT);


  //pinMode(_limitSwitchHorizontalPin,INPUT_PULLUP);
  pinMode(_limitSwitchHorizontalLeftPin,INPUT_PULLUP);
  pinMode(_limitSwitchHorizontalRightPin,INPUT_PULLUP);
  pinMode(_limitSwitchVerticalLeftUpPin,INPUT_PULLUP);
  pinMode(_limitSwitchVerticalRightUpPin,INPUT_PULLUP);
  pinMode(_limitSwitchVerticalLeftDownPin ,INPUT_PULLUP);
  pinMode(_limitSwitchVerticalRightDownPin ,INPUT_PULLUP);


  pinMode(_drill1LimitSwitchPin,INPUT_PULLUP);
  pinMode(_drill2LimitSwitchPin,INPUT_PULLUP);
  pinMode(_drill3LimitSwitchPin,INPUT_PULLUP);
  pinMode(_drill4LimitSwitchPin,INPUT_PULLUP);


  digitalWrite(_drillRelay1and3Pin,LOW);
  digitalWrite(_drillRelay2and4Pin,LOW);
  digitalWrite(_drillRelay1and3NegPin,LOW);
  digitalWrite(_drillRelay2and4NegPin,LOW);
  
  digitalWrite(_stepHorizontalPin5V,HIGH);
  digitalWrite(_dirHorizontalPin5V,HIGH);
  digitalWrite(_stepVerticalRightPin5V,HIGH);
  digitalWrite(_dirVerticalRightPin5V,HIGH);
  digitalWrite(_stepVerticalLeftPin5V,HIGH);
  digitalWrite(_dirVerticalLeftPin5V,HIGH);


	stepperHorizontal.setMaxSpeed(1000);
	stepperHorizontal.setAcceleration(1800.0);
	stepperVerticalLeft.setMaxSpeed(1000);
	stepperVerticalLeft.setAcceleration(1800.0);
	stepperVerticalRight.setMaxSpeed(1000);
	stepperVerticalRight.setAcceleration(1800.0);

	//stepperHorizontal.setMinPulseWidth(7);
 
  // open the serial port at 9600 bps:
  Serial.begin(9600);

  SabertoothTXPinSerial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
  //saveToEEPROM();
  getEEPROM();
  // See the Sabertooth 2x60 documentation for information on ramping values.
  // There are three ranges: 1-10 (Fast), 11-20 (Slow), and 21-80 (Intermediate).
  // The ramping value 14 used here sets a ramp time of 4 seconds for full
  // forward-to-full reverse.
  //
  // 0 turns off ramping. Turning off ramping requires a power cycle.
  //
  // WARNING: The Sabertooth remembers this command between restarts AND in all modes.
  // To change your Sabertooth back to its default, call ST.setRamping(0)
  ST.setRamping(0); //14);


	_timer.every(_drillRampingIncrementFrequency, rampDrills);
	_timer.every(150,finishBrakingDrill);
	//_timer.every(_drillOscillateFrequency,oscillateDrills);
	_timer.every(700, sendInfo);
  //	attachInterrupt(digitalPinToInterrupt(_limitSwitchHorizontalPin), checkLimitSwitches, CHANGE);

  //attachInterrupt(5, isr0, CHANGE); //Interrupt 5 is pin 18 (we were using interrupt 3 on pin 20 before 10/4/2014) 
  //instance0_=this; 
}


void NavigationSystem::sendInfo()
{
	//Serial.print("counter[0] =  ");
	//Serial.println(counter[0]);
	//Serial.println("sentInfo()");
}


void NavigationSystem::update()
{
	checkLimitSwitches();
	/***********************************************************************
	************************************************************************
	TODO TODO TODO TODO TODO
	TODO TODO TODO TODO TODO
	Error. for some reason the relay shorts out when actuated to drills 3/4
	there is a problem with the board. this occurs after about 8 seconds of
	being on drills 3/4. 

	WORKAROUND: Switch back to drills 1 and 2 if drills are not moving
	************************************************************************
	************************************************************************/
			
	if(drillPower[2]==0 && drillPower[3]==0 && drills1And2==false
		&&drillBraking[2]==false&&drillBraking[3]==false){ //swith to use drills 1 and 2 
		drills1And2 = true; // We are now using drills 1 and 2
		stopDrill(3);
		stopDrill(4);
		digitalWrite(_drillRelay1and3Pin,LOW); //switches drill 3 to 1
		digitalWrite(_drillRelay2and4Pin,LOW); //switches drill 4 to 2
		Serial.println("switched to drills 1 and 2");
	}

	/*
	if(digitalRead(_drill1LimitSwitchPin)==LOW) { //we are at one of the limits 
		//Serial.println("Drill 1 completed moving");
		//stopDrill(1);

	}

	if(digitalRead(_drill2LimitSwitchPin)==LOW) { //we are at one of the limits 
		Serial.println("Drill 2 completed moving");
		stopDrill(2);
	}
	*/

	_timer.update();
	moveSteppers();
	finishSwitchingDrillSet();
}
float stepperVerticalAdjustment = 0.805; //should be .80 but testing indicates it is a little short 
float stepperHorizontalAdjustment = 0.67; //should be .66666 but testing indicates it is a little short 

void NavigationSystem::move(float xAxisDistanceInches, float yAxisDistanceInches)
{

    if (eepromVariables.yAxisDistanceFromBottomInInches+yAxisDistanceInches > yAxisMaximumDistanceFromLimit){
    	Serial.println("ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR");
    	Serial.println("You are attemping to move the yAxis past the limit switch");
    	Serial.println("The vertical up limit switches are broken!! Need fixed ");
    	Serial.println("ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR");	
    }
    else
    {
	    eepromVariables.xAxisPositionInInches += xAxisDistanceInches;
	    eepromVariables.xAxisDistanceFromLeftInInches += xAxisDistanceInches;
		eepromVariables.yAxisPositionInInches += yAxisDistanceInches;
	    eepromVariables.yAxisDistanceFromBottomInInches += yAxisDistanceInches;
	    saveToEEPROM();

		float movement = 2.0*400.0*stepperHorizontalAdjustment*xAxisDistanceInches;
		if(xAxisDistanceInches>0)
			Serial.print("moving right ");
		else		
			Serial.print("moving left ");
		Serial.print(xAxisDistanceInches,3);
		Serial.print("      steps ");
		Serial.println(movement);
	    stepperHorizontal.move(movement); //(1200); //100 steps
	   

		movement = 2.0*400.0*stepperVerticalAdjustment*yAxisDistanceInches;
		if(yAxisDistanceInches>0)
			Serial.print("moving up ");
		else		
			Serial.print("moving down ");
		Serial.print(yAxisDistanceInches,3);
		Serial.print("      steps ");
		Serial.println(movement);
	    stepperVerticalLeft.move(movement); //(1200); //100 steps
	    stepperVerticalRight.move(movement); //(1200); //100 steps
	}

}
/*
void NavigationSystem::moveRight(float distanceInches)
{
    eepromVariables.xAxisPositionInInches += distanceInches;
    eepromVariables.xAxisDistanceFromLeftInInches += distanceInches;
    saveToEEPROM();

	float movement = 2.0*400.0*stepperHorizontalAdjustment*distanceInches;
	Serial.print("moving right ");
	Serial.print(distanceInches,3);
	Serial.print("      steps ");
	Serial.println(movement);
    stepperHorizontal.move(movement); //(1200); //100 steps
}
void NavigationSystem::moveLeft(float distanceInches)
{
	eepromVariables.xAxisPositionInInches -= distanceInches;
    eepromVariables.xAxisDistanceFromLeftInInches -= distanceInches;
    saveToEEPROM();
    
    float movement = -2.0*400.0*stepperHorizontalAdjustment*distanceInches;
	Serial.print("moving left ");
	Serial.print(distanceInches,3);
	Serial.print("      steps ");
	Serial.println(movement);
    stepperHorizontal.move(movement); //(-1200); //100 steps
}
void NavigationSystem::moveUp(float distanceInches)
{
	eepromVariables.yAxisPositionInInches += distanceInches;
    eepromVariables.yAxisDistanceFromBottomInInches += distanceInches;
    saveToEEPROM();
    
	float movement = 2.0*400.0*stepperVerticalAdjustment*distanceInches;
	Serial.print("moving up ");
	Serial.print(distanceInches,3);
	Serial.print("      steps ");
	Serial.println(movement);
    stepperVerticalLeft.move(movement); //(1200); //100 steps
    stepperVerticalRight.move(movement); //(1200); //100 steps
}
void NavigationSystem::moveDown(float distanceInches)
{
	eepromVariables.yAxisPositionInInches -= distanceInches;
    eepromVariables.yAxisDistanceFromBottomInInches -= distanceInches;
    saveToEEPROM();

	float movement = -2.0*400.0*stepperVerticalAdjustment*distanceInches;
	Serial.print("moving down ");
	Serial.print(distanceInches,3);
	Serial.print("      steps ");
	Serial.println(movement);
    stepperVerticalLeft.move(movement); //steps
    stepperVerticalRight.move(movement); //(1200); //100 steps
}
*/
void NavigationSystem::moveDrill(int drillNumber, int direction)
{
	//Serial.print("moving drill ");
	//Serial.print(drillNumber);
	int controllerDrillNumber = drillNumber; // Even though there are 4 drills, the motor controller only has
		// 2 controls. So we use relays to switch between drills 1&2 vs 3&4.  In other words, only two drills
		// are live at any moment.


	if (drillNumber>2 && drills1And2){ //swith to use drills 3 and 4
		controllerDrillNumber=controllerDrillNumber-2; //3 becomes 1, 4 becomes 2 for the motor controller
		drills1And2 = false; // We are now using drills 3 and 4
		stopDrill(1); // just in case they are running, we need to stop drills 1 and 2 before flipping the relay
		stopDrill(2); // just in case they are running, we need to stop drills 1 and 2 before flipping the relay
		digitalWrite(_drillRelay1and3Pin,HIGH); //switches drill 1 to 3
		digitalWrite(_drillRelay2and4Pin,HIGH); //switches drill 2 to 4
		Serial.println("switched to drills 3 and 4");
	}
	else if(drillNumber<=2 && drills1And2==false){ //swith to use drills 1 and 2 
		drills1And2 = true; // We are now using drills 1 and 2
		stopDrill(3); // just in case they are running, we need to stop drills 3 and 4 before flipping the relay
		stopDrill(4); // just in case they are running, we need to stop drills 3 and 4 before flipping the relay
		digitalWrite(_drillRelay1and3Pin,LOW); //switches drill 3 to 1
		digitalWrite(_drillRelay2and4Pin,LOW); //switches drill 4 to 2
		Serial.println("switched to drills 1 and 2");
	}

	// See the Sabertooth 2x60 documentation for information on ramping values.
	// There are three ranges: 1-10 (Fast), 11-20 (Slow), and 21-80 (Intermediate).
	// The ramping value 14 used here sets a ramp time of 4 seconds for full
	// forward-to-full reverse.
	//
	// 0 turns off ramping. Turning off ramping requires a power cycle.
	//
	// WARNING: The Sabertooth remembers this command between restarts AND in all modes.
	// To change your Sabertooth back to its default, call ST.setRamping(0)
	//ST.setRamping(0); //14);

	switch(direction){
      case -1: //move down
      	//eepromVariables.drillDirectionForward[drillNumber-1]=false;
        Serial.println(" reverse");
        eepromVariables.drillDirectionForward[drillNumber-1] = false;
    	drillPower[drillNumber-1] = -_drillRampingStartPower;
    	ST.motor(controllerDrillNumber, -_drillRampingStartPower);

        if(!eepromVariables.switchingDrillSet) // if we are switching drill sets it will be saves by another method
        	saveToEEPROM();
        //ST.motor(controllerDrillNumber,-_drillRampingStartPower); //0 to 127
        break;
      case 1: //move up
      	//eepromVariables.drillDirectionForward[drillNumber-1]=true;
        Serial.println(" forward");
        eepromVariables.drillDirectionForward[drillNumber-1] = true;
    	drillPower[drillNumber-1] = _drillRampingStartPower;
    	ST.motor(controllerDrillNumber, _drillRampingStartPower);
    	drillDirectionForwardOscillateCount[drillNumber-1]=1;
        if(!eepromVariables.switchingDrillSet) // if we are switching drill sets it will be saves by another method
        	saveToEEPROM();
        //ST.motor(controllerDrillNumber,_drillRampingStartPower); //0 to 127
        break;
      default: // by default, the system will move in the opposite direction 
      		// in other words. if the drill is screwed in, it will default to screwing out
      		//   if the drill is screwed out (unfastened), it will move in the opposite
        eepromVariables.drillDirectionForward[drillNumber-1] = !eepromVariables.drillDirectionForward[drillNumber-1];


      		//   direction and screw in
    	if(eepromVariables.drillDirectionForward[drillNumber-1]){
    		Serial.println(" default mode forward=true");
        	drillPower[drillNumber-1] = _drillRampingStartPower;
        	ST.motor(controllerDrillNumber, _drillRampingStartPower);
    		drillDirectionForwardOscillateCount[drillNumber-1]=1;
        }
    	else{
    		Serial.println(" default mode forward=false");
        	drillPower[drillNumber-1] = -_drillRampingStartPower;
        	ST.motor(controllerDrillNumber, -_drillRampingStartPower);
    	}

        if(!eepromVariables.switchingDrillSet) // if we are switching drill sets it will be saves by another method
        	saveToEEPROM();
        break;
    }
}

void NavigationSystem::switchDrillSet(char drillSet){
	eepromVariables.switchingDrillSet=true; 
	if(drillSet=='H'||drillSet=='V'){
		eepromVariables.activeDrillSet=drillSet; 
	}
	else if(eepromVariables.activeDrillSet=='H'){
		eepromVariables.activeDrillSet='V'; 
	}
	else{
		eepromVariables.activeDrillSet='H'; 
	}

	Serial.print("Got to here 3!");
	Serial.println(eepromVariables.activeDrillSet);

	if(eepromVariables.activeDrillSet=='H'){
		if(!eepromVariables.drillDirectionForward[0])
			moveDrill(1,1);
		if(!eepromVariables.drillDirectionForward[1])
			moveDrill(2,1);
	} else if(eepromVariables.activeDrillSet=='V'){
		if(!eepromVariables.drillDirectionForward[2])
			moveDrill(3,1);
		if(!eepromVariables.drillDirectionForward[3])
			moveDrill(4,1);
	}
	saveToEEPROM();
}
void NavigationSystem::finishSwitchingDrillSet(){
	if(eepromVariables.switchingDrillSet){
		if(eepromVariables.activeDrillSet=='H')	{
			if(drillPower[0]==0 && drillPower[1]==0 
				&&drillBraking[0]==false&&drillBraking[1]==false){ 
				//Horizontal Drills 1&2 are fastened
				if(eepromVariables.drillDirectionForward[2])
					moveDrill(3,-1);
				if(eepromVariables.drillDirectionForward[3])
					moveDrill(4,-1);
				eepromVariables.switchingDrillSet=false; //We are done switching the drillSet
				saveToEEPROM();
				Serial.println("Finished switching to Horizontal DrillSet");
			}
		} else if(eepromVariables.activeDrillSet=='V'){
			if(drillPower[2]==0 && drillPower[3]==0 
				&&drillBraking[2]==false&&drillBraking[3]==false){ 
				//Vertical Drills 3&4 are fastened
				if(eepromVariables.drillDirectionForward[0])
					moveDrill(1,-1);
				if(eepromVariables.drillDirectionForward[1])
					moveDrill(2,-1);
				eepromVariables.switchingDrillSet=false; //We are done switching the drillSet
				saveToEEPROM();
				Serial.println("Finished switching to Vertical DrillSet");
			}
		}
	}
}
void NavigationSystem::rampDrills()
{
	for(int i=0;i<4;i++){
		//Temporary code. we stop the drill immediatly after hitting max power
		/*if (drillPower[i]==_drillRampingMaxPower || drillPower[i]==-_drillRampingMaxPower)
		{
			drillPower[i]=0; //stopDrill(i+1);
			
			if(i==0 || i==2) //motor controller 1
				ST.motor(1,drillPower[i]);
			else  //motor controller 2 
				ST.motor(2,drillPower[i]);
		}*/
		if(drillBraking[i]==false){
			if ((drillPower[i]>0 && drillPower[i]<_drillRampingMaxPower) ||
				(drillPower[i]<0 && drillPower[i]>-_drillRampingMaxPower))
			{

				if(drillPower[i]>0)
					drillPower[i]=drillPower[i]+_drillRampingPowerIncrement; //increase the power by 20
				else
					drillPower[i]=drillPower[i]-_drillRampingPowerIncrement; //reverse the power increase by 20 
				
				if(drillPower[i]>_drillRampingMaxPower)
					drillPower[i]=_drillRampingMaxPower;


				if(drillPower[i]<-_drillRampingMaxPower)
					drillPower[i]=-_drillRampingMaxPower;
				
				/*Serial.print("finishing i=");
				Serial.print(i);
				Serial.print(" , drillPower[i]=");
				Serial.println(drillPower[i]);*/
				
				if(i==0 || i==2) //motor controller 1
					ST.motor(1,drillPower[i]);
				else  //motor controller 2 
					ST.motor(2,drillPower[i]);
			}
		}

	}
}

void NavigationSystem::stopDrill(int drillNumber)
{

	int controllerDrillNumber = drillNumber; // Even though there are 4 drills, the motor controller only has
		// 2 controls. So we use relays to switch between drills 1&2 vs 3&4.  In other words, only two drills
		// are live at any moment.

	//ST.setRamping(0); // any value greater than 0 will cause the sabertooth to gradually stop

	if (drillNumber>2) //use drills 3 and 4
		controllerDrillNumber=controllerDrillNumber-2; //3 becomes 1, 4 becomes 2 for the motor controller

	//THis code is to immediatly stop the drill. i reverse polarity for 1 millisecond 
    //ST.motor(controllerDrillNumber,0);
    //ST.stop();

    if(drillBraking[drillNumber-1]==false){
    	drillBraking[drillNumber-1]=true;
	    Serial.print("Brake applied to drill ");
	    Serial.print(drillNumber);
	    Serial.print(".  Drill power=");
	    Serial.println(drillPower[drillNumber-1]);		

	    if(drillPower[drillNumber-1]>0){
	    	ST.motor(controllerDrillNumber,-_drillBrakePower);
    		drillPower[drillNumber-1] = -_drillBrakePower;		    		
	    }
	    else if(drillPower[drillNumber-1]<0){
	    	ST.motor(controllerDrillNumber,_drillBrakePower);
    		drillPower[drillNumber-1] = _drillBrakePower;
    	}
    	else
	    	ST.motor(controllerDrillNumber,0);    		

	    _timer.after(_drillBrakeDuration,finishBrakingDrill);
    }
    
    //ST.motor(controllerDrillNumber,0);
    
    //drillPower[drillNumber-1] = 0;
}

bool isDrillOscillatingBackwards[4]= {false,false,false,false};
void NavigationSystem::finishBrakingDrill()
{
	for(int i=0;i<4;i++){		
    	if(drillBraking[i]==true){
    		int controllerDrillNumber = i+1; // Even though there are 4 drills, the motor controller only has
			if (i>2) //use drills 3 and 4
				controllerDrillNumber=controllerDrillNumber-2; //3 becomes 1, 4 becomes 2 for the motor controller

    		drillBraking[i]=false;
    		ST.motor(controllerDrillNumber,0);
    		drillPower[i] = 0;		    		
		    Serial.print("Finished braking drill is now fully stopped ");
		    Serial.println(i+1);

		    if(eepromVariables.drillDirectionForward[i]){
		    	if(isDrillOscillatingBackwards[i]){		    
		    		isDrillOscillatingBackwards[i]=false;		
			    	drillPower[i] = _drillRampingStartPower;
			    	ST.motor(controllerDrillNumber, _drillRampingStartPower);
				    //Serial.println("Drill oscillate returning to forward "); 
		    	}
		    	if(drillDirectionForwardOscillateCount[i]<drillForwardOscillateCount){
			    	//we are moving forward so we unscrew/rescrew to ensure we are fastened to the wood
			    	drillDirectionForwardOscillateCount[i]++;
			    	drillPower[i] = -_drillRampingStartPower;
			    	isDrillOscillatingBackwards[i]=true;

			    	ST.motor(controllerDrillNumber, -_drillRampingStartPower);
				   // Serial.print("Drill oscillate forward count = ");
				    //Serial.println(drillDirectionForwardOscillateCount[i]);
			    }
		    } 

    	}
	}
}
void NavigationSystem::checkLimitSwitches()
{
	/*
	if(digitalRead(_limitSwitchHorizontalLeftPin)==LOW)  //we are at one of the limits 
		Serial.println("all the way left"); // cannot call Serial from an interrupt
	if(digitalRead(_limitSwitchHorizontalRightPin)==LOW)  //we are at one of the limits 
		Serial.println("all the way right"); // cannot call Serial from an interrupt
	if(digitalRead(_limitSwitchVerticalLeftUpPin)==LOW)  //we are at one of the limits 
		Serial.println("all the way up (left side)"); // cannot call Serial from an interrupt
	if(digitalRead(_limitSwitchVerticalLeftDownPin)==LOW)  //we are at one of the limits 
		Serial.println("all the way down (left side)"); // cannot call Serial from an interrupt
	if(digitalRead(_limitSwitchVerticalRightUpPin)==LOW)  //we are at one of the limits 
		Serial.println("all the way up (right side)"); // cannot call Serial from an interrupt
	if(digitalRead(_limitSwitchVerticalRightDownPin)==LOW) //we are at one of the limits 
		Serial.println("all the way down (right side)"); // cannot call Serial from an interrupt
	*/

	// If we have gone on to the next millisecond
	if(millis() != time)
	{
		reading[0] = digitalRead(_drill1LimitSwitchPin);
		reading[1] = digitalRead(_drill2LimitSwitchPin);
		reading[2] = digitalRead(_drill3LimitSwitchPin);
		reading[3] = digitalRead(_drill4LimitSwitchPin);
		reading[4] = digitalRead(_limitSwitchHorizontalLeftPin);
		reading[5] = digitalRead(_limitSwitchHorizontalRightPin);
		reading[6] = digitalRead(_limitSwitchVerticalLeftUpPin);
		reading[7] = digitalRead(_limitSwitchVerticalLeftDownPin);
		reading[8] = digitalRead(_limitSwitchVerticalRightUpPin);
		reading[9] = digitalRead(_limitSwitchVerticalRightDownPin);
		for (int i=0;i<10;i++){
			/*if (drillPower[i]== _drillRampingMaxPower || 
				drillPower[i]== -_drillRampingMaxPower)
			{*/ //We only stop the drill if it is fully ramped. This is because when we are all the way down (or up) we are at the limit switch.
				// Since we are at the limit switch when starting, we allow the ramp time to get past the limit.  Once we are done ramping,
				// the drill needs to be past the limit switch otherwise it will stop immediatly thinking it reached the other limit. 
			if(reading[i] == HIGH)
			{
				//if(counter[i]>-debounce_count)
			  	//	counter[i]--;

				if(counter[i]>-debounce_count && i <4)
					counter[i]--;

				if(counter[i]>-stepper_debounce_count && i >=4)
					counter[i]--;

			  	if(i<4){ //Drill limit switches are 0 through 3
			  		if(drillAtLimit[i] && counter[i]<=-debounce_count){
						Serial.print("Drill ");
						Serial.print(i+1);
						Serial.println(" is past the limit!");
						drillAtLimit[i]=false;
						counter[i] = 0;
					}
			  	}
				
			}
			
			if(reading[i] ==LOW)
			{
				if(counter[i]<debounce_count && i <4)
					counter[i]++;

				if(counter[i]<stepper_debounce_count && i >=4)
					counter[i]++;

			  	if(i<4){ //Drill limit switches are 0 through 3
					// If the Input has shown the same value for long enough let's switch it
					if(drillAtLimit[i]==false && counter[i] >= debounce_count) 
					{
					  //We only stop the drill if it is in motion past the limit switch 
					  stopDrill(i+1);
					  Serial.print("Drill ");
					  Serial.print(i+1);
					  Serial.println(" stopped at limit");
					  drillAtLimit[i]=true;
					  counter[i] = 0; 
					}
				}

				if(i>=4){ //Steppers are 4 through 9
					if(counter[i] >= stepper_debounce_count) { //we are at one of the limits 
						
						Serial.print("Stepper at limit i = ");
						Serial.print(i);
						Serial.print("counter = ");
						Serial.println(counter[i]);

						counter[i] = 0;

						switch(i){
					      case 4:
							if(stepperHorizontal.distanceToGo()<0){ // we are moving left 
								float distanceRemainingInINches = distanceLeftToMoveXAxis(); //stepperHorizontal.distanceToGo()/(2.0f*400.0f*stepperHorizontalAdjustment);
								eepromVariables.xAxisPositionInInches -= distanceRemainingInINches;
							    eepromVariables.xAxisDistanceFromLeftInInches=0.0f;
							    saveToEEPROM();

								Serial.println("all the way left"); // cannot call Serial from an interrupt
								stepperHorizontal.stop();
								stepperHorizontal.setCurrentPosition(0);
								stepperHorizontal.move(160);
							}
					        break;
					      case 5:
							if(stepperHorizontal.distanceToGo()>0){ // we are moving right 
								float distanceRemainingInINches = distanceLeftToMoveXAxis(); //stepperHorizontal.distanceToGo()/(2.0f*400.0f*stepperHorizontalAdjustment);
								eepromVariables.xAxisPositionInInches -= distanceRemainingInINches;
							    eepromVariables.xAxisDistanceFromLeftInInches-=distanceRemainingInINches;
							    saveToEEPROM();

								Serial.println("all the way right"); // cannot call Serial from an interrupt
								stepperHorizontal.stop();
								stepperHorizontal.setCurrentPosition(0);
								stepperHorizontal.move(-60);
							}
					        break;
					      /* Not connected
					      case 6:
							if(stepperVerticalLeft.distanceToGo()>0){ // we are moving up 
								float distanceRemainingInINches = distanceLeftToMoveYAxis(); //stepperVerticalLeft.distanceToGo()/(2.0f*400.0f*stepperVerticalAdjustment);
								eepromVariables.yAxisPositionInInches -= distanceRemainingInINches;
							    eepromVariables.yAxisDistanceFromBottomInInches-=distanceRemainingInINches;
							    saveToEEPROM();

								Serial.println("all the way up (left side)"); // cannot call Serial from an interrupt
								stepperVerticalLeft.stop(); 
								stepperVerticalLeft.setCurrentPosition(0);
							}
					        break;*/
					      case 7:
							if(stepperVerticalLeft.distanceToGo()<0){ // we are moving down 
								float distanceRemainingInINches = distanceLeftToMoveYAxis(); //stepperVerticalLeft.distanceToGo()/(2.0f*400.0f*stepperVerticalAdjustment);
								eepromVariables.yAxisPositionInInches -= distanceRemainingInINches;
							    eepromVariables.yAxisDistanceFromBottomInInches=0.0f;
							    saveToEEPROM();
								Serial.println("all the way down (left side)"); // cannot call Serial from an interrupt
								stepperVerticalLeft.stop(); 
								stepperVerticalLeft.setCurrentPosition(0);
								stepperVerticalLeft.move(60); //we need to step away from the limit switch
							}
					        break;
					      /* Not connected
					      case 8:
							if(stepperVerticalRight.distanceToGo()>0){ // we are moving up 
								Serial.println("all the way up (right side)"); // cannot call Serial from an interrupt
								stepperVerticalRight.stop(); 
								stepperVerticalRight.setCurrentPosition(0);
							}
					        break;*/
					      case 9:
							if(stepperVerticalRight.distanceToGo()<0){ // we are moving down 
								Serial.println("all the way down (right side)"); // cannot call Serial from an interrupt
								stepperVerticalRight.stop(); 
								stepperVerticalRight.setCurrentPosition(0);
								stepperVerticalRight.move(60); //we need to step away from the limit switch
							}
					        break;
					    }
					}
				}


			}
			//}
		}
		time = millis();
	}
}
void NavigationSystem::stop()
{
	Serial.println("navigation system stopped");
	stepperHorizontal.stop();
	stepperHorizontal.setCurrentPosition(0);
	stepperVerticalLeft.stop();
	stepperVerticalLeft.setCurrentPosition(0);
	stepperVerticalRight.stop();
	stepperVerticalRight.setCurrentPosition(0);
	stopDrill(1);
	stopDrill(2);
	stopDrill(3);
	stopDrill(4);
    saveToEEPROM();
}
void NavigationSystem::moveSteppers()
{

	if(stepperHorizontal.distanceToGo()!=0) // we are moving left/right
		stepperHorizontal.run();
	if(stepperVerticalLeft.distanceToGo()!=0) // we are moving Up/down			
		stepperVerticalLeft.run();
	if(stepperVerticalRight.distanceToGo()!=0) // we are moving up/down 			
		stepperVerticalRight.run();
}


float NavigationSystem::distanceLeftToMoveXAxis(){
	return stepperHorizontal.distanceToGo()/(2.0f*400.0f*stepperHorizontalAdjustment);
}
float NavigationSystem::distanceLeftToMoveYAxis(){
	return stepperVerticalLeft.distanceToGo()/(2.0f*400.0f*stepperVerticalAdjustment);
}
bool NavigationSystem::isMoving()
{
	
	if(stepperHorizontal.distanceToGo()!=0) // we are moving left/right
		return true;
	if(stepperVerticalLeft.distanceToGo()!=0) // we are moving Up/down	
		return true;
	if(stepperVerticalRight.distanceToGo()!=0) // we are moving up/down 
		return true;	
	if(drillPower[0]!=0 || drillPower[1]!=0 || drillPower[2]!=0 || drillPower[3]!=0) //drills are moving
		return true;
	if(eepromVariables.switchingDrillSet)
		return true;
	return false;
}


void NavigationSystem::saveToEEPROM()
{
	eepromVariables.isMoving=isMoving();
	eepromVariables.xAxisDistanceLeftToMoveInches = distanceLeftToMoveXAxis();
	eepromVariables.yAxisDistanceLeftToMoveInches = distanceLeftToMoveYAxis();
	EEPROM.put(0,eepromVariables);

	Serial.println("Saved to EEPROM()");

	Serial.print("xAxis:");
	Serial.print(eepromVariables.xAxisPositionInInches);
	Serial.print(" yAxis:");
	Serial.print(eepromVariables.yAxisPositionInInches);
	Serial.print(" xAxisFromLimit:");
	Serial.print(eepromVariables.xAxisDistanceFromLeftInInches);
	Serial.print(" yAxisFromLimit:");
	Serial.println(eepromVariables.yAxisDistanceFromBottomInInches);


	Serial.print("Drills fastened? DrillSet (1,2,3,4) : ");
	Serial.print(eepromVariables.activeDrillSet);
	Serial.print(eepromVariables.drillDirectionForward[0]);
	Serial.print(eepromVariables.drillDirectionForward[1]);
	Serial.print(eepromVariables.drillDirectionForward[2]);
	Serial.println(eepromVariables.drillDirectionForward[3]);

}


void NavigationSystem::getEEPROM()
{

	EEPROM.get(0,eepromVariables);

	Serial.println("Got from EEPROM()");

	Serial.print("xAxis:");
	Serial.print(eepromVariables.xAxisPositionInInches);
	Serial.print(" yAxis:");
	Serial.print(eepromVariables.yAxisPositionInInches);
	Serial.print(" xAxisFromLimit:");
	Serial.print(eepromVariables.xAxisDistanceFromLeftInInches);
	Serial.print(" yAxisFromLimit:");
	Serial.println(eepromVariables.yAxisDistanceFromBottomInInches);

	Serial.print("Drills fastened? DrillSet (1,2,3,4) : ");
	Serial.print(eepromVariables.activeDrillSet);
	Serial.print(eepromVariables.drillDirectionForward[0]);
	Serial.print(eepromVariables.drillDirectionForward[1]);
	Serial.print(eepromVariables.drillDirectionForward[2]);
	Serial.println(eepromVariables.drillDirectionForward[3]);

}



