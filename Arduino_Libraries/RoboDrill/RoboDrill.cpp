
/*
  RoboDrill.h - Library for the controlling the 4 dewalt impact
  drills that fasten to the roof. 

  Author         Date          Note
  _____________  __________    __________________________________
  Michael Baird  12/31/2017      Initial creation
*/


#include "Arduino.h" 
#include "RoboDrill.h"
#include "RoboUtility.h"
#include "EEPROM.h"
#include "Timer.h"   //timer library
 
/*
const int _drill1_IR = A0;
const int _drill1_Limit = A1;
const int _drill1_Dir = 11;
const int _drill1_PWM = 9;

const int _drill2_IR = A2;
const int _drill2_Limit = A3;
const int _drill2_Dir = 5;
const int _drill2_PWM = 7;

const int _drill3_IR = -1;
const int _drill3_Limit = -1;
const int _drill3_Dir = 10;
const int _drill3_PWM = 8;

const int _drill4_IR = -1;
const int _drill4_Limit = -1;
const int _drill4_Dir = 4;
const int _drill4_PWM = 6;
*/

#define DRILL_1 0
#define DRILL_2 1
#define DRILL_3 2
#define DRILL_4 3

#define DIRECTION_DOWN 0
#define DIRECTION_UP 1


const int _drill_IR_Sensor[4]=  {A1,A3,A4,A5}; //{A0,A2,A4,A5};  // infared sensor from Shart Sharp-IR-2cm-to-15cm-gp2y0a51sk_e
const int _drill_Down_Limit_Switch[4] =  {A0,A2,34,39}; //{A1,A3,34,39}; //limit switch to know when drill is screwed all the way down
const int _drill_Up_Limit_Switch[4] = {1,0,33,38}; //limit switch to know when drill is screwed all the way up
const int _drill_Dir[4] = {11,5,10,4}; //the drill direction 
const int _drill_Speed_PWM[4] = {9,7,8,6}; // the pulse width modulation (PWM) pins to constrol the drills speed


bool isDrillMoving[4] = {false,false,false,false};
unsigned long drillStartedMovingMillis[4]= {0,0,0,0}; 
unsigned long drillStoppedMovingMillis[4]= {0,0,0,0}; 
unsigned long drillCurrentSpeed[4]= {0,0,0,0}; 
int drillDirection[4] = {DIRECTION_UP, DIRECTION_UP, DIRECTION_UP, DIRECTION_UP}; //start in the up position
unsigned long lastDrillRampMillis = 0;

// we debounce the limit switches to avoid flickering/interference from 
//  triggering them incorrectly. 
unsigned long lastDebounceTimeDrill[4]= {0,0,0,0};  // the last time we read the switch
int debounceCounterDown[4]= {0,0,0,0};  // count the milliseconds for each reading of teh down limit switch
int debounceCounterUp[4]= {0,0,0,0};  // count the milliseconds for each reading of the up limit switch

bool atDownLimitSwitchDrill[4] = {false,false,false,false}; //whether the drill is at limit switch (screwed all the way down)
bool atUpLimitSwitchDrill[4] = {false,false,false,false}; //whether the drill is at limit switch (screwed all the way up)

//unsigned long atLimitSwitchDrillCount[4] = {0,0,0,0};  // count the number of times we hit the down limit switch 


unsigned long lastIRSensorReadingMillis = 0; 
int drillDistanceIR[4] = {0,0,0,0};

struct DrillCalibration { 
  int drillStartSpeed=10; //they are 18v to 20v drills so we never want to give the full 255 (24v power)
  int drillMaxSpeed=180; //185 is about 17.4 volots. they are 18v to 20v drills so we never want to give the full 255 (24v power)
  int drillRampPeriodMillis=250; // milliseconds to get to max speed 
  int drillMoveMaxTimeMillis=3000; //always stop the drill after x millis to prevent it from going crazy and just screwing forever
  int debounceCount = 5;    // the debounce time counter in millis(); increase if the noise/electro-magnetic-interference occurs with the limit switches
  
  int drillMoveUpPauseAfterMillis = 450;
  int drillMoveUpPauseDurationMillis = 70;
  int drillMoveUpAfterPauseSpeed = 100;

  int irSensorFrequencyMillis = 4; //The sensor only changes value every 16 millis. I cut that in 1/2 because we don't know WHEN it changed value. so maximum delay is 16ms + 8ms = 24ms
  int targetIRDrillDownDistance = 155; //55; //205;  //Down is 210 to 230 
  int targetIRDrillUpDistance[4] = {70, 80, 75, 75}; //150;  //Up is 120 to 140
  float irSensorSmoothingFactor = 0.9f; //meaning we keep 90% of the previous readings to reduce noise/chatter 
  //int restartDrillTolerance = 20; //if we are within 20, we won't restart
};

DrillCalibration drillCalibration;

Timer drillTimer;

RoboUtility utility;

RoboDrill::RoboDrill()
{
//The constructor runs before certain arduino components are loaded including interrupts needed for the delay() and attachInterrup() functions.
// Thus, we added method setup() to handle constructor setup 
}
void RoboDrill::setup()
{  

  for(int drill=0; drill<4; drill++){ //loop through all 4 drills  
    pinMode(_drill_Down_Limit_Switch[drill], INPUT_PULLUP);
    pinMode(_drill_Up_Limit_Switch[drill], INPUT_PULLUP);
    pinMode(_drill_IR_Sensor[drill],INPUT);
    pinMode(_drill_Dir[drill],OUTPUT);
    pinMode(_drill_Speed_PWM[drill],OUTPUT);
  }
  drillTimer.every(5000, sendInfoToSerial);
  //Stepper motor and limit switches
 /* pinMode(_horizontalLeftLimit, INPUT_PULLUP);
  pinMode(_horizontalRightLimit, INPUT_PULLUP); 
  pinMode(_horizontalStepperDirection, OUTPUT); 

  pinMode(_vertical1UpLimit, INPUT_PULLUP);
  pinMode(_vertical1DownLimit, INPUT_PULLUP); 
  pinMode(_vertical1StepperDirection, OUTPUT); 

  pinMode(_vertical2UpLimit, INPUT_PULLUP);
  pinMode(_vertical2DownLimit, INPUT_PULLUP); 
  pinMode(_vertical2StepperDirection, OUTPUT); 

  pinMode(_stepperRelay, OUTPUT); 
  digitalWrite(_stepperRelay, LOW); //Relay for 48V stepper motor controller power

  xCurrentPositionInches = 0.0f; //needs calibrate() - we assume the horizontal stepper is all the way to the left on system start
  yCurrentPositionInches = 0.0f; //needs calibrate() - we assume the vertical stepper is all the way to the bottom on system start

  Timer1.initialize(drillCalibration.stepperPwmTargetPeriod); // 100 us = 10Khz (360rpm @ 1600 steps/revolution), 125=8Khz. set a timer of length 250 microseconds (or 0.000250 sec - or 4Khz)
  Timer3.initialize(drillCalibration.stepperPwmTargetPeriod); // 100 us = 10Khz (360rpm @ 1600 steps/revolution), 125=8Khz. set a timer of length 250 microseconds (or 0.000250 sec - or 4Khz)
  printCalibrationInfo();
  */
  return;
}
static void RoboDrill::sendInfoToSerial()
{  
  //TODO add back in when hooked up
  /*Serial.print("IR distance 1  -  2  -  3  -  4: ");
  for(int drill=0; drill<4; drill++){ //loop through all 4 drills  
    Serial.print (drillDistanceIR[drill]);
    if(atLimitSwitchDrill[drill])
      Serial.print("/true");
    else
      Serial.print("/false");

    Serial.print("    -    ");
  }
  Serial.println();
  */
  if (!utility.DEBUG_MODE)
    return;

  Serial.print("Drill (Down:Up) 1 / 2 / 3 / 4 : (");
  Serial.print(digitalRead(_drill_Down_Limit_Switch[0]));
  Serial.print(":");
  Serial.print(digitalRead(_drill_Up_Limit_Switch[0]));
  Serial.print(") / (");
  Serial.print(digitalRead(_drill_Down_Limit_Switch[1]));
  Serial.print(":");
  Serial.print(digitalRead(_drill_Up_Limit_Switch[1]));
  Serial.print(") / (");
  Serial.print(digitalRead(_drill_Down_Limit_Switch[2]));
  Serial.print(":");
  Serial.print(digitalRead(_drill_Up_Limit_Switch[2]));
  Serial.print(") / (");
  Serial.print(digitalRead(_drill_Down_Limit_Switch[3]));
  Serial.print(":");
  Serial.print(digitalRead(_drill_Up_Limit_Switch[3])); 
  Serial.println(")");

  Serial.print("Drill (IR Sensor) 1 / 2 / 3 / 4 : ");
  Serial.print(drillDistanceIR[0]);
  Serial.print(" / ");
  Serial.print(drillDistanceIR[1]);
  Serial.print(" / ");
  Serial.print(drillDistanceIR[2]);
  Serial.print(" / ");
  Serial.print(drillDistanceIR[3]);
  Serial.println();
}
void RoboDrill::run()
{
  drillTimer.update();
  checkIRSensors(); //Just gets a reading. does NOT stop drill 
  checkLimitSwitches(); //Stops the drill if it is at the limit switch (either up or down)

/*
  Serial.println("Starting drill");

    digitalWrite(_drill_Dir[2], DIRECTION_DOWN);
    analogWrite(_drill_Speed_PWM[2],40);
    delay(500);

  Serial.println("Pause before reversing drill");
    analogWrite(_drill_Speed_PWM[2],0);
    delay(500);

  Serial.println("Reverse drill");
    digitalWrite(_drill_Dir[2], DIRECTION_UP);
    analogWrite(_drill_Speed_PWM[2],40);
    delay(500);

  Serial.println("Stop drill");
    analogWrite(_drill_Speed_PWM[2],0);
    delay(15000);
*/



  if(!isDrillMoving[0] && !isDrillMoving[1] && !isDrillMoving[2] && !isDrillMoving[3])
    return; //drill is not moving so no need to slow down processing by ramping/stopping logic

  rampDrill();

  for(int drill=0; drill<4; drill++){ //loop through all 4 drills  

    unsigned long drillMovingTime = millis()-drillStartedMovingMillis[drill];

    //first, check to see if the drill has been running for too long (more than drillMoveMaxTimeMillis)
    if(isDrillMoving[drill] && drillMovingTime > drillCalibration.drillMoveMaxTimeMillis)
    {
      stopDrill(drill);
      Serial.println("Drill stopped after running longer than "+String(drillCalibration.drillMoveMaxTimeMillis));
    }
  } 




}

void RoboDrill::moveDrill(int drill)
{
  isDrillMoving[drill]=true;
  drillStartedMovingMillis[drill] = millis();

  //change direction
  if(drillDirection[drill]==DIRECTION_DOWN)
  {
    drillDirection[drill]=DIRECTION_UP;
    digitalWrite(_drill_Dir[drill], HIGH);
    //Serial.println("**************************************************Moving Up");
  }
  else
  {
    drillDirection[drill]=DIRECTION_DOWN;
    digitalWrite(_drill_Dir[drill], LOW);
    //Serial.println("**************************************************Moving Down");
  }

  analogWrite(_drill_Speed_PWM[drill],drillCalibration.drillStartSpeed);

  Serial.print("RoboDrill() - Drill ["+String(drill+1)+"] Started moving ");
  if(drillDirection[drill]==DIRECTION_UP)
    Serial.print("UP/Reverse...");
  else
    Serial.print("Down/Forward..."); 
  Serial.println(" atLimit(Up/Down):("+String(atUpLimitSwitchDrill[drill])+"/"+String(atDownLimitSwitchDrill[drill]) + ") IR distance:" + String(drillDistanceIR[drill])+" - "+String(millis())); 
}

void RoboDrill::rampDrill()
{ 

  if(millis()==lastDrillRampMillis)
    return; //we only ramp every millisecond
  else
    lastDrillRampMillis=millis();

  for(int drill=0; drill<4; drill++){ //loop through all 4 drills
    if(isDrillMoving[drill]) //we only ramp if it is moving
    {
      unsigned long drillRunTime = millis()-drillStartedMovingMillis[drill];
      //accelerate
      //if(drillRunTime < drillCalibration.drillRampPeriodMillis)
      //{
        int newDutyCycle = (float(float(drillRunTime)/float(drillCalibration.drillRampPeriodMillis))
          * drillCalibration.drillMaxSpeed);

        if(newDutyCycle<drillCalibration.drillStartSpeed)
          newDutyCycle=drillCalibration.drillStartSpeed;
        else if (newDutyCycle>drillCalibration.drillMaxSpeed)
          newDutyCycle = drillCalibration.drillMaxSpeed;

        //MOVE UP LOGIC 
        //We perform these logical steps:
        // 1. Move the drill up
        // 2. Pause to read the IR sensor
        // 3. Continue moving up if the IR sensor is not at the target distance
        // 4. Repeat steps 1 through 3 until we reach the target IR sensor distance
        //TODO: Add this code back in once all the IR sensors are hooked up
        //TODO
        //TODO
        //TODO: Add code below back in after IR sensors are working for all 4 drills
        /*
        if(drillDirection[drill]==DIRECTION_UP && drillRunTime > drillCalibration.drillMoveUpPauseAfterMillis)
        {
          int period = (drillRunTime - drillCalibration.drillMoveUpPauseAfterMillis)/drillCalibration.drillMoveUpPauseDurationMillis;
          if (period % 2 ==0 ) //it's an even number so lets pause the drill
          {
            newDutyCycle=0;
            if(drillCurrentSpeed[drill]!=newDutyCycle)
              Serial.println("Move up logic: duty:"+String(newDutyCycle)+" - "+String(millis()));
          }
          else //it's an odd number, lets move
          {
            newDutyCycle=drillCalibration.drillMoveUpAfterPauseSpeed;

            if(drillCurrentSpeed[drill]!=newDutyCycle)
              Serial.println("Move up logic: duty:"+String(newDutyCycle)+" - "+String(millis()));

            if(drillCurrentSpeed[drill]==newDutyCycle) //we are resuming from pause. so lets scheck the IR sensor
            {
              if(drillDistanceIR[drill] < drillCalibration.targetIRDrillUpDistance[drill]){ //we screwed all the way up  
                  stopDrill(drill);
                  Serial.println("Drill ["+String(drill+1)+"] UP IR Target distance reached:"+String(drillDistanceIR[drill])+" - "+String(millis()));
              }
            }
          }
        }
        */

        if(drillCurrentSpeed[drill]==newDutyCycle) 
          return; //already moving at the target speed

        drillCurrentSpeed[drill]=newDutyCycle;
        analogWrite(_drill_Speed_PWM[drill],drillCurrentSpeed[drill]);
        //Serial.println("RoboDrill().. ramping to " + String (newDutyCycle) + " at " + String(millis()));
      //}
    } 
  }
}

void RoboDrill::stop()
{
  for(int drill=0; drill<4; drill++){ //loop through all 4 drills  
    stopDrill(drill);
  }
}

void RoboDrill::stopDrill(int drill)
{
  isDrillMoving[drill]=false;
  drillStoppedMovingMillis[drill] = millis();
  analogWrite(_drill_Speed_PWM[drill],0);
  Serial.print("RoboDrill() - Drill ["+String(drill+1)+"] STOPPED moving ");
  if(drillDirection[drill]==DIRECTION_UP)
    Serial.print("UP/Reverse...");
  else
    Serial.print("Down/Forward..."); 
  Serial.println(" atLimit(Up/Down):(" + String(atUpLimitSwitchDrill[drill])+"/"+String(atDownLimitSwitchDrill[drill]) + ") IR distance:" + String(drillDistanceIR[drill])+" - "+String(millis())); 

}

void RoboDrill::checkIRSensors()
{
  // The sharp 16.5ms read cycle. So it is a waste to check more frequently.
  //  I measured it and analogRead() is expensive. 
  //  It slows the Arduino Mega 2650 from 28 to 2 cycles/loops per millisecond
  if(millis()-lastIRSensorReadingMillis < drillCalibration.irSensorFrequencyMillis)
  return;
  
  lastIRSensorReadingMillis = millis();

  //Up is 120 to 140 - set to 150
  //Down is 210 to 230 - set to 205

  for(int drill=0; drill<4; drill++){ //loop through all 4 drills  
    drillDistanceIR[drill] = (drillDistanceIR[drill]* drillCalibration.irSensorSmoothingFactor) + analogRead(_drill_IR_Sensor[drill])*(1- drillCalibration.irSensorSmoothingFactor);

    /*if(drillDistanceIR[drill] > drillCalibration.targetIRDrillDownDistance && 
      isDrillMoving[drill] &&
      drillDirection[drill] == DIRECTION_DOWN){ //we screwed all the way down  

        stopDrill(drill);
        Serial.println("Drill ["+String(drill+1)+"] IR Target distance reached:"+String(drillDistanceIR[drill]));
    }


    if(drillDistanceIR[drill] < drillCalibration.targetIRDrillUpDistance && 
      isDrillMoving[drill] &&
      drillDirection[drill] == DIRECTION_UP){ //we screwed all the way down  

        stopDrill(drill);
        Serial.println("Drill ["+String(drill+1)+"] IR Target distance reached:"+String(drillDistanceIR[drill]));
    }*/

    /*
    if(millis()- drillStoppedMovingMillis[drill] <100)
    {
      if(atDownLimitSwitchDrill[drill])
        return;
      if(drillDistanceIR[drill] < drillCalibration.targetIRDrillDownDistance-10 && 
      !isDrillMoving[drill] &&
      drillDirection[drill] == DIRECTION_DOWN){ //we screwed all the way down  
        isDrillMoving[drill]=true;
        drillStartedMovingMillis[drill] = millis();
        analogWrite(_drill_Speed_PWM[drill],drillCalibration.drillStartSpeed);
        
        Serial.print("RoboDrill() - Drill ["+String(drill+1)+"] restarting due to IR sensor noise ");
        Serial.println(String(atDownLimitSwitchDrill[drill]) + " IR distance:" + String(drillDistanceIR[drill])); 
      }
      if (drillDistanceIR[drill] > drillCalibration.targetIRDrillUpDistance+10 && 
      !isDrillMoving[drill] &&
      drillDirection[drill] == DIRECTION_UP )
      {
        isDrillMoving[drill]=true;
        drillStartedMovingMillis[drill] = millis();
        analogWrite(_drill_Speed_PWM[drill],drillCalibration.drillStartSpeed);
        Serial.print("RoboDrill() - Drill ["+String(drill+1)+"] restarting due to IR sensor noise ");
        Serial.println(String(atDownLimitSwitchDrill[drill]) + " IR distance:" + String(drillDistanceIR[drill])); 
      }
    }  
    */
      

  }

  //Up is 120 to 140
  //Down is 210 to 230 

}


void RoboDrill::checkLimitSwitches()
{ 
  for(int drill=0; drill<4; drill++){ //loop through all 4 drills 
    debounceLimitSwitch(drill); //remove noise/electromagnetic interference EMF 

    if(atDownLimitSwitchDrill[drill] && 
      isDrillMoving[drill] &&
      drillDirection[drill] == DIRECTION_DOWN){ //we screwed all the way down to the limit  
        stopDrill(drill);
        Serial.println("Drill ["+String(drill+1)+"] DOWN limit reached with IR distance:"+String(drillDistanceIR[drill])+" - "+String(millis()));
    }
    
    if(atUpLimitSwitchDrill[drill] && 
      isDrillMoving[drill] &&
      drillDirection[drill] == DIRECTION_UP){ //we screwed all the way up to the limit  
        stopDrill(drill);
        Serial.println("Drill ["+String(drill+1)+"] UP limit reached with IR distance:"+String(drillDistanceIR[drill])+" - "+String(millis()));
    }
  } 
}

void RoboDrill::debounceLimitSwitch(int drill)
{ 
  // BEGIN 
  // TODO
  //  Use this only for debugging when NOT connected to the robot 
  //return;
  // END

  // If we have gone on to the next millisecond
  if(millis() != lastDebounceTimeDrill[drill])
  {
    int downReading = digitalRead(_drill_Down_Limit_Switch[drill]);
    int upReading = digitalRead(_drill_Up_Limit_Switch[drill]);

    //TODO: Rewire in the next revision. HIGH should be at the limit. LOW should be away. this will reduce electromagnetic interference

    if(downReading == LOW)
    {
      if(debounceCounterDown[drill] < drillCalibration.debounceCount)
      {
       debounceCounterDown[drill]++;
      }
      else
      {
        if(!atDownLimitSwitchDrill[drill]){
          atDownLimitSwitchDrill[drill]=true;
          Serial.println(" switching drill["+String(drill)+"] DOWN limit to True");
        }
      }
    }
    else
    {
      if(debounceCounterDown[drill] > (-1 * drillCalibration.debounceCount))
      {
       debounceCounterDown[drill]--;
      }
      else
      {
        if(atDownLimitSwitchDrill[drill]){
          atDownLimitSwitchDrill[drill]=false;
          Serial.println(" switching drill["+String(drill)+"] DOWN limit to False");
        }
      }
    }


    if(upReading == LOW)
    {
      if(debounceCounterUp[drill] < drillCalibration.debounceCount)
      {
       debounceCounterUp[drill]++;
      }
      else
      {
        if(!atUpLimitSwitchDrill[drill]){
          atUpLimitSwitchDrill[drill]=true;
          Serial.println(" switching drill["+String(drill)+"] UP limit to True");
        }
      }
    }
    else
    {
      if(debounceCounterUp[drill] > (-1 * drillCalibration.debounceCount))
      {
       debounceCounterUp[drill]--;
      }
      else
      {
        if(atUpLimitSwitchDrill[drill]){
          atUpLimitSwitchDrill[drill]=false;
          Serial.println(" switching drill["+String(drill)+"] UP limit to False");
        }
      }
    }
/*
    Serial.println( "/"+String(debounceCounterUp[drill]));
     delay(100);

    if(debounceCounterUp[drill] >= drillCalibration.debounceCount)
    {
      if(!atUpLimitSwitchDrill[drill])
        Serial.println("switching UP limit to True "+String(debounceCounterUp[drill])+" >= "+String(drillCalibration.debounceCount));
      atUpLimitSwitchDrill[drill]=true;
      debounceCounterUp[drill] =0;
    }
    if(debounceCounterUp[drill] <= -1*drillCalibration.debounceCount)
    {
      if(atUpLimitSwitchDrill[drill])
        Serial.println("switching UP limit to False "+String(debounceCounterUp[drill])+" <= "+String(-1*drillCalibration.debounceCount));
      atUpLimitSwitchDrill[drill]=false;
      debounceCounterUp[drill] =0;
    } 
*/
    /*
    if((downReading == LOW && !atDownLimitSwitchDrill[drill]) //we hit the limit
      || (downReading == HIGH && atDownLimitSwitchDrill[drill])) //we moved away from the limit 
    {
      if(debounceCounterDown[drill] < drillCalibration.debounceCount) //we don't want to increment to infinity because it will take too long to switch back
        debounceCounterDown[drill]++;
      else{
        atDownLimitSwitchDrill[drill] = !atDownLimitSwitchDrill[drill];
        //if(atDownLimitSwitchDrill[index])
        //  atDownLimitSwitchDrillCount[index]++; // count the number of times we hit the limit switch 
        debounceCounterDown[drill] = 0;
      }
    }

    //Second, check the up limit switch
    if((upReading == LOW && !atUpLimitSwitchDrill[drill]) //we hit the limit 
      || (upReading == HIGH && atUpLimitSwitchDrill[drill])) //we moved away from the limit 
    {
      if(debounceCounterUp[drill] < drillCalibration.debounceCount) //we don't want to increment to infinity because it will take too long to switch back
        debounceCounterUp[drill]++;
      else{
        atUpLimitSwitchDrill[drill] = !atUpLimitSwitchDrill[drill];
        //if(atUpLimitSwitchDrill[index])
        //  atUpLimitSwitchDrillCount[index]++; // count the number of times we hit the limit switch 
        debounceCounterUp[drill] = 0;
      }
    }
    */

    lastDebounceTimeDrill[drill] = millis();
  } 
}
