#include <Wire.h> //I2C library 
#include <ArduinoJson.h> //Json formatter
#include <Timer.h> //timer library to avoid using delay() that sleeps the processor
#include <RoboUtility.h> 
#include <RoboStepper.h> 
#include <RoboDrill.h> 
#include <AccelStepper.h>
//#include "iAccelStepper.h" //interrupt based stepper library
#include <TimerOne.h> //interrupt based timer library. use ONLY for stepper frequency to gaurantee smooth motion
#include <TimerThree.h> //interrupt based timer library

/*
// move to Robonail_Inclinometer library
#include <SPI.h>


// Set pins
const int dataINPin = 50;     //MISO White-wire
const int dataOUTPin = 51;    //MOSI Yellow-wire
const int serialClockPin = 52;//SCK  Green-wire
const int chipSelectPin = 53;//Chip Select Pin

//!!!Sets commands according to spec sheet to access memory register addresses
//!!!Commands are 8 bits or one byte

const byte MEAS  = B00000000; //Measure mode (normal operation mode after power on)
const byte RWTR  = B00001000; //Read and write temperature data register, ONLY WRITE COMMAND, note currently in use
const byte RDSR  = B00001010; //Read status register
const byte RLOAD = B00001011; //Reload NV data to memory output register
const byte STX   = B00001110; //Activate Self test for X-channel
const byte STY   = B00001111; //Activate Self test for Y-channel
const byte RDAX  = B00010000; //Read X-channel acceleration through SPI
const byte RDAY  = B00010001; //Read Y-channel acceleration through SPI
*/

RoboUtility robo;
Timer timer;
RoboStepper roboStepper;
RoboDrill roboDrill;

unsigned long messagesTotal = 0;
unsigned long messagesSuccessful = 0; 
String response;
String receivedCmd="";
String receivedVal="";
 
float xDegrees = 0; //+/-90 degrees where analog reading 512 = 0 degrees
float yDegrees = 0; //+/-90 degrees where analog reading 512 = 0 degrees


bool isTestingLoopRunning=false;
int testingLoopNumber = 1;
  
void setup() {

 //Wire.setClock(400000L); //1000000); //400000);
 Wire.begin(8);                /* join i2c bus with address 8 */
 Wire.onReceive(receiveEvent); /* register receive event */
 Wire.onRequest(requestEvent); /* register request event */
 Serial.begin(115200); //115200);           /* start serial for debug */
 Serial1.begin(115200);           /* start serial for debug */
 
  timer.every(4000, sendInfoToSerial);
  timer.every(1000,runTestingLoop);
  //timer.every(10000,tempMoveStepper);



  // move to Robonail_Inclinometer library
  //SCA100T_D02_setup(); 
  

  
/*
 int debounceCount=1000;
unsigned long lastDebounceTime[30]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // the last time we read the switch
int debounceCounter[30]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // count the milliseconds for each reading
//bool atLimitSwitch[6] = {false,false,false,false,false,false}; //whether the stepper is at limit switch
int lastReading[30] = {LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW};  // count the number of times we hit each limit switch 

while(1==1)
{
  for(int i=25; i<55; i++)
    {
      pinMode(i,INPUT_PULLUP);
      int reading = digitalRead(i);
      if (reading != lastReading[i-25])
      {
        debounceCounter[i-25]++;
        if(debounceCounter[i-25]>=debounceCount)
        {
          Serial.println("pin changed "+String(i));
          lastReading[i-25] = reading;
        }
      }
      else
        debounceCounter[i-25]=0;
      
    }
}
*/
  randomSeed(analogRead(0));
  roboStepper.setup();
  roboDrill.setup();
  
}
unsigned long countLoops = 0;
unsigned long maxLoopsPerMilli = 0;
unsigned long lastLoopMillis = 0;
unsigned long maxMillisBetweenLoops = 0;


void loop() { 

 countLoops++;
 if(millis()!=lastLoopMillis){
  if(millis() - lastLoopMillis > maxMillisBetweenLoops)
    maxMillisBetweenLoops = millis() - lastLoopMillis;
  //if(countLoopsThisMillis> maxLoopsPerMilli)
  //  maxLoopsPerMilli = countLoopsThisMillis;
  lastLoopMillis=millis();
  //countLoopsThisMillis=0;
 }
 
 if(response.length()<1)
  response = "I:"+String(messagesSuccessful)+"/"+String(messagesTotal)+";                       ";


 calculateXYAngle();
 roboStepper.run(); //takes us from 46 loops per milli to 14 
 roboDrill.run(); // takes us from 46 loops per milli to 2 
 

 if(roboStepper.isError()){
  Serial.println("*************************************************************************");
  Serial.println("stopStepper() STOP the HSS86 motor controllers after roboStepper.isError()");
  Serial.println("*************************************************************************");
  roboStepper.stop();
  timer.after(600, startStepper);
 }
 
 if(receivedCmd.length()>0) //command in queue, need to process it 
  processCommand();

 timer.update(); 
}
 
void startStepper(){
  Serial.println("*************************************************************************");
  Serial.println("startStepper() START the HSS86 motor controllers after roboStepper.isError()");
  Serial.println("*************************************************************************");
  roboStepper.start();
}


unsigned long millisLast=millis();
int sentCount = 0;
int receivedCount=0;


const byte numChars = 32;
char receivedChars[numChars]="I:this is a message that is 32 ;";   // an array to store the received data


// function that executes whenever data is received from master 
//This is an interrupt routine. needs to be FAST. No Serial.println, no complex calculations. FAST
// The code will break and cause garbled messages if we do too much computing here
void receiveEvent(int howMany) {
 messagesTotal++;
 String received;
 while (0 <Wire.available()) {
    char c = Wire.read();      // receive byte as a character
      received.concat(c);
  }
 if(receivedCmd.length()>0) //command in queue, need to process it first
  processCommand();
  
 receivedCmd=robo.getCommand(received);
 receivedVal=robo.getValue(received);
 response = robo.acknowledgeCommand(receivedCmd, receivedVal); 
 receivedCount++;
 messagesSuccessful++;
}

// function that executes whenever data is requested from master
//This is an interrupt routine. needs to be FAST. No Serial.println, no complex calculations. FAST
// The code will break and cause garbled messages if we do too much computing here
void requestEvent() {
  messagesTotal++;
   if(response.length()<1)
    response = "I:"+String(messagesSuccessful)+"/"+String(messagesTotal)+";                        ";
  Wire.write(response.c_str(),32);
  response=""; 
  messagesSuccessful++;
}

void stopNav(){
  Serial.println("Stop all movement");
  
  isTestingLoopRunning=false;
  roboDrill.stop();
  roboDrill.resetError();
  
  roboStepper.resetError();
  if(roboStepper.isStopped())
    roboStepper.start();
  else
    roboStepper.stop();
}
void processCommand(){
 
 Serial.println("Mega Nav received:"+robo.formatCommand(receivedCmd,receivedVal));
 if(receivedCmd.equals("X")){
  stopNav();
 } else if(receivedCmd.equals("R")){
  Serial.println("Move right");
  float moveX = receivedVal.toFloat();
  //if(moveX>roboStepper.getRightAvailableInches())
  //  roboStepper.move(roboStepper.getRightAvailableInches(), 0.0f);
  //else 
    roboStepper.move(moveX,0.0f); 
 } else if(receivedCmd.equals("L")){
  Serial.println("Move left");
  float moveX = receivedVal.toFloat();
  //if(moveX>roboStepper.getLeftAvailableInches())
  //  roboStepper.move(-roboStepper.getLeftAvailableInches(), 0.0f);
  //else 
    roboStepper.move(-moveX,0.0f);
 } else if(receivedCmd.equals("U")){
  Serial.println("Move up");
  float moveY = receivedVal.toFloat();
  roboStepper.move(0.0f,moveY); 
 } else if(receivedCmd.equals("D")){
  Serial.println("Move down");
  float moveY = receivedVal.toFloat();
  roboStepper.move(0.0f,-moveY); 
 } else if(receivedCmd.equals("D1")){
  Serial.println("Drill 1");
  roboDrill.moveDrill(0);
 } else if(receivedCmd.equals("D2")){
  Serial.println("Drill 2");
  roboDrill.moveDrill(1);
 } else if(receivedCmd.equals("D3")){
  Serial.println("Drill 3");
  roboDrill.moveDrill(2);
 } else if(receivedCmd.equals("D4")){
  Serial.println("Drill 4");
  roboDrill.moveDrill(3);
 } else if(receivedCmd.equals("Ds")){
  Serial.println("Switching between drill sets 1&2 vs 3&4");
  roboDrill.switchDrillSets();
 } else if(receivedCmd.equals("T1")){
  Serial.println("Test routine 1");
  isTestingLoopRunning=true; 
 } else if(receivedCmd.equals("Z")){
  Serial.println("Received Z - Sending d2 to ripbot in 1 second");
  timer.after(1000, sendMessageToRipbot); 
 } else if(receivedCmd.equals("SV")){
  Serial.println("Speed set to "+receivedVal);
  //TODO TEST PWM stepperHorizontal.setMaxSpeed(receivedVal.toFloat()); //600); //1000
  roboStepper.setCalibrationValue("name", "value");
  //TODO: break apart the value from format cmd,v
  //Timer1.setPeriod(receivedVal.toInt()); // 33 of 1024 is a 3.2% duty cycle. about 4 microseconds on an 8Khz frequency
 }
 
  
  
 
 receivedCmd="";
 receivedVal="";
}

void sendMessageToRipbot(){
  response = "G:move"; //"G:UP";
  Serial.println("Sent " + response + " to RipBot");
}
void sendInfoToSerial(){
  Serial.print("Degrees X/Y: ");
  Serial.print(xDegrees);
  Serial.print("/");
  Serial.print(yDegrees); 

  Serial.print(" -Drills(position:IR): ");
  for(int drill=0; drill<4; drill++){ //loop through all 4 drills 
    if(roboDrill.getDrillPosition(drill)==1)
      Serial.print(" UP:");
    if(roboDrill.getDrillPosition(drill)==2)
      Serial.print(" DOWN:");
    if(roboDrill.getDrillPosition(drill)==3)
      Serial.print(" MIDDLE:");
    if(roboDrill.getDrillPosition(drill)==4)
      Serial.print(" ERROR:"); 
    Serial.print(roboDrill.getDrillIRSensor(drill));
  }
  Serial.println();
    /*
    Serial.print("Received count:");
    Serial.print(receivedCount);
    Serial.print("   Sent count:");
    Serial.print(sentCount);
    Serial.print("   stats:");
    Serial.print(messagesSuccessful);
    Serial.print("/");
    Serial.print(messagesTotal);
    Serial.print(" - Steps: ");
    Serial.println(0); //stepperIsrCount);
    //stepperIsrCount=0;
    //Serial.println(received);
    millisLast=millis();
    sentCount = 0;
    receivedCount=0;
    */
}



void calculateXYAngle()
{
  int xTilt = analogRead(A7);
  int yTilt = analogRead(A6);
  float currentXDegrees = (float(xTilt) - 512.0f) / (1024.0f / 180.0f); //+/-90 degrees where 512 = 0 degrees
  float currentYDegrees = (float(yTilt) - 512.0f) / (1024.0f / 180.0f); //+/-90 degrees where 512 = 0 degrees

  xDegrees = 0.9 * xDegrees + 0.1 * currentXDegrees; //smooth out over 10 readings 
  yDegrees = 0.9 * yDegrees + 0.1 * currentYDegrees; //smooth out over 10 readings 
}



int currentLoop = 1;
int currentCommand = 0;

void runTestingLoop()
{
  if(!isTestingLoopRunning)
    return;
  if(roboDrill.isMoving())
    return;
  if(roboStepper.isMoving())
    return;

  if(roboDrill.isError() || roboStepper.isError())
  {
    Serial.println("******************************** ERROR ***************************************");
    Serial.println("Stopping runTestingLoop(drillErrorCode / stepperErrorCode) : "+ String(roboDrill.getErrorCode()) + " / " + String(roboStepper.getErrorCode()));
    Serial.println("************************************************************************************");
    roboDrill.stop();
    roboStepper.stop();
    isTestingLoopRunning=false;
    return;
  }
  const int LOOPS=12;
  const int COMMAND_SIZE = 6;
  
  char* commands[COMMAND_SIZE]={"D2", "D4", "D2", "D4", "U", "R"};
  char* values[COMMAND_SIZE]=  {"00", "00", "00", "00", "1", "1"};
  
  if(currentCommand == COMMAND_SIZE) //we made it through all commands, lets loop through again
  {
    currentLoop++;
    currentCommand = 0;
  }
  
  if(currentLoop > LOOPS) // we made it through all loops, lets stop 
  {
    currentLoop=1;
    currentCommand=0;
    isTestingLoopRunning=false;
    roboDrill.saveToEEPROM();
    return;
  }

  Serial.println("Running Testing Loop (loop, command, value): "+String(currentLoop)+",  "+String(commands[currentCommand])+",  "+String(values[currentCommand]));
  receivedCmd = String(commands[currentCommand]);
  receivedVal = String(values[currentCommand]);
  processCommand();
  currentCommand++;
}





