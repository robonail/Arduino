
/*
  RoboDrill.h - Library for the controlling the 4 dewalt impact
  drills that fasten to the roof. 
  Author         Date          Note
  _____________  __________    __________________________________
  Michael Baird  12/31/2017      Initial creation
*/

#ifndef RoboDrill_h
#define RoboDrill_h
//libraries 
#include "Arduino.h"




class RoboDrill
{
  public:

    RoboDrill(); 
    void setup(); 
    void run();
    void stop();
    void stopDrill(int drill);
    void moveDrill(int drill);
    /*
    bool isStopped(); 
    bool isMoving();
    void saveToEEPROM();
    void getEEPROM();
    */

  private:  
    void checkIRSensors();
    void checkLimitSwitches();
    void debounceLimitSwitch(int drill);
    void rampDrill();
    static void sendInfoToSerial();
    /*
    void accelerateMotor();
    */
};
#endif

