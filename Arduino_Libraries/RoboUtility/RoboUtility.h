
/*
  RoboUtility.h - Library for the mechanical RoboUtility
  Author         Date          Note
  _____________  __________    __________________________________
  Michael Baird  5/5/2014      Initial creation
*/

#ifndef RoboUtility_h
#define RoboUtility_h
//libraries 
#include "Arduino.h"




class RoboUtility
{
  public:
    bool DEBUG_MODE = true; //verbose logging and serial output to aid troubleshooting
    bool PROGRAMMING_MODE = false; //PROGRAMMING_MODE disables certain features only used when connected to the robot. this allows us to take a arduino and write code without constant errors being raised from sensors that don't exist without the robot

    RoboUtility();
  	String floatToString(double number, uint8_t digits);
  	float stringToFloat(String stringInput);
    String decodeUrl(String str);
    String encodeUrl(String str);

    //used for sending and receiving commands between devices 
    String formatCommand(String cmd, String val);
    String getCommand(String stringInput);
    String getValue(String stringInput);
    String acknowledgeCommand(String cmd, String val); 
    String getMessageFromHttpResponse(String stringInput);
    //bool verifyAcknowledgement(String cmd, String val, String acknowledgementReceived);

  private:
	//static RoboUtility * instance0_;
    unsigned char h2int(char c);
	
};
#endif

