/*
  Bluetooth.h - Library for bluetooth communication
  on the serial port. This library enables us to send
  ascii text to a mobile device (e.g. an android phone).
  The format expected is <command,value> 
  the brackets <> indicate start and end of message
  the comma , is a delimiter
  
  Author         Date          Note
  _____________  __________    __________________________________
  Michael Baird  5/5/2014      Initial creation
  Michael Baird  7/10/2017     fixed defect in Arduino Mega Serial2 with pinMode( 17, INPUT_PULLUP );
*/

#include "Arduino.h"
#include "Bluetooth.h"

// Example 5 - Receive with start- and end-markers combined with parsing

#define blutoothGndPin 22
#define numChars 64

char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing


// variables to hold the parsed data
char commandFromPC[numChars] = {0};
char valueFromPC[numChars] = {0};
int integerFromPC = 0;
float floatFromPC = 0.0;

//boolean newData = false; //tells us when there is new data
boolean _messageAvailable = false; //tells us when the consumer pulls the data out of our buffer variables
                                 // we don't want to read another command until someone reads the first one
//============
Bluetooth::Bluetooth()
{
    Serial2.begin(9600); 
    pinMode( 17, INPUT_PULLUP ); // fix Serial2 known issue with arduino Mega
    pinMode(blutoothGndPin,OUTPUT);
    digitalWrite(blutoothGndPin,LOW);
}
//============
void Bluetooth::update()
{
    recvWithStartEndMarkers();
    if (_messageAvailable == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        //showParsedData();
        //newData = false;
        //_messageAvailable = true;
    }
}
//============
bool Bluetooth::messageAvailable()
{
    return _messageAvailable;
}
//============
void Bluetooth::sendMessage(String command, String value)
{
	Serial2.print("<");
	Serial2.print(command);
	Serial2.print(",");
	Serial2.print(value);
	Serial2.println(">");
} 
//============
String Bluetooth::getCommand()
{
	String cmdFromPC(commandFromPC);
	return cmdFromPC;
}
//============
String Bluetooth::getValue()
{
	String valFromPC(valueFromPC);
	_messageAvailable = false; //we can read the next message after the consumer reads the first one
	return valFromPC;
}
//============
void Bluetooth::recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial2.available() > 0 && _messageAvailable == false) {
        rc = Serial2.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                _messageAvailable = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============
void Bluetooth::parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    strcpy(commandFromPC, strtokIndx); // copy it to commandFromPC

    
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    strcpy(valueFromPC, strtokIndx);  // copy it to valueFromPC
    
    /**********************************************************************************
     * The lines of code below show how to get an integer and a float from the values passed in
     */
     /*
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    integerFromPC = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    floatFromPC = atof(strtokIndx);     // convert this part to a float
    */

}
//============

 
/*
String _sendLines; //Store the lines of text to be sent to the receiving device
String _receiveLines; //Store the lines of text received from the mobile device
int _rxPin; //RX Pin (connect to TX of other device)
int _txPin; //TX Pin (connect to RX of other device)
boolean lineComplete = false; //full line of text received
	
//SoftwareSerial Genotronex(10,11); // RX, TX

Bluetooth::Bluetooth()
{
	//Genotronex.begin(9600);
	Serial1.begin(9600);
	_receiveLines.reserve(56);
}

void Bluetooth::println(String line)
{
	Serial1.flush();
	Serial1.println(line);
}


void serialEvent1(){
//Serial.println("serialEvent1()");
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read(); 
    // add it to the _receiveLines:
    _receiveLines += inChar;
    if (inChar == '\n') {
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
      lineComplete = true;
    // add it to the _receiveLines:
    //_receiveLines += inChar;
    }
	else{
    // add it to the _receiveLines:
    //_receiveLines += inChar;
	}
  }
}
String Bluetooth::receive()
{
// print the string when a newline arrives:
	String returnString = String(_receiveLines);
  if (lineComplete) {
    // clear the string:
    _receiveLines = "";
    lineComplete = false;
  }
	return returnString; 
}
*/