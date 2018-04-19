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
#ifndef Bluetooth_h
#define Bluetooth_h
//libraries 
#include "Arduino.h"


#define blutoothGndPin 22
#define numChars 64

class Bluetooth
{
	public:
		Bluetooth();
		void update();
		bool messageAvailable();
		String getCommand();
		String getValue();
		void sendMessage(String command, String value);
		void println(String line);
		//void send(String message);
		String receive();
		//void SerialEvent1();
	private:
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
		void recvWithStartEndMarkers();
		void parseData();
};
#endif

