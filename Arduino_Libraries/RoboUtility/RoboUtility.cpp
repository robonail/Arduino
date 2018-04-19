
/*
  RoboUtility.h - Library for the mechanical RoboUtility
  Author         Date          Note
  _____________  __________    __________________________________
  Michael Baird  5/5/2014      Initial creation
*/

#include "Arduino.h"
#include "RoboUtility.h"

RoboUtility::RoboUtility()
{
//The constructor runs before certain arduino components are loaded including interrupts needed for the delay() and attachInterrup() functions. Thus, we added method begin() to handle constructor setup 
}
String RoboUtility::floatToString(double number, uint8_t digits) 
{ 
  String returnString="";
  // Handle negative numbers
  if (number < 0.0)
  {
     returnString="-";
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  returnString+=String(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)    
    returnString+=String("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    returnString+=String(toPrint);
    remainder -= toPrint; 
  } 
  return returnString;
}

float RoboUtility:: stringToFloat(String stringInput)
{
  char delaystr[24]; //create the char array
  stringInput.toCharArray(delaystr,24); //convert the String 'command' into the char array
  float returnFloat = atof(delaystr); //convert the char array into float
  return returnFloat;
}

String RoboUtility::decodeUrl(String str)
{
    
    String encodedString="";
    char c;
    char code0;
    char code1;
    for (int i =0; i < str.length(); i++){
        c=str.charAt(i);
      if (c == '+'){
        encodedString+=' ';  
      }else if (c == '%') {
        i++;
        code0=str.charAt(i);
        i++;
        code1=str.charAt(i);
        c = (h2int(code0) << 4) | h2int(code1);
        encodedString+=c;
      } else{
        
        encodedString+=c;  
      }
      
      yield();
    }
    
   return encodedString;
}


String RoboUtility::encodeUrl(String str)
{
    String encodedString="";
    char c;
    char code0;
    char code1;
    char code2;
    for (int i =0; i < str.length(); i++){
      c=str.charAt(i);
      if (c == ' '){
        encodedString+= '+';
      } else if (isalnum(c)){
        encodedString+=c;
      } else{
        code1=(c & 0xf)+'0';
        if ((c & 0xf) >9){
            code1=(c & 0xf) - 10 + 'A';
        }
        c=(c>>4)&0xf;
        code0=c+'0';
        if (c > 9){
            code0=c - 10 + 'A';
        }
        code2='\0';
        encodedString+='%';
        encodedString+=code0;
        encodedString+=code1;
        //encodedString+=code2;
      }
      yield();
    }
    return encodedString;
    
}

unsigned char RoboUtility::h2int(char c)
{
    if (c >= '0' && c <='9'){
        return((unsigned char)c - '0');
    }
    if (c >= 'a' && c <='f'){
        return((unsigned char)c - 'a' + 10);
    }
    if (c >= 'A' && c <='F'){
        return((unsigned char)c - 'A' + 10);
    }
    return(0);
}

String RoboUtility::formatCommand(String cmd, String val)
{
  //Expected format is "command:value;"
  return cmd+":"+val+";";
}

String RoboUtility::acknowledgeCommand(String cmd, String val)
{
  //Expected format is "ack:command-value;"
  return "ack:"+cmd+"-"+val+";";
}
/*
bool RoboUtility::verifyAcknowledgement(String cmd, String val, String acknowledgementReceived)
{
  //Expected format is "ack:command-value;"
  if(cmd=="S") {
    //S-StatusRequests/I-Info are different. the acknowledgement is I-Info in format "I:value;"
    //Status requests are more frequent and we minimize bandwidth by just sending the information back
    if(acknowledgementReceived.startsWith("I:"))
      return true;
  }
  //Expected format is "ack:command-value;"
  if(acknowledgementReceived.equals(acknowledgeCommand(cmd,val)));
    return true;
  else
    return false;
}
*/
String RoboUtility::getCommand(String stringInput)
{
  //Expected format is "command:value;"
  int from = 0;
  int to = 0;
  if (stringInput.indexOf(':')>0)
    to = stringInput.indexOf(':');
  String result = stringInput.substring(from,to);
  return result;
}
String RoboUtility::getValue(String stringInput)
{

  int from = 0;
  int to = 0;
  /*if(stringInput.indexOf(':')>0 &&  stringInput.lastIndexOf(';') >0){
    from = stringInput.indexOf(':')+1;
    to = stringInput.lastIndexOf(';');
  } */
  if(stringInput.indexOf(':')>0){
    from = stringInput.indexOf(':')+1;
  }
  String result = stringInput.substring(from);

  if(result.lastIndexOf(';') >0)
    result = result.substring(0,result.lastIndexOf(';'));

  //Serial.println("RoboUtility input:"+stringInput);
  //Serial.println("RoboUtility value:"+result);
  return result;
}

String RoboUtility::getMessageFromHttpResponse(String stringInput)
{

  int from = 0;
  //int to = 0;
  //Serial.print("access control index:"+String(stringInput.indexOf('Access-Control-Allow-Origin: *')));
  //Serial.print(" "+String(stringInput.lastIndexOf('\r'))+" ");
  //Serial.println(stringInput);
  if(stringInput.indexOf('Access-Control-Allow-Origin: *')>0 ){ //&&  stringInput.lastIndexOf('\r') >0){
    from = stringInput.indexOf('Access-Control-Allow-Origin: *')+5;
    //to = stringInput.lastIndexOf('\r');
  }  
  String result = stringInput.substring(from); //,to);
  //Serial.println("RoboUtility input:"+stringInput);
  //Serial.println("RoboUtility value:"+result);
  return result;
}


