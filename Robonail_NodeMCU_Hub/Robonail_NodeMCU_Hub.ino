#include <ESP8266WiFi.h> 
#include <ESP8266WebServer.h>
#include <Wire.h> //I2C library 
//#include <ArduinoJson.h> //Json formatter
#include <RoboUtility.h>
#include <Timer.h> //timer library to avoid using delay() that sleeps the processor

//////////////////////
// WiFi Definitions //
//////////////////////
const char WiFiAPPSK[] = "robonail"; //password to WEP2 wifi

/////////////////////
// Pin Definitions //
/////////////////////
const int LED_PIN = 2; // NodeMCU's onboard, blue LED
//const int LED_PIN2 = 12; //NodeMCU's onboard, red LED

unsigned long messagesTotal = 0;
unsigned long messagesSuccessful = 0; 

ESP8266WebServer server(80);
RoboUtility robo;
Timer timer;

/*String lastCommand="";
String lastValue="";
String command="";
String value="";
*/

String messageToSendNav="";
String messageToSendMobile="";
String messageToSendRipbot="";

String ripBotIpAddress="192.168.1.101";
String ripBotInfo="";
String navInfo = "";

//unsigned long millisLast=millis();
int sentCount = 0;
int receivedCount=0; 

bool ripBotConnected=false;

void setup() 
{
  initHardware();
  
  setupWiFi();
  server.begin();
  //Wire.setClock(400000L); //1000000); //400000);
  Wire.begin(D4, D3); //D3,D4); //(D2,D3); //(D7, D8); //(D5, D6); //D1, D2); // join i2c bus with SDA=D1 and SCL=D2 of NodeMCU 
  //Wire.setClock(400000L); //1000000); //400000);
  timer.every(75,receiveI2C); 
  timer.every(125,getWifiRipbotInfo);
  timer.every(5000,sendInfoToSerial);
  
  timer.every(750,toggleLed);
}

void loop() 
{
  delay(1); // needed to allow ESP8266 to run background wifi tasks 
  yield();
  timer.update(); 
  server.handleClient();

  if(messageToSendRipbot.length()>0){
    Serial.println("**********************************SENDING MESSAGE TO RIPBOT********************************");
    Serial.println(messageToSendRipbot);
    String cmd = robo.getCommand(messageToSendRipbot);
    String val = robo.getValue(messageToSendRipbot);
    String from = "nav"; //robo.getFrom(messageToSendRipbot);
    String ripBotResponse = sendWifiToRipbot(cmd, val, from);
    messageToSendRipbot="";
    Serial.println("**********************************RECEIVED ACK FROM RIPBOT********************************");
    Serial.println("Sent cmd / val / from: " + cmd + " / " + val + " / " + from);
    Serial.println("Received ACK: " + ripBotResponse);
  }
  //String ripBotResponse = sendWifiToRipbot(cmd, val, from);
  //String navResponse = sendI2CtoNav(cmd,val,from);
}


void receiveWifiCommand(){
  messagesTotal++;
  String cmd = String(server.arg("c"));
  String val = String(server.arg("v"));
  String from = String(server.arg("f"));
  from.replace(";","");

  //if(messageToSendNav.length()>0) //There is already a message in the queue, send it immediatly
  //  sendI2C();
   
  String htmlResponse;

  if(cmd.equals("ripbot_ip_address")){
    ripBotIpAddress = String(val);
    Serial.println("ip set to: "+ripBotIpAddress);
    ripBotConnected=true;
  }
  //messageToSend=robo.formatCommand(cmd,val);
  String ripBotResponse = sendWifiToRipbot(cmd, val, from);
  String navResponse = sendI2CtoNav(cmd,val,from);
  /*
  if(!cmd.equals("I")){
    Serial.println("from:"+from);
    if(from.equals("mobile;"))
    {
      htmlResponse=messageToSendMobile;
      messageToSendMobile="";
      messageToSendRipbot= messageToSendRipbot+robo.formatCommand(cmd,val);
      messageToSendNav = robo.formatCommand(cmd,val); //Queue the message up for the next time sendI2C() is called
    } else if (from.equals("ripbot;")){    
      htmlResponse=messageToSendRipbot;
      messageToSendRipbot="";
      messageToSendMobile= messageToSendMobile+robo.formatCommand(cmd,val);
      messageToSendNav = robo.formatCommand(cmd,val); //Queue the message up for the next time sendI2C() is called
    }
  }
  */

  htmlResponse = htmlResponse+ robo.acknowledgeCommand(cmd,val) +" - ripBot=" +  
      ripBotResponse+" - Nav="+ navResponse +";\n";
    
  server.send(200, "text/plain",htmlResponse);
  if(!cmd.equals("I") && !cmd.equals("ack")) //important command received. I-Info and ack-acknowledgement are meh
    Serial.println("NodeMCU received: " + robo.formatCommand(cmd,val) + " from "+from);
    
  messagesSuccessful++;
}

void receiveWifiInfoRequest(){
  messagesTotal++; 
  long a = random(-100,100);
  long b = random(-1000,1000);
  String htmlResponse="I:" + String(messagesTotal) + "," + String(messagesSuccessful) + ";\n";
  server.send(200, "text/plain",htmlResponse);
  messagesSuccessful++;
}

String sendWifiToRipbot(String cmd, String val, String from)
{
  if (!ripBotConnected) //ripBot is not connected. don't bother requesting info
    return "RipBot not connected";
   messagesTotal++;
  
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  client.setTimeout(500);
  
  const int httpPort = 80; 
  if (!client.connect(ripBotIpAddress.c_str(), httpPort)) {
    Serial.println("connection failed");
    return "";
  } 
  
  // We now create a URI for the request
  String url = "/cmd?c="+robo.encodeUrl(cmd)+"&v="+robo.encodeUrl(val)+"&f="+robo.encodeUrl(from);
  //String url = "/i";
   
  
  // This will send the request to the server
  client.print(String("GET ") + url  + " HTTP/1.1\r\n" +
               "Host: " + ripBotIpAddress + "\r\n" + 
               "Connection: Keep-Alive\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 1000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return "";
    }
  }

  String response;
  // Read all the lines of the reply from server and print them to Serial
  if(client.available()){
    String line = client.readStringUntil(';'); //'\r');
     
  response = robo.getMessageFromHttpResponse(line);
  }
  messagesSuccessful++;
  return response;
}

void getWifiRipbotInfo()
{
  if (!ripBotConnected) //ripBot is not connected. don't bother requesting info
    return;
    
  messagesTotal++;
  
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  client.setTimeout(500);
  
  const int httpPort = 80; 
  if (!client.connect(ripBotIpAddress.c_str(), httpPort)) {
    Serial.println("connection failed");
    return;
  } 
  
  // We now create a URI for the request
  //String url = "/cmd?c=I&v=99999&f=ripbot&s="+String(messagesTotal)+":"+String(messagesSuccessful)+";";
  String url = "/i";
   
  
  // This will send the request to the server
  client.print(String("GET ") + url  + " HTTP/1.1\r\n" +
               "Host: " + ripBotIpAddress + "\r\n" + 
               "Connection: Keep-Alive\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 1000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }
  
  // Read all the lines of the reply from server and print them to Serial
  if(client.available()){
    String line = client.readStringUntil(';'); //'\r');
    //line = line.substring(line.lastIndexOf('\n')+1,line.indexOf(';')); // grab the last line of the response
    //Serial.print("RipBot Info received: ");
    //Serial.println(line);
    //client.stop();
  ripBotInfo = robo.getMessageFromHttpResponse(line);
  }
  messagesSuccessful++; 
}

String sendI2CtoNav(String cmd, String val, String from)
{
  String message = robo.formatCommand(cmd,val);
  Serial.println("message to Nav:"+message);
  if(message.length()<1) //nothing to send
    return "nothing_to_send";
  Wire.beginTransmission(8); /* begin with device address 8 */
  Wire.write(message.c_str()); 
  Wire.endTransmission();    /* stop transmitting */ 
  
  delay(2); //we need to have a 2 second delay to avoid garbled communication
  
  //Now lets get the acknowledgement:
  String response; 
  if(Wire.requestFrom(8, 32)){ //, false)){ //request succeeded to read data of size 32 (maximum arduino supports) from slave  
    for(int i=0; i<32; i++){
      char c = Wire.read();
      response.concat(c);
    }
  }
  else{
    int ii =0; //Serial.println("Failed to get I2C message from Mega");
  }
  Serial.println("acknowledgement was:"+response);
  return response;
}

void receiveI2C()
{
  messagesTotal++;
  //delay(1); // needed to allow ESP8266 to run wifi tasks 
  String received; 
  if(Wire.requestFrom(8, 32)){ //, false)){ //request succeeded to read data of size 32 (maximum arduino supports) from slave  
    for(int i=0; i<32; i++){
      char c = Wire.read();
      received.concat(c);
    }
  }
  else{
    int ii = 0; //Serial.println("Failed to get I2C message from Mega");
  }
  /*while(Wire.available()){
    char c = Wire.read();
    //Serial.print(c);
    received.concat(c);
  }*/
  String cmd = robo.getCommand(received);
  String val = robo.getValue(received);
  
  navInfo = robo.formatCommand(cmd,val);
  navInfo.replace(";","");
  
  if(!cmd.equals("I")) //we don't care about Info commands
  {
    Serial.println("Received from mega:"+robo.formatCommand(cmd,val));
    messageToSendMobile=robo.formatCommand(cmd,val);
    messageToSendRipbot=robo.formatCommand(cmd,val);
  }

  if(cmd.length()<1)
    int i = 0; //Serial.println("Invalid response from mega:"+received);
  else
    messagesSuccessful++;
}

void sendInfoToSerial(){ 
  Serial.print("Hub Stats:");
  Serial.print(messagesSuccessful);
  Serial.print("/");
  Serial.print(messagesTotal);
  Serial.print("   RipBot Stats:"+ripBotInfo);
  Serial.println("   Nav Stats:"+navInfo);
}

void setupWiFi()
{
  
  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "Robonail " + macID;

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i=0; i<AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  // config static IP
  IPAddress ip(192, 168, 4, 100); // where xx is the desired IP Address
  IPAddress gateway(192, 168, 4, 1); // set gateway to match your network
  IPAddress subnet(255, 255, 255, 0); // set subnet mask to match your network
  //WiFi.config(ip, gateway, subnet);
  WiFi.softAPConfig (ip, gateway, subnet);
  WiFi.softAP(AP_NameChar, WiFiAPPSK);

  IPAddress apip = WiFi.softAPIP();
  Serial.print("visit: \n");
  Serial.println(apip);
  
  server.on("/i", receiveWifiInfoRequest); //request for info/status 
  server.on("/cmd", receiveWifiCommand);
  
  server.begin();
  Serial.println("HTTP server beginned");
}

void toggleLed(){
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void initHardware()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  digitalWrite(D1, HIGH); //supply 3.3v to levelshifter
  digitalWrite(D2, LOW); //ground levelshifter 
  
  // Don't need to set ANALOG_PIN as input, 
  // that's all it can be.
}




