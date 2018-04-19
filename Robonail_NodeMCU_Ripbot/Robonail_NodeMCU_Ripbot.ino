#include <ESP8266WiFi.h>
#include <Timer.h> //timer library to avoid using delay() that sleeps the processor
#include <ESP8266WebServer.h>
#include <RoboUtility.h>

const char* ssid = "Robonail B924"; //"Robonail 206B"; //"Robonail F057";
const char* password = "robonail";

const char* host = "192.168.4.100";
 //http://192.168.4.1/cmd?c=U&v=5.625
 
unsigned long messagesTotal = 0;
unsigned long messagesSuccessful = 0; 
long lastInfoRequest = millis();
int limitSwitch1 = 0;
int limitSwitch2 = 0;

ESP8266WebServer server(80);
RoboUtility robo;
Timer timer;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(D0,OUTPUT);
  //digitalWrite(D0,HIGH);
  
  //air cylinder
  pinMode(D1,OUTPUT);
  digitalWrite(D1,HIGH);
  //linear actuator 1
  pinMode(D2,OUTPUT);
  digitalWrite(D2,HIGH);
  //linear actuator 2
  pinMode(D3,OUTPUT);
  digitalWrite(D3,HIGH);
  
  pinMode(D4,OUTPUT);
  digitalWrite(D4,HIGH);

  //limit switch 1
  pinMode(D5,INPUT_PULLUP);
  //limit switch 2
  pinMode(D6,INPUT_PULLUP);

  pinMode(D7,INPUT);
  
  //digitalWrite(LED_BUILTIN, HIGH); //Needs to be High for D0 to have voltage
  setupWifi();
  timer.every(750,toggleLed);
  
  //timer.every(5000,sendRipBotIPaddress); // just in case we reconnect and get a new IP address
  
//  timer.every(200,sendWifiCommand); //communicate every x milliseconds
}
 
void loop() { 
  delay(1); // needed to allow WiFi tasks to run in the background 
  yield();
  
  if(WiFi.status() != WL_CONNECTED){ //lets try reconnecting to wifi 
    Serial.print("WiFi connection problem. trying to reconnect ");
    Serial.println(WiFi.status());
    WiFi.disconnect();
    delay(5000);
    setupWifi();
  }

  limitSwitch1 = digitalRead(D5);
  limitSwitch2 = digitalRead(D6);

  //TODO: put logic in for the limit switches
  //if (limitSwitch1 == HIGH)
  //  digitalWrite(D2, HIGH);
  
  timer.update();
  server.handleClient();
  
  
}
void toggleLed(){
  //digitalWrite(D4, !digitalRead(D4));
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
void setupWifi()
{
  delay(2000); // give the AP a chance to set up Wifi for us to connect to on system start

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  //IPAddress ip(192, 168, 4, 200);

  // config static IP
  /*IPAddress ip(192, 168, 4, 101); // where xx is the desired IP Address
  IPAddress gateway(192, 168, 4, 1); // set gateway to match your network
  IPAddress subnet(255, 255, 255, 0); // set subnet mask to match your network
  WiFi.config(ip, gateway, subnet);
  */
  
  WiFi.begin(ssid, password);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); 
    Serial.print(". status:");
    Serial.print(WiFi.status());
    tries++;
    if(tries>20){
      WiFi.disconnect();
      return;
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  //Now that we know our localIP, lets send it to the Nav so it knows how to contact us
  //cmd="ripbot_ip_address";
  //val=WiFi.localIP().toString();
  
  //sendRipBotIPaddress();
  sendWifiCommand("ripbot_ip_address",WiFi.localIP().toString());
  
  //set up the webserver
  server.on("/cmd", receiveWifiCommand); //command 
  server.on("/i", receiveWifiInfoRequest); //request for info/status 
  server.begin();
}
/*void sendRipBotIPaddress(){
  sendWifiCommand("ripbot_ip_address",WiFi.localIP().toString());
}*/
void sendWifiCommand(String cmd, String val)
{
  messagesTotal++;
  
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80; 
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  } 
  
  // We now create a URI for the request
  //String url = "/cmd?c=I&v=99999&f=ripbot&s="+String(messagesTotal)+":"+String(messagesSuccessful)+";";
  String url = "/cmd?c="+robo.encodeUrl(cmd)+"&v="+robo.encodeUrl(val)+"&f=ripbot;";
   
  
  // This will send the request to the server
  client.print(String("GET ") + url  + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" + 
               "Connection: Keep-Alive\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }
  
  // Read all the lines of the reply from server and print them to Serial
  if(client.available()){
    String line = client.readStringUntil(';'); //'\r');
    //line = line.substring(line.lastIndexOf('\n')+1,line.indexOf(';')); // grab the last line of the response
    Serial.print("Robonail received:");
    Serial.println(robo.getMessageFromHttpResponse(line));
    //client.stop();
  }
  messagesSuccessful++; 
}

void receiveWifiInfoRequest(){
  messagesTotal++; 
  lastInfoRequest = millis();
  long a = random(-100,100);
  long b = random(-1000,1000);
  String htmlResponse="I:" + String(messagesSuccessful) + "/" + String(messagesTotal) + ";\n";
  server.send(200, "text/plain",htmlResponse);
  messagesSuccessful++;
}

void receiveWifiCommand(){
  messagesTotal++;
  String cmd = String(server.arg("c"));
  String val = String(server.arg("v"));
  String from = String(server.arg("f"));
  from.replace(";",""); 
  
  if(cmd.equals("G")){ //raise or lower the gate
    moveGate(val.charAt(0));
  } 
  else if(cmd.equals("P")){ //raise or lower the pry bar
    movePryBar(val.charAt(0));
  }
  /*
  if(cmd.equals("d1")){
    digitalWrite(D1, !digitalRead(D1));
    if (digitalRead(D1))
      Serial.println("D1 High (+5V)");
    else
      Serial.println("D1 Low (0V)");
  }
  else if(cmd.equals("d2")){
    digitalWrite(D2, !digitalRead(D2));
     if (digitalRead(D2))
      Serial.println("D2 High (+5V)");
    else
      Serial.println("D2 Low (0V)");
  }
  else if(cmd.equals("d3")){
    digitalWrite(D3, !digitalRead(D3));
     if (digitalRead(D3))
      Serial.println("D3 High (+5V)");
    else
      Serial.println("D3 Low (0V)");
  }*/
  String htmlResponse =robo.acknowledgeCommand(cmd,val);
  server.send(200, "text/plain",htmlResponse);
  
  Serial.println("Ripbot received: " + robo.formatCommand(cmd,val) + " from "+from);
    
  messagesSuccessful++;
}

void moveGate(char dir) // U = up, D = down
{
   if(dir=='U'){
    digitalWrite(D2, LOW);
    digitalWrite(D3, LOW);
    digitalWrite(D3, HIGH); 
    Serial.println("raising Gate Up");

    //digitalWrite(D2, HIGH);
    //digitalWrite(D3, HIGH);
    //Serial.println("1/2 there");
    //delay(2000);
    //digitalWrite(D3, LOW);
    //Serial.println("raising Gate Up");
    //delay(5000);
  }
  else if(dir=='D'){
    digitalWrite(D2, LOW);
    digitalWrite(D3, LOW);
    digitalWrite(D2, HIGH); 
    Serial.println("lowering Gate Down");
    //digitalWrite(D2, HIGH);
    //digitalWrite(D3, HIGH);
    //Serial.println("1/2 there");
    //delay(2000);
    //digitalWrite(D2, LOW);
    //Serial.println("lowering Gate Down");
    //delay(5000);
  }/*
  else if(dir=='d'){
    digitalWrite(D2, LOW);
    digitalWrite(D3, LOW);
    delay(2000);
    Serial.println("1/2 there");
    digitalWrite(D2, HIGH); 
    Serial.println("lowering Gate down");
    delay(5000);
  }
  else if(dir=='u'){
    digitalWrite(D2, LOW);
    digitalWrite(D3, LOW);
    delay(2000);
    Serial.println("1/2 there");
    digitalWrite(D3, HIGH); 
    Serial.println("raising Gate up");
    delay(5000);
  }*/
  else {
    char oppositeDirection =' ';
    
    if (digitalRead(D3)==HIGH)
      oppositeDirection = 'D';
    else 
      oppositeDirection = 'U';
      
    Serial.println("Switching gate to opposite direction of " +String(oppositeDirection));
    moveGate(oppositeDirection);
  }
}


void movePryBar(char dir) // U = up, D = down
{
   if(dir=='U'){
    digitalWrite(D1, LOW);
    Serial.println("raising PryBar Up");
  }
  else if(dir=='D'){
    digitalWrite(D1, HIGH);
    Serial.println("lowering PryBar Down");
  }
  else {
    char oppositeDirection =' ';
    
    if (digitalRead(D1)==HIGH)
      oppositeDirection = 'U';
    else 
      oppositeDirection = 'D';
      
    Serial.println("Switching PryBar to opposite direction of " +String(oppositeDirection));
    movePryBar(oppositeDirection);
  }
}




