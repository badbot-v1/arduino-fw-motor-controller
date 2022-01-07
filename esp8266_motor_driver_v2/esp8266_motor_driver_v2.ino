
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
//#include <AutoPID.h>

int RM_SPEED=5; //D1 PWM A
int LM_SPEED=4; //D2 PWM B

int RM_DIR=0; //D3 DIR A
int LM_DIR=2; //D4 DIR B


#ifndef STASSID
#define STASSID "badbot"
#define STAPSK  "badbot12"
#endif


#define DEBUG

unsigned int localPort = 40010;      // local port to listen on
unsigned int txPort = 40020; //for sending data to mcu

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; //buffer to hold incoming packet,

long lastRxAt = 0;
long lastTxAt = 0;
unsigned char lmSpeed = 0;
unsigned char rmSpeed = 0;

WiFiUDP Udp;

void setup() {
  Serial.begin(115200);

   pinMode(RM_SPEED, OUTPUT); 
   pinMode(LM_SPEED, OUTPUT); 
   pinMode(RM_DIR, OUTPUT); 
   pinMode(LM_DIR, OUTPUT); 
   pinMode(LED_BUILTIN, OUTPUT);

     
  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.print('.');
  }
  Serial.print("WiFi MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("WiFi IP: "); Serial.println(WiFi.localIP());
  
  Serial.printf("UDP server on port %d\n", localPort);
  Udp.begin(localPort);


  //dance
  digitalWrite(LM_DIR, HIGH);    
  digitalWrite(RM_DIR, LOW);    
  
}

char heartbeat[3]="hb";

void loop() 
{
   // if there's data available, read a packet 
   long currMillis = millis();
   
   bool wifiReconnected = false;

    while (WiFi.status() != WL_CONNECTED) 
    {
      wifiReconnected = true;
      delay(1000);
      Serial.print(".");
    }

    if(wifiReconnected == true)
    {
      Serial.print("WiFi MAC: "); Serial.println(WiFi.macAddress());
      Serial.print("WiFi IP: "); Serial.println(WiFi.localIP());
  
      Udp.begin(localPort);
    }

    //send heartbeat every 1 second
    if( (currMillis - lastTxAt) > 1000)
    {
      //update last tx time
      lastTxAt = currMillis;

      //send heatbeat
      Udp.beginPacket("10.42.0.1", txPort );
      Udp.write((byte*)heartbeat, sizeof(heartbeat));
      Udp.endPacket();
      
    }
     
  int packetSize = Udp.parsePacket();
  if (packetSize) 
  {

    int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;

    lmSpeed = (unsigned char)packetBuffer[0];
    rmSpeed = (unsigned char)packetBuffer[1];

    char data[255];

    #ifdef DEBUG
    sprintf(data,"LM: [%d] RM: [%d]", lmSpeed, rmSpeed);
    Serial.println(data);
    #endif
    
    lastRxAt = millis();
   
    if(lmSpeed > 0){
      analogWrite(LM_SPEED, map(lmSpeed, 1,255, 800,1023));
      //digitalWrite(LM_SPEED, 1);
      #ifdef DEBUG
      Serial.println("Left High");
      #endif
    }else{
      digitalWrite(LM_SPEED, 0);
      #ifdef DEBUG
      Serial.println("Left Low");
      #endif
    }

    if(rmSpeed > 0){
      analogWrite(RM_SPEED, map(rmSpeed, 1,255, 800,1023));
      //digitalWrite(RM_SPEED, 1);
      #ifdef DEBUG
      Serial.println("Right High");
      #endif
    }else{
      digitalWrite(RM_SPEED, 0);
      #ifdef DEBUG
      Serial.println("Right Low");
      #endif
    }

  }


  if( (currMillis - lastRxAt) > 100){

      digitalWrite(LM_SPEED, 0);
      digitalWrite(RM_SPEED, 0);
    
  }
}
