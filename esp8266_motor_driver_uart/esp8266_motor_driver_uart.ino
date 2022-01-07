#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

//https://cdn.hackaday.io/files/8856378895104/user-mannual-for-esp-12e-motor-shield.pdf
int RM_SPEED=5; //D1 PWM A
int LM_SPEED=4; //D2 PWM B

int RM_DIR=0; //D3 DIR A
int LM_DIR=2; //D4 DIR B

#define HDR_QUALIFIER       0xF8
#define HDR_PING            HDR_QUALIFIER | 0x01
#define HDR_CMD_DRIVELINE   HDR_QUALIFIER | 0x02

//#define DEBUG
#define SERIAL_READ_TIMEOUT 200


/*
 * pin layout of esp board
 * https://i2.wp.com/randomnerdtutorials.com/wp-content/uploads/2019/05/ESP8266-NodeMCU-kit-12-E-pinout-gpio-pin.png?ssl=1
 * 
 * example of motor sheild with code
 * https://hackaday.io/project/8856-incubator-controller/log/29291-node-mcu-motor-shield
 * 
 * example code soft serial
 * https://github.com/plerup/espsoftwareserial/blob/master/examples/onewiretest/onewiretest.ino
 * 
 * header soft serial
 * https://github.com/plerup/espsoftwareserial/blob/master/src/SoftwareSerial.h
 * 
 * 
 * 
 */

#if defined(ESP8266) && !defined(D5)
//soft i2c master
#define D5 (14)
#define D6 (12)
//soft serial
#define D7 (13)
#define D8 (15)
#endif

#ifdef ESP32
#define BAUD_RATE 115200
#else
#define BAUD_RATE 115200
#endif

SoftwareSerial swSer;

long lastRxAt = 0;

void setup() {

  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  
  Serial.begin(115200);

   pinMode(RM_SPEED, OUTPUT); 
   pinMode(LM_SPEED, OUTPUT); 
   pinMode(RM_DIR, OUTPUT); 
   pinMode(LM_DIR, OUTPUT); 
   pinMode(LED_BUILTIN, OUTPUT);
  
   swSer.begin(BAUD_RATE, SWSERIAL_8N1, D7, D8, false, 95, 95*(8+1));
   swSer.println("Connected");

  //dance
  digitalWrite(LM_DIR, HIGH);    
  digitalWrite(RM_DIR, LOW);    
  
}


void loop() {
 
 // if there's data available, read a packet
  long currMillis = millis();

  byte header;   
  
  int availableToRead = swSer.available();
  if (availableToRead > 0) 
  {
    if(readHeader(&swSer, header))
    {
      #ifdef DEBUG
        Serial.println("Got header");
      #endif

      lastRxAt = millis();
      
      switch(header)
      {
        case HDR_PING:
          //handleping();
          #ifdef DEBUG
            Serial.println("Got ping");
          #endif
        break;
          
        case HDR_CMD_DRIVELINE:
          #ifdef DEBUG
            Serial.println("Got driveline");
          #endif
          
          processDrivelineCommand();
        break;
      }
    }
  }
  
  if( (currMillis - lastRxAt) > 300){

      digitalWrite(LM_SPEED, 0);
      digitalWrite(RM_SPEED, 0); 
  }

  yield();
}

void processDrivelineCommand()
{

  byte packetBuffer[2] = {0};
  if(!readSoftSerial(&swSer, 2, packetBuffer))
  {
    return;
  }
  
  byte lmSpeed = packetBuffer[0];
  byte rmSpeed = packetBuffer[1];

  #ifdef DEBUG
    char data[255];
    sprintf(data,"LM: [%d] RM: [%d]", lmSpeed, rmSpeed);
    Serial.println(data);
  #endif

  //we plot anyway
  #ifndef DEBUG
    Serial.print(lmSpeed);
    Serial.print(" ");
    Serial.println(rmSpeed);
  #endif
  
  if(lmSpeed > 0){
    analogWrite(LM_SPEED, map(lmSpeed, 1,247, 800,1023));
    
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
    analogWrite(RM_SPEED, map(rmSpeed, 1,247, 800,1023));      

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

//reads the next byte and returns true if it's a header
bool readHeader(SoftwareSerial *ss, byte &header)
{
  header = ss->read();

  if ((header & HDR_QUALIFIER) == HDR_QUALIFIER)
  {
    return true;
  }
  header = 0;
  return false;
}

bool readSoftSerial(SoftwareSerial *ss, uint32_t  numBytesToRead, byte *buffer)
{
  unsigned long begin_time = millis();
  uint32_t numBytesRead = 0;

  do
  {
    if(ss->available() > 0)
    {
      buffer[numBytesRead++] = ss->read();
    }else{
      yield();
    }
    
  }while(
    numBytesRead < numBytesToRead
    &&
    (millis() - begin_time) < SERIAL_READ_TIMEOUT
  );

  if(numBytesRead == numBytesToRead)
  {
    return true;
  }
  return false;
}
