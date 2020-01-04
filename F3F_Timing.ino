/*
  F3F Timing
  use 2 pins as interrupt mode 
*/
#include <Wire.h>


#define SLAVE_ADDRESS 0x05
const byte ledPin = 4;
const byte baseAPin = 2;
const byte baseBPin = 3;

enum chronoStatus{
  WaitToLaunch,
  Launched,
  InStart,
  InProgress,
  Finished
  };

typedef struct{
  unsigned long lap[10];
  byte lapCount;
  chronoStatus runStatus; 
}chronoStr;

enum i2c_request{
  setStatus_InStart,
  getStatus,
  getLapCount,
  reset,
  getTime=10
  };

typedef struct{
  byte data[10];
  byte nbData;
}i2cStr;
  
volatile byte ledState = LOW;
volatile unsigned long time1=0;
volatile unsigned long oldtime=0;
volatile unsigned long starttime=0;
volatile byte oldbase=0;
volatile String timestr="";
volatile chronoStr chrono={0};
volatile i2cStr i2cReceive, i2cSend;
volatile unsigned long nbinterruptA=0, nbinterruptB=0;
volatile byte debug=false;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(ledPin, OUTPUT);
  //Initialize buttons pin in interrupt mode
  pinMode(baseAPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(baseAPin), baseA_Interrupt, FALLING);
  pinMode(baseBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(baseBPin), baseB_Interrupt, FALLING);
  //Initialize time process & chrono var.
  time1=0;
  oldtime=time1;  
  memset (&chrono, 0, sizeof(chrono));

  //for debug only set status Launched
  if (debug){
    chrono.runStatus=Launched;
  }

  //Initialize I2C link as slave with Rpi
  memset (&i2cReceive, 0, sizeof(i2cReceive));
  memset (&i2cSend, 0, sizeof(i2cSend));
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);  
  
  //Only for debug
  Serial.begin(57600);
  while(!Serial){
    if (debug){
      delay(100);
      Serial.println("F3F timer connected");
    }
  }
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(ledPin, ledState);
  ledState=false;
  if (debug){
    if (time1!=oldtime){
      Serial.print("Lap : ");
      Serial.println(chrono.lapCount);
      Serial.println((float)chrono.lap[chrono.lapCount-1]/1000);
      oldtime=time1;
      Serial.print("base A : ");
      Serial.print(nbinterruptA);
      Serial.print(", base B : ");
      Serial.println(nbinterruptB);
    }
  }
  delay (100);
}


// callback for received data
void receiveData(int byteCount){
  memset(&i2cReceive,0, sizeof(i2cReceive));
  while(Wire.available()) {
    i2cReceive.data[i2cReceive.nbData] = Wire.read();
    i2cReceive.nbData++;
  }
  if (debug){
    Serial.print("data received: ");
    for (byte i=0; i<i2cReceive.nbData; i++){
      Serial.print(i2cReceive.data[i]);
      Serial.print(", ");
    }
    Serial.println();
  }
}

// callback for sending data
void sendData(){
  memset(&i2cSend, 0, sizeof(i2cSend));
  switch(i2cReceive.data[0]){
    case setStatus_InStart:
      i2cSend.data[0]=setStatus_InStart;
      chrono.runStatus=InStart;
      i2cSend.data[1]=chrono.runStatus;
      i2cSend.nbData=2;
      break;
    case getStatus:
      i2cSend.data[0]=getStatus;
      i2cSend.data[1]=chrono.runStatus;
      i2cSend.nbData=2;
      break;
    case getLapCount:
      i2cSend.data[0]=getLapCount;
      i2cSend.data[1]=chrono.lapCount;
      i2cSend.nbData=2;
      break;
    case reset:
      memset(&chrono, 0, sizeof(chronoStr));
      i2cSend.data[0]=reset;
      i2cSend.nbData=1;
      break;
    case getTime:
    case getTime+1:
    case getTime+2:
    case getTime+3:
    case getTime+4:
    case getTime+5:
    case getTime+6:
    case getTime+7:
    case getTime+8:
    case getTime+9:
      i2cSend.data[0]=i2cReceive.data[0];
      memcpy(&i2cSend.data[1], &chrono.lap[i2cReceive.data[0]-getTime], sizeof(unsigned long));
      i2cSend.nbData=5;
      break;
    default:
      break;
  }
  
  for (byte i=0; i<i2cSend.nbData; i++){
    Wire.write(i2cSend.data[i]);
  }
  if (debug){
    Serial.print("Data Send : ");
    for (byte i=0; i<i2cSend.nbData; i++){
      Serial.print(i2cSend.data[i]);
      Serial.print(", ");
    }
    Serial.println();
  }
}

void baseA_Interrupt() {
  baseCheck(baseAPin);
  ledState=true;
  nbinterruptA++;
}

void baseB_Interrupt(){
  baseCheck(baseBPin);
  ledState=true;
  nbinterruptB++;
}

void baseCheck(byte base) {
  if (chrono.runStatus==InStart || chrono.runStatus==InProgress){
    if (oldbase==base & chrono.runStatus==InStart){
      starttime=millis();
      chrono.runStatus=InProgress;
      if (debug){
        Serial.print("In Progress\n");          
      }
    }
    if (oldbase!=base){
      if (chrono.runStatus==InProgress){
        time1=millis();
        chrono.lap[chrono.lapCount]=time1-starttime;
        chrono.lapCount++;
        if (chrono.lapCount>=10){
          chrono.runStatus=Finished;
        }
      }
    }
  }
  if (chrono.runStatus==Launched){
        chrono.runStatus=InStart;
        if (debug){
          Serial.print("In Start\n");          
        }
  }
  oldbase=base;
}
