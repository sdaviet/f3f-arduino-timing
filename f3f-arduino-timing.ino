/*
  F3F Timing
  use 2 pins as interrupt mode 
*/
#include <Wire.h>

#define SLAVE_ADDRESS 0x05
const byte baseAPin = 2;
const byte baseBPin = 3;
const byte voltagePin = A1;
const byte buzzerPin = 12;
const byte loopDelay = 100;

enum chronoStatus{
  InWait=0,
  WaitLaunch,
  Launched,
  InStart,
  InProgress,
  WaitAltitude,
  Finished
};

typedef struct{
  byte runStatus; 
  int analogVoltage;
  byte lapCount;
  unsigned long lap[10];
}chronoStr;

enum i2c_request{
  setStatus=1,
  setBuzzerTime,
  setRebundBtn,
  reset,
  getData,
  getData1
};

typedef struct{
  byte data[10];
  byte nbData;
}i2cReceiveStr;

typedef struct{
  byte data[30];
  byte nbData;
}i2cSendStr;

volatile unsigned long time1=0;
volatile unsigned long oldtime=0;
volatile unsigned long starttime=0;
volatile unsigned long startaltitudetime=0;

volatile unsigned long oldBaseA_event=0, oldBaseB_event=0, rebundBtn_time=0;
volatile byte oldbase=0;
volatile String timestr="";
volatile chronoStr chrono={0};
volatile i2cReceiveStr i2cReceive={0};
volatile i2cSendStr i2cSend={0};
volatile unsigned long nbinterruptA=0, nbinterruptB=0;
volatile byte buzzerCmd=0;
volatile unsigned int buzzerTime=500;  //in milliseconds
volatile int buzzerCount=0;   //in milliseconds
volatile int analogReadTime=2000;
volatile int analogCount=0;
volatile byte buzzerState=false;
volatile byte debug=false;
volatile byte debuglap=false;


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN and buzzer PIN as an output.
  pinMode(buzzerPin, OUTPUT);
  //Initialize buttons pin in interrupt mode
  pinMode(baseAPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(baseAPin), baseA_Interrupt, RISING);
  pinMode(baseBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(baseBPin), baseB_Interrupt, RISING);
  //Initialize time process & chrono var.
  time1=0;
  oldtime=time1;  
  memset (&chrono, 0, sizeof(chrono));
  chrono.analogVoltage=900;   //Initalize @12V for the first measurements
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
    delay(100);
    if (debug){
      Serial.println("F3F timer connected");
    }
  }
}

// the loop function runs over and over again forever
void loop() {
  debugRun();
  analogRun();
  buzzerRun();
  if (chrono.runStatus==WaitAltitude){
    baseCheck(0);
  }
  delay (loopDelay);
}


void debugRun(void)
{
  if (debug){
    if (debuglap=true){
      Serial.print("Lap : ");
      Serial.println(chrono.lapCount);
      Serial.println((float)chrono.lap[chrono.lapCount-1]/1000);

      Serial.print("base A : ");
      Serial.print(nbinterruptA);
      Serial.print(", base B : ");
      Serial.println(nbinterruptB);
      Serial.print("nb buzzer : ");
      Serial.println(buzzerCmd);
      debuglap=false;
    }
  }
}

void analogRun(void)
{
  if (analogCount>analogReadTime){
    chrono.analogVoltage=analogRead(voltagePin);
    analogCount=0;
  }else{
    analogCount+=loopDelay;
  }
}

void buzzerRun(void){
  if (buzzerCmd>0){
    if (buzzerState==true & buzzerCount<=buzzerTime){
      digitalWrite(buzzerPin, HIGH);
      buzzerCount+=loopDelay;
      if (buzzerCount>=buzzerTime){
        buzzerState=false;
        buzzerCount=0;        
      }
    }
    if(buzzerState==false & buzzerCount<=buzzerTime){
      digitalWrite(buzzerPin, LOW);
      buzzerCount+=loopDelay;
      if (buzzerCount>=buzzerTime){
        buzzerState=false;
        buzzerCount=0;        
        if (buzzerCmd>0){
          buzzerSet(buzzerCmd-1);
        }
      }
    }
  }
}

void buzzerSet(byte cmd)
{
  buzzerState=cmd>0;
  buzzerCount=0;
  buzzerCmd=cmd;
}



// callback for received data
void receiveData(int byteCount){
  memset(&i2cReceive,0, sizeof(i2cReceive));
  while(Wire.available() and i2cReceive.nbData<sizeof (i2cReceive.data)) {
    i2cReceive.data[i2cReceive.nbData] = Wire.read();
    i2cReceive.nbData++;
  }
  
  
  switch(i2cReceive.data[0]){
    case setStatus:
      if (chrono.runStatus==InStart or chrono.runStatus==InProgress){
        baseCheck(baseAPin);
      }else{
        chrono.runStatus=i2cReceive.data[1];
      }
      if (debug){
        Serial.print("I2c setStatus : ");
        Serial.println(chrono.runStatus);        
      }
      break;
    case setBuzzerTime:
      buzzerTime=(i2cReceive.data[1]&0xff)|((i2cReceive.data[2]<<8)&0xff00);
      if (debug){
        Serial.println("setBuzzer");
        Serial.print("time : ");
        Serial.println(buzzerTime);
      }
      break;
    case setRebundBtn:
      rebundBtn_time=(i2cReceive.data[1]&0xff)|((i2cReceive.data[2]<<8)&0xff00);
      break;
    case reset:
      if (debug){
        Serial.println("reset");
      }
      memset(&chrono, 0, sizeof(chronoStr));
      startaltitudetime=0;
      break;
    default:
      break;
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
//  memset(&i2cSend, 0, sizeof(i2cSend));
   switch(i2cReceive.data[0]){
    case getData:
      i2cSend.data[0]=chrono.runStatus;
      memcpy(&i2cSend.data[1], &chrono.analogVoltage,2);
      i2cSend.data[3]=chrono.lapCount;
      memcpy(&i2cSend.data[4], chrono.lap,12);
      i2cSend.nbData=16;
      break;
    case getData1:
      memcpy(i2cSend.data, &chrono.lap[3], 32);
      i2cSend.nbData=32;
      break;
  default:
      break;
  }
  Wire.write((byte *)i2cSend.data, i2cSend.nbData);
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
  if ((oldBaseA_event + rebundBtn_time) < millis()){
    oldBaseA_event = millis();
    baseCheck(baseAPin);
    nbinterruptA++;
  }
}

void baseB_Interrupt(){
  if ((oldBaseB_event + rebundBtn_time) < millis()){
    oldBaseB_event = millis();
    baseCheck(baseBPin);
    nbinterruptB++;
  }
}

void baseCheck(byte base) {
  switch(chrono.runStatus){
    case Launched :
      if (baseAPin==base){
        chrono.runStatus=InStart;
        buzzerSet(1);
      }
      break;
    case InStart:
      if (baseAPin==base){
        starttime=millis();
        oldtime=starttime;
        buzzerSet(1);
        chrono.runStatus=InProgress;
      }
      break;
    case InProgress:
      if (oldbase!=base){
        time1=millis();
        chrono.lap[chrono.lapCount]=time1-oldtime;
        oldtime=time1;
        chrono.lapCount++;
        buzzerSet(1);
        if (chrono.lapCount==9){
          buzzerSet(2);
        }
        if (chrono.lapCount>=10){
          chrono.runStatus=WaitAltitude;
          startaltitudetime=millis();
          buzzerSet(3);
        }
      }
      break;
     case WaitAltitude:
        if ((startaltitudetime+5000)<millis()){
          buzzerSet(3);
          chrono.runStatus=Finished;
        }
      break;
    default:
      break;  
  }
  oldbase=base;        
}
