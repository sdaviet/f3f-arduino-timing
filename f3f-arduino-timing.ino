/*
  F3F Timing
  use 2 pins as interrupt mode
*/
#include <Wire.h>

#define SLAVE_ADDRESS 0x05
const byte BASEAPIN = 2;
const byte BASEBPIN = 3;
const byte VOLTAGEPIN = A1;
const byte BUZZERPIN = 12;
const byte LOOPDELAY = 100;

enum chronoStatus {
  InWait = 0,
  WaitLaunch,
  Launched,
  InStart,
  InProgressA,
  InProgressB,
  WaitAltitude,
  Finished
};

typedef struct {
  byte runStatus;
  int analogVoltage;
  byte lapCount;
  unsigned long lap[10];

  unsigned long time1;
  unsigned long oldtime;
  unsigned long starttime;
  unsigned long startaltitudetime;
} chronoStr;

enum i2c_request {
  setStatus = 1,
  setBuzzerTime,
  setRebundBtn,
  eventBaseA,
  reset,
  getData,
  getData1
};

typedef struct {
  byte data[10];
  byte nbData;
} i2cReceiveStr;

typedef struct {
  byte data[30];
  byte nbData;
} i2cSendStr;

typedef struct {
  unsigned int Time;  //in milliseconds
  int Count;        //in milliseconds
  int Cmd;         //blink number
  byte State;   //buzzer state
  byte Pin;
} buzzerStr;

typedef struct {
  unsigned long old_event;
  unsigned long rebundBtn_time;
  unsigned long nbInterrupt;
  byte Pin;
} baseEventStr;

typedef struct {
  int readTime;
  int count;
  int rawData;
  byte Pin;
} analogStr;

typedef struct {
  byte state;
  byte received;
} debugStr;


volatile chronoStr chrono = {0};
volatile i2cReceiveStr i2cReceive = {0};
volatile i2cSendStr i2cSend = {0};
volatile baseEventStr baseA = {0}, baseB = {0};
volatile analogStr accu = {0};
volatile buzzerStr buzzer = {0};
volatile buzzerStr led = {0};
volatile debugStr debug = {0};
volatile unsigned int i = 0;

void(* resetFunc) (void) = 0;//declare reset function at address 0
void baseA_Interrupt(void);
void baseB_Interrupt(void);
void sendData(void);
void receiveData(int byteCount);
void baseCheck(byte base);
void debugRun(void);
void analogRun(void);
void buzzerRun(buzzerStr *data);
void baseCheck(byte base);

// the setup function runs once when you press reset or power the board
void setup() {
  //Initialize chrono var.
  memset (&debug, 0, sizeof(debug));
  memset (&chrono, 0, sizeof(chrono));
  chrono.analogVoltage = 900; //Initalize @12V for the first measurements
  //Initialize I2C link as slave with Rpi
  memset (&i2cReceive, 0, sizeof(i2cReceive));
  memset (&i2cSend, 0, sizeof(i2cSend));
  memset (&buzzer, 0, sizeof(buzzer));
  memset (&led, 0, sizeof(led));
  memset (&accu, 0, sizeof(accu));

  buzzer.Time = 500;
  buzzer.Pin = BUZZERPIN;

  led.Time = 500;
  led.Pin = LED_BUILTIN;

  accu.Pin = VOLTAGEPIN;
  accu.readTime = 2000;

  memset (&baseA, 0, sizeof(baseA));
  baseA.rebundBtn_time = 200;
  baseA.Pin = BASEAPIN;

  memset (&baseB, 0, sizeof(baseB));
  baseB.rebundBtn_time = 200;
  baseB.Pin = BASEBPIN;

  // initialize digital pin LED_BUILTIN and buzzer PIN as an output.
  pinMode(buzzer.Pin, OUTPUT);
  pinMode(led.Pin, OUTPUT);
  //Initialize buttons pin in interrupt mode
  pinMode(baseA.Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(baseA.Pin), baseA_Interrupt, FALLING);
  pinMode(baseB.Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(baseB.Pin), baseB_Interrupt, FALLING);

  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  //Only for debug
  Serial.begin(57600);
  while (!Serial) {
    delay(100);
    Serial.println("F3F timer connected");
  }
}

// the loop function runs over and over again forever
void loop() {
  debugRun();
  analogRun();
  buzzerRun(&buzzer);
  if (chrono.runStatus == WaitAltitude) {
    baseCheck(0);
  }
  delay (LOOPDELAY);
}


void debugRun(void) {
  if (Serial.available() > 0) {
    debug.received = Serial.read();
    switch (debug.received) {
      case 'd':
        led.Cmd = -1;
        Serial.println("buzzer :");
        Serial.print("cmd ");
        Serial.println(buzzer.Cmd);
        Serial.print("time ");
        Serial.println(buzzer.Time);
        Serial.print("state ");
        Serial.println(buzzer.State);
        Serial.println("led : ");
        Serial.print("cmd ");
        Serial.println(led.Cmd);
        Serial.print("time ");
        Serial.println(led.Time);
        Serial.print("state ");
        Serial.println(led.State);
        break;
      case 'r':
        resetFunc();
        break;
      case 's':
        Serial.print("status : ");
        Serial.println (chrono.runStatus);
        break;
      case 'v':
        Serial.print("voltage : ");
        Serial.println(chrono.analogVoltage);
        break;
      case 'l':
        Serial.print("Lap count : ");
        Serial.println(chrono.lapCount);
        for (i = 0; i < chrono.lapCount; i++) {
          Serial.print("Lap : ");
          Serial.println(i);
          Serial.println((float)chrono.lap[i] / 1000);
        }
        break;
      case 'b':
        Serial.println("base A :");
        Serial.print("rebund time : ");
        Serial.println(baseA.rebundBtn_time);
        Serial.print("nb interrrupt : ");
        Serial.println(baseA.nbInterrupt);
        Serial.println("base B :");
        Serial.print("rebund time : ");
        Serial.println(baseA.rebundBtn_time);
        Serial.print("nb interrrupt : ");
        Serial.println(baseB.nbInterrupt);

      default:
        break;
    }
  }
  buzzerRun(&led);
}

void analogRun(void)
{
  if (accu.count > accu.readTime) {
    accu.rawData = analogRead(accu.Pin);
    chrono.analogVoltage = accu.rawData;
    accu.count = 0;
  } else {
    accu.count += LOOPDELAY;
  }
}

void buzzerRun(buzzerStr *data) {
  if (data->Cmd != 0) {
    if (data->State == true & data->Count <= data->Time) {
      digitalWrite(data->Pin, HIGH);
      data->Count += LOOPDELAY;
      if (data->Count >= data->Time) {
        data->State = false;
        data->Count = 0;
      }
    }
    if (data->State == false & data->Count <= data->Time) {
      digitalWrite(data->Pin, LOW);
      data->Count += LOOPDELAY;
      if (data->Count >= data->Time) {
        data->State = false;
        data->Count = 0;
        if (data->Cmd != 0) {
          if (data->Cmd > 0) {
            data->Cmd -= 1;
          }
          data->State = true;
        }
      }
    }
  }
}

// callback for received data
void receiveData(int byteCount) {
  memset(&i2cReceive, 0, sizeof(i2cReceive));
  while (Wire.available() and i2cReceive.nbData < sizeof (i2cReceive.data)) {
    i2cReceive.data[i2cReceive.nbData] = Wire.read();
    i2cReceive.nbData++;
  }
  if (i2cReceive.data[0]<setStatus and i2cReceive.data[0]>getData1) {
    for (i = 0; i < i2cReceive.nbData; i++) {
      Serial.println(i2cReceive.data[i2cReceive.nbData]);
    }
  }
  switch (i2cReceive.data[0]) {
    case setStatus:
      chrono.runStatus = i2cReceive.data[1];
      break;
    case eventBaseA:
      baseCheck(BASEAPIN);
      break;
    case setBuzzerTime:
      buzzer.Time = (i2cReceive.data[1] & 0xff) | ((i2cReceive.data[2] << 8) & 0xff00);
      break;
    case setRebundBtn:
      baseA.rebundBtn_time = (i2cReceive.data[1] & 0xff) | ((i2cReceive.data[2] << 8) & 0xff00);
      baseB.rebundBtn_time = (i2cReceive.data[1] & 0xff) | ((i2cReceive.data[2] << 8) & 0xff00);
      break;
    case reset:
      memset(&chrono, 0, sizeof(chronoStr));
      break;
    default:
      break;
  }
}

// callback for sending data
void sendData() {
  memset(&i2cSend, 0, sizeof(i2cSend));
  switch (i2cReceive.data[0]) {
    case getData:
      i2cSend.data[0] = chrono.runStatus;
      memcpy(&i2cSend.data[1], &chrono.analogVoltage, 2);
      i2cSend.data[3] = chrono.lapCount;
      memcpy(&i2cSend.data[4], chrono.lap, 12);
      i2cSend.nbData = 16;
      break;
    case getData1:
      memcpy(i2cSend.data, &chrono.lap[3], 32);
      i2cSend.nbData = 32;
      break;
    default:
      break;
  }
  Wire.write((byte *)i2cSend.data, i2cSend.nbData);
}

void baseA_Interrupt(void) {
  if ((baseA.old_event + baseA.rebundBtn_time) < millis()) {
    baseCheck(baseA.Pin);
    baseA.old_event=millis();
    baseA.nbInterrupt++;
  }
}

void baseB_Interrupt(void) {
  if ((baseB.old_event + baseB.rebundBtn_time) < millis()) {
    baseCheck(baseB.Pin);
    baseB.old_event=millis();
    baseB.nbInterrupt++;
  }
}

void baseCheck(byte base) {
  switch (chrono.runStatus) {
    case Launched :
      if (BASEAPIN == base) {
        chrono.runStatus = InStart;
        buzzer.Cmd = 1;
      }
      break;
    case InStart:
      if (BASEAPIN == base) {
        buzzer.Cmd = 1;
        chrono.time1 = millis();
        chrono.oldtime = chrono.time1;
        chrono.lapCount=0;
        chrono.runStatus = InProgressB;
      }
      break;
    case InProgressA:
      if (base == BASEAPIN) {
        chrono.time1 = millis();
        chrono.lap[chrono.lapCount] = chrono.time1 - chrono.oldtime;
        chrono.lapCount++;

        chrono.oldtime = chrono.time1;
        buzzer.Cmd = 1;
        if (chrono.lapCount >= 10) {
          chrono.runStatus = WaitAltitude;
          chrono.startaltitudetime = millis();
          buzzer.Cmd = 3;
        } else {
          chrono.runStatus = InProgressB;
        }
      }
      break;
    case InProgressB:
      if (base == BASEBPIN) {
        chrono.time1 = millis();
        chrono.lap[chrono.lapCount] = chrono.time1 - chrono.oldtime;
        chrono.oldtime = chrono.time1;
        chrono.lapCount++;
        buzzer.Cmd = 1;
        chrono.runStatus = InProgressA;
        if (chrono.lapCount == 9) {
          buzzer.Cmd = 2;
        }
      }else{
        if (base == BASEAPIN and chrono.lapCount==0){
          buzzer.Cmd = 1;
        }
      }
      break;
    case WaitAltitude:
      if ((chrono.startaltitudetime + 5000) < millis()) {
        buzzer.Cmd = 3;
        chrono.runStatus = Finished;
      }
      break;
    default:
      buzzer.Cmd = 1;
      break;
  }
}
