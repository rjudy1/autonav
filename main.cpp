//Libraries to include
#include <Arduino.h>  //required in VsCode
#include "QuadEncoder.h"
#include <PWMServo.h>

//Pin Definitions
#define ApinL 7 //Left encoder A phase
#define BpinL 8 //Left encoder B phase
#define ApinR 2 //you get it
#define BpinR 3
#define buzzerPin 19 //Pin controlling buzzer
#define greenPin 20 //should be obvious
#define yellowPin 21
#define redPin 22
#define modeInputPin 18
#define pwmPinAng 14
#define pwmPinLin 15
#define errorLed 6
#define goodLed 5
//Constants Definitions
#define blinkPeriod 500 //controls how fast the LED blinks when robot is in auto mode
#define errorBlinkPeriod 50
#define goodBlinkPeriod 20
#define btSerial Serial4
#define extSerial Serial1

//enc(encoder number (1-4), A pin, B pin, pullups required (0/1), 4=??) //IDK what the 4 does, but it seems necessary...
QuadEncoder encL(1, ApinL, BpinL, 1, 4);
QuadEncoder encR(2, ApinR, BpinR, 1, 4);

//create servo objects
PWMServo servoAng;
PWMServo servoLin;

//Global Variables
unsigned long commTimer = 0; //used to stop robot if communication is lost for longer than serial_dead_time
int inputAng = 0;
int inputLin = 0;
int angPWM = 89;
int linPWM = 89;
bool errorReceived = false;
bool goodReceived = false;
unsigned long errorBlinkStartTime = 0;
unsigned long goodBlinkStartTime = 0;

void printEncoders() {
  //print the encoder counts with format "E,leftCount,rightCount,**"
  Serial.println("E,"+String(encL.read())+","+String(encR.read())+",**");
}
//Q,**
int parseSerial() {
  //this is C string stuff, it's confusing -_-
  char incomingString[16];
  byte i = 0;
  while(Serial.available() > 0) {
    incomingString[i] = Serial.read();
    i++;
    delayMicroseconds(50);
  }
  btSerial.println(incomingString);
  extSerial.println(incomingString);
  char* pch;
  char* chrt[4];
  int count = 0;
  pch = strtok (incomingString,",");
  while (pch != NULL)  {
    chrt[count] = pch;
    pch = strtok (NULL, ",");
    count++;
  }
  //return a number based on the received message
  char indicator = chrt[0][0];
  if(indicator == 'Q') {
    return 1;
  }
  else if(indicator == 'B') {
    if(atoi(chrt[1]))
      return 2;
    else
      return 3;
  }
  else if(indicator == 'G') {
    if(atoi(chrt[1]))
      return 4;
    else
      return 5;
  }
  else if(indicator == 'Y') {
    if(atoi(chrt[1]))
      return 6;
    else
      return 7;
  }
  else if(indicator == 'R') {
    if(atoi(chrt[1]))
      return 8;
    else
      return 9;
  }
  else if(indicator == 'M') {
    inputAng = atoi(chrt[1]);
    inputLin = atoi(chrt[2]);
    return 10;
  }
  else {
    return 0; //This should never happen, it means an invalid message was received
  }
}

void setup() {
  pinMode(greenPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(modeInputPin, INPUT_PULLUP);
  pinMode(errorLed, OUTPUT);
  pinMode(goodLed, OUTPUT);
  //serial used for USB communications to receive commands and report encoder counts
  Serial.begin(115200);
  extSerial.begin(115200);
  btSerial.begin(9600);
  encL.setInitConfig();
  encL.init();
  encR.setInitConfig();
  encR.init();
  servoAng.attach(pwmPinAng, 1000, 2000);
  servoLin.attach(pwmPinLin, 1000, 2000);
  servoAng.write(89);
  servoLin.write(89);
}

void loop() {
  while(!Serial.available()) {
    static bool redBlink = false; 
    redBlink = !digitalRead(modeInputPin);
    //blink the red light without using delays
    static unsigned long startTime = millis();
    if(!redBlink) {
      digitalWrite(redPin, HIGH);
    }
    if(redBlink && millis() - startTime >= blinkPeriod) {
      startTime = millis();
      static bool ledState = false;
      ledState = !ledState;
      digitalWrite(redPin, ledState);
    }
    if(errorReceived && millis() - errorBlinkStartTime >= errorBlinkPeriod) {
      errorReceived = false;
      digitalWrite(errorLed, LOW);
    }
    if(goodReceived && millis() - goodBlinkStartTime >= goodBlinkPeriod) {
      goodReceived = false;
      digitalWrite(goodLed, LOW);
    }
  }
  int sig = parseSerial();
  switch (sig) {
  case 0:
    errorReceived = true;
    errorBlinkStartTime = millis();
    digitalWrite(errorLed, HIGH);
    break;
  case 1:
    goodReceived = true;
    goodBlinkStartTime = millis();
    digitalWrite(goodLed, HIGH);
    printEncoders();
    break;
  case 2:
    goodReceived = true;
    goodBlinkStartTime = millis();
    digitalWrite(goodLed, HIGH);
    digitalWrite(buzzerPin, HIGH);
    break;
  case 3:
    goodReceived = true;
    goodBlinkStartTime = millis();
    digitalWrite(goodLed, HIGH);
    digitalWrite(buzzerPin, LOW);
    break;
  case 4:
    goodReceived = true;
    goodBlinkStartTime = millis();
    digitalWrite(goodLed, HIGH);
    digitalWrite(greenPin, HIGH);
    break;
  case 5:
    goodReceived = true;
    goodBlinkStartTime = millis();
    digitalWrite(goodLed, HIGH);
    digitalWrite(greenPin, LOW);
    break;
  case 6:
    goodReceived = true;
    goodBlinkStartTime = millis();
    digitalWrite(goodLed, HIGH);
    digitalWrite(yellowPin, HIGH);
    break;
  case 7:
    goodReceived = true;
    goodBlinkStartTime = millis();
    digitalWrite(goodLed, HIGH);
    digitalWrite(yellowPin, LOW);
    break;
  case 8:
    //Serial.println("Red light control no longer available");
    break;
  case 9:
    //Serial.println("Red light control no longer available");
    break;
  case 10:
    goodReceived = true;
    goodBlinkStartTime = millis();
    digitalWrite(goodLed, HIGH);
    angPWM = constrain(inputAng, 0, 179);
    linPWM = constrain(inputLin, 0, 179);
    servoAng.write(angPWM);
    servoLin.write(linPWM);
    break;
  default:
    break;
  }
}