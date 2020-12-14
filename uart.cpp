#include "Arduino.h"
#include "uart.h"
#include "motor.h"
#include "Potentiometer.h"
#include "PID.h"

int Pot1 = getPot1(); // claw
int Pot2 = getPot2(); // wrist
int Pot3 = getPot3(); // elbow
int Pot4 = getPot4(); // shoulder
int Pot5 = getPot5(); // waist

const int M1 = 1; // On MDD10A (1)
const int M2 = 2; // On MDD10A (2)
const int M3 = 3; // On MDD10A (3)
const int M4 = 4; // On MDD10A (4)
const int M5 = 5; // On MDD10A (5)

const int FOR = 1;
const int REV = -1;

int SPEED = getSpeed(); // as a percent
int SHOULDER_SPEED = getShoulderSpeed();
int CLAW_SPEED = getClawSpeed();
int WAIST_SPEED = getWaistSpeed();

String readline()
{
  String temp = "";
  char inChar= 'l';
  Serial.flush();
  char t = Serial.read();
  while(true)
  {
    while(Serial.available() > 0)
    {
      inChar = Serial.read();
      if(inChar == '\n')
      {
        Serial.println(temp);
        return temp;
      }
      temp += inChar;
    }
  }
}

String readFloat()
{
  String temp = "";
  char inChar= 'l';
  Serial.flush();
  char t = Serial.read();
  while(1)
  {
    while(Serial.available() > 0)
    {
      inChar = Serial.read();
      if(inChar == '\n')
      {
        Serial.println(temp);
        return temp;
      }
      if(isDigit(inChar) || inChar == '.')
      {
        temp += inChar;
      }  
    }
  }
}

String readInt()
{
  String temp = "";
  char inChar= 'l';
  Serial.flush();
  char t = Serial.read();
  while(true)
  {
    while(Serial.available() > 0)
    {
      inChar = Serial.read();
      if(inChar == '\n')
      {
        Serial.println(temp);
        return temp;
      }
      if(isDigit(inChar))
        temp += inChar;
    }
  }
}

char findChar()
{
  char incomingByte; // reads 1 letter only at a time...
  if(Serial.available() > 0)
  {
    incomingByte = Serial.read();
    Serial.println(incomingByte);// echo
    return incomingByte;
  }
  return 0;
}

void commandHandler(char incomingByte)
{
  motorCommandHandler(incomingByte);
  potentiometerCommandHandler(incomingByte);
  PIDCommandHandler(incomingByte);
  switch(incomingByte)
  {
    case 'T':
      toggleLight();
      break;
    case '0':
      lightOff();
      break;
    case '1':
      lightOn();
      break;
    case 'H':
      move2Position(getElbowUp());
      break;
    case 'h':
      move2Position(getElbowFlat());  
      break;
    case 'Z':
      sleepPosition();
      break;
  }
}
