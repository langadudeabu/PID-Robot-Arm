#include "Arduino.h"
#include "motor.h"
#include "uart.h"
#include "Potentiometer.h"
#include "PID.h"

#define POT1 A0 // claw
#define POT2 A1 // wrist
#define POT3 A2 // elbow
#define POT4 A3 // shoulder
#define POT5 A4 // waist

int CLAW_CLOSED = 3014;
int CLAW_OPEN = 1722;

//int ELBOW_UP = 1987;
//int ELBOW_FLAT = 3341;
int ELBOW_UP = 69;
int ELBOW_FLAT = 425;

int getPot1(void) { return POT1; }
int getPot2(void) { return POT2; }
int getPot3(void) { return POT3; }
int getPot4(void) { return POT4; }
int getPot5(void) { return POT5; }

int getClawOpen(void) { return CLAW_OPEN; }
int getClawClosed(void) { return CLAW_CLOSED; }
int getElbowUp(void) { return ELBOW_UP; }
int getElbowFlat(void) { return ELBOW_FLAT; }

void setClawOpen(int val) { CLAW_OPEN = val; }
void setClawClosed(int val) { CLAW_CLOSED = val; }
void setElbowUp(int val) { ELBOW_UP = val; }
void setElbowFlat(int val) { ELBOW_FLAT = val; }

double convert2Angle(int adcVal)
{
  int zero = ELBOW_FLAT;
  int ninety = ELBOW_UP;
  
  double range = ninety - zero;
  double diff = adcVal - zero;
  
  double angle = (double)90.0*(double)((double)diff/(double)range);
  
  return angle;
}

int readPot(int pin, int printVal)
{
  //analogReadResolution(12);
  int pot = 0;
  int sum = 0;
  int numTests = 150;
  for(int i = 0; i < numTests; i++)
  {
    pot = analogRead(pin);
//    if(pot < 0)
//      pot += 440;
    sum += pot;
  }

  int avgVal = (int)((double)sum/(double)numTests);
  if(avgVal < 30)
  {
    avgVal += 440;
  }

  if(printVal)
  {
    Serial.print("Potentiometer Value: ");
    Serial.println(avgVal);
  }
  
  return avgVal;
}

void printPots(void)
{
  readPot(POT1,1);
  readPot(POT2,1);
  readPot(POT3,1);
  readPot(POT4,1);
//  readPot(POT5,1);
}

void printAngle(void)
{
  double angle = convert2Angle(readPot(POT3,0));
  Serial.print("\nAngle = ");
  Serial.print(angle);
  Serial.println(" degrees");
}

void testPotentiometer(void)
{
  char inChar = 0;

  while(1)
  {
    inChar = findChar();
    motorCommandHandler(inChar);
    if(inChar == 'Q')
    {
      fullStop();
      break;
    }
    int potVal = readPot(POT3,0);
    Serial.print("Elbow = ");
    Serial.println(potVal);
    delay(300);
  }
}

void potentiometerCommandHandler(char incomingByte)
{
  switch(incomingByte)
  {
    case 'P':
      printErrors();
      break;
    case 'J':
      printPots();
      break;
    case 'o':
      printAngle();
      break;
    case 'C':
      testPotentiometer();
      break;  
  }
}
