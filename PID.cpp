#include "Arduino.h"
#include "motor.h"
#include "uart.h"
#include "Potentiometer.h"
#include "PID.h"

int Kp = 7;
int Kd = 0;
int MaxSpeedPercent = 30;
int MinSpeedPercent = 5;
int thresh = 1;
//int maxU = 12000;
int maxU = 1200;

int getKp(void) { return Kp; }
int getKd(void) { return Kd; }
int getMaxSpeedPercent(void) { return MaxSpeedPercent; }
int getMinSpeedPercent(void) { return MinSpeedPercent; }
int getThresh(void) { return thresh; }
int getMaxU(void) { return maxU; }

void setKp(int val) { Kp = val; }
void setKd(int val) { Kd = val; }
void setMaxSpeedPercent(int val) { MaxSpeedPercent = val; }
void setMinSpeedPercent(int val) { MinSpeedPercent = val; }
void setThresh(int val) { thresh = val; }
void setMaxU(int val) { maxU = val; }

void newKp(void)
{
  Serial.print("New Kp: ");
  Kp = readInt().toInt();
}

void newKd(void)
{
  Serial.print("New Kd: ");
  Kd = readInt().toInt();
}

void newMaxSpeed(void)
{
  Serial.print("New MaxSpeed: ");
  MaxSpeedPercent = readInt().toInt();
}

void newMinSpeed(void)
{
  Serial.print("New MinSpeed: ");
  MinSpeedPercent = readInt().toInt();
}

void newThresh(void)
{
  Serial.print("New Thresh: ");
  thresh = readInt().toInt();
}

void newMaxU(void)
{
  Serial.print("New MaxU: ");
  maxU = readInt().toInt();
}

void printPIDValues(void)
{
  Serial.print("Kp = ");
  Serial.println(Kp);
  Serial.print("Kd = ");
  Serial.println(Kd);
}

void printErrors(void)
{
  int elbow = readPot(getPot3(),0);
  int error1 = elbow - getElbowUp();
  Serial.print("Error1 = ");
  Serial.println(error1);
  int error2 = elbow - getElbowFlat();
  Serial.print("Error2 = ");
  Serial.println(error2); 
}

void printData(double *times, double *angles, double *angularVelocities, int len)
{
  Serial.println("Time\tAngle\tAngular Velocity");
  for(int i = 0; i < len; i++)
  {
    Serial.print(times[i]);
    Serial.print("\t");
    Serial.print(angles[i]);
    Serial.print("\t");
    Serial.println(angularVelocities[i]);
  }
}

void printPIDData(int error, int P, int D, int U, int newSpeed, double angle, double Time, double angVel)
{
  Serial.print("Time:");
  Serial.print(Time);
  
  Serial.print(" error:");
  Serial.print(error);
  Serial.print("(");
  Serial.print((double)(100*(double)error/(double)(getElbowFlat()-getElbowUp())));
  Serial.print("%)");
  
  Serial.print(" P:");
  Serial.print(P);
  
  Serial.print(" D:");
  Serial.print(D);
  
  Serial.print(" U:");
  Serial.print(U);
  Serial.print("(");
  Serial.print((double)(100.0*(double)U/(double)maxU));
  Serial.print("%)");
  
  Serial.print(" speed:");
  Serial.print(newSpeed);
  Serial.print("(");
  Serial.print((double)(100.0*(double)newSpeed/(double)getMAXSPEED()));
  Serial.print("%)");

  Serial.print(" Angle:");
  Serial.print(angle);

  Serial.print(" AngVel:");
  Serial.println(angVel);
}

void PIDCommandHandler(char incomingByte)
{
  switch(incomingByte)
  {
    case 'v':
      newKp();
      break;
    case 'b':
      newKd();
      break;
    case 'p':
      printPIDValues();
      break;
    case 'l':
      newMaxSpeed();
      break;
    case 'L':
      newMinSpeed();
      break;
    case 'k':
      newThresh();
      break;
    case 'u':
      newMaxU();
      break;
  }
}
