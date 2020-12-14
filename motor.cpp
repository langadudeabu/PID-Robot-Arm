#include "Arduino.h"
#include "motor.h"
#include "uart.h"
#include "Potentiometer.h"
#include "PID.h"

int pot1 = getPot1(); // claw
int pot2 = getPot2(); // wrist
int pot3 = getPot3(); // elbow
int pot4 = getPot4(); // shoulder
int pot5 = getPot5(); // waist

int Direction1 = 0;
int Direction2 = 0;
int Direction3 = 0;
int Direction4 = 0;
int Direction5 = 0;
#define MAX_SPEED 255

// Motors 1 and 2
#define PWM1 2
#define PWM2 3
#define DIR1 24 //23
#define DIR2 25 //22

// Motors 3 and 4
#define PWM3 4
#define PWM4 5
#define DIR3 23 //25
#define DIR4 22 //24

// Motors 5 and 6
#define PWM5 6
#define PWM6 7
#define DIR5 27
#define DIR6 26

#define LIGHT 53
int light = LOW;

#define m1 1 // On MDD10A (1)
#define m2 2 // On MDD10A (2)
#define m3 3 // On MDD10A (3)
#define m4 4 // On MDD10A (4)
#define m5 5 // On MDD10A (5)

#define forward 1
#define reverse -1

int Speed = 80; // out of 255 (elbow)
int shoulderSpeed = 25; // percent
int claw_speed = 15; // percent
int waist_speed = 35; // percent

int getSpeed(void) { return Speed; }
int getShoulderSpeed(void) { return shoulderSpeed; }
int getClawSpeed(void) { return claw_speed; }
int getWaistSpeed(void) { return waist_speed; }

int getMAXSPEED(void) { return MAX_SPEED; }

void newSpeed(void)
{
  Serial.print("New Speed: ");
  Speed = readInt().toInt();
  Serial.println(Speed); 
}

void GPIO_setup()
{  
  pinMode(LIGHT, OUTPUT); // pin 53
  
  pinMode(PWM1, OUTPUT);   // pin 2
  pinMode(PWM2, OUTPUT);   // pin 3
  pinMode(DIR1, OUTPUT); // pin 23
  pinMode(DIR2, OUTPUT);  // pin 24

  pinMode(PWM3, OUTPUT);   // pin 4
  pinMode(PWM4, OUTPUT);   // pin 5
  pinMode(DIR3, OUTPUT); // pin 25
  pinMode(DIR4, OUTPUT);  // pin 26

  pinMode(PWM5, OUTPUT);   // pin 6
  pinMode(PWM6, OUTPUT);   // pin 7
  pinMode(DIR5, OUTPUT); // pin 27
//  pinMode(DIR6, OUTPUT);  // pin 28
}

void setSpeed1(int percent, int dir)
{
  if(percent == 0)
  {
    Serial.println("m1 Brake");
    analogWrite(PWM1, 0);
    return;
  }
  if(dir == forward)
  {
    Serial.println("m1 Forward");
    digitalWrite(DIR1, false);
    Direction1 = forward;
    Serial.println("forward");
  }
  if(dir == reverse)
  {
    Serial.println("m1 Reverse");
    digitalWrite(DIR1, true);
    Direction1 = reverse;
    Serial.println("reverse");
  }
    
  int duty = int( MAX_SPEED*( double(percent)/100.0 ) );
  analogWrite(PWM1, duty);
}

void setSpeed2(int percent, int dir)
{
  if(percent == 0)
  {
    //Serial.println("m2 Brake");
    analogWrite(PWM2, 0);
    return;
  }
  if(dir == forward)
  {
    //Serial.println("m2 Forward");
    digitalWrite(DIR2, false);
    Direction2 = forward;
  }
  if(dir == reverse)
  {
    //Serial.println("m2 Reverse");
    digitalWrite(DIR2, true);
    Direction2 = reverse;
  }
    
//  int duty = int( MAX_SPEED*( double(percent)/100.0 ) );
  int duty = percent;
  analogWrite(PWM2, duty);
}

void setSpeed3(int percent, int dir)
{
  if(percent == 0)
  {
    Serial.println("m3 Brake");
    analogWrite(PWM3, 0);
    return;
  }
  if(dir == forward)
  {
    Serial.println("m3 Forward");
    digitalWrite(DIR3, false);
    Direction3 = forward;
  }
  if(dir == reverse)
  {
    Serial.println("m3 Reverse");
    digitalWrite(DIR3, true);
    Direction3 = reverse;
  }
  int duty = int( MAX_SPEED*( double(percent)/100.0 ) );
  analogWrite(PWM3, duty);
}

void setSpeed4(int percent, int dir)
{
  if(percent == 0)
  {
//    Serial.println("m4 Brake");
    analogWrite(PWM4, 0);
    return;
  }
  if(dir == forward)
  {
//    Serial.println("m4 Forward");
    digitalWrite(DIR4, false);
    Direction4 = forward;
  }
  if(dir == reverse)
  {
//    Serial.println("m4 Reverse");
    digitalWrite(DIR4, true);
    Direction4 = reverse;
  }
//  int duty = int( MAX_SPEED*( double(percent)/100.0 ) );
  int duty = percent;
  analogWrite(PWM4, duty);
}

void setSpeed5(int percent, int dir)
{
  if(percent == 0)
  {
    Serial.println("m5 Brake");
    analogWrite(PWM5, 0);
    return;
  }
  if(dir == forward)
  {
    Serial.println("m5 Forward");
    digitalWrite(DIR5, true);
    Direction5 = forward;
  }
  if(dir == reverse)
  {
    Serial.println("m5 Reverse");
    digitalWrite(DIR5, false);
    Direction5 = reverse;
  }
  int duty = int( MAX_SPEED*( double(percent)/100.0 ) );
  analogWrite(PWM5, duty);
}

void changeSpeed(int percent, int dir, int motorNum)
{
  switch(motorNum)
  {
    case m1:
      setSpeed1(percent, dir);
      break;
    case m2:
      setSpeed2(percent, dir);
      break;
    case m3:
      setSpeed3(percent, dir);
      break;
    case m4:
      setSpeed4(percent, dir);
      break;
    case m5:
      setSpeed5(percent, dir);
  }
}

void brake(int motorNum)
{
  changeSpeed(0, forward, motorNum);
}

void fullStop()
{
  Serial.println("Full Stop.");
  brake(m1);
  brake(m2);
  brake(m3);
  brake(m4);
  brake(m5);  
}

void lightOff()
{
  //Serial.println("Turning Light OFF.");
  light = LOW;
  digitalWrite(LIGHT, light);
}

void lightOn()
{
  //Serial.println("Turning Light ON.");
  light = HIGH;
  digitalWrite(LIGHT, light);
}

void toggleLight()
{
  //Serial.println("Toggling Light.");
  light = !light;
  digitalWrite(LIGHT, light);
}

void flashLight()
{
  lightOn();
  delay(100);
  lightOff();
}

void signalLight()
{
  toggleLight();
  delay(250);
  toggleLight();
  delay(250);
  toggleLight();
  delay(250);
  toggleLight();
  delay(250);
}

void sleepPosition(void)
{
  int Kp1 = 10;
  int Kd1 = 4;

  int Kp2 = 10;
  int Kd2 = 4;

  int P1 = 0;
  int D1 = 0;
  int U1 = 0;
  
  int P2 = 0;
  int D2 = 0;
  int U2 = 0;

  int MaxSpeed = MAX_SPEED*getMaxSpeedPercent()/100;
  int MinSpeed = MAX_SPEED*getMinSpeedPercent()/100;

  int error1 = 0;
  int prevError1 = 0;
  int error2 = 0;
  int prevError2 = 0;
  
  int newSpeed1 = 0;
  int newSpeed2 = 0;

  int elbow = 0;
  int shoulder = 0;
  
  char inChar = 0;
  while(1)
  {
    inChar = findChar();
    if(inChar == 'q')
      break;
    elbow = readPot(pot3,0);
    shoulder = readPot(pot4,0);

    error1 = elbow - 4001;
    error2 = shoulder - 890;

    P1 = error1*Kp1;
    D1 = (error1 - prevError1)*Kd1;
    U1 = P1 + D1;

    P2 = error2*Kp2;
    D2 = (error2 - prevError2)*Kd2;
    U2 = P2 + D2;

    newSpeed1 = (int)(((double)abs(U1)/(double)getMaxU())*(MaxSpeed-MinSpeed)) + MinSpeed;
    newSpeed2 = (int)(((double)abs(U2)/(double)getMaxU())*(MaxSpeed-MinSpeed)) + MinSpeed;

    if(newSpeed1 > MaxSpeed)
      newSpeed1 = MaxSpeed;
    if(newSpeed2 > MaxSpeed)
      newSpeed2 = MaxSpeed;

    if(error1 < -getThresh())
    {
      if(Direction2 != reverse)
      {
        delay(500);
        Serial.println("m2 Reverse");
      }
      changeSpeed(newSpeed1, reverse, m2); // move the elbow down
    }
    if(error1 > getThresh())
    {
      if(Direction2 != forward)
      {
        delay(500);
        Serial.println("m2 Forward");
      }
      changeSpeed(newSpeed1, forward, m2); // raise the elbow up
    }
    if(error1 >= -getThresh() and error1 <= getThresh())
    {
      if(Direction2 != 0)
      {
        brake(m2);
        Direction2 = 0;
        Serial.println("m2 Brake");
      }
    }
    if(error2 < -getThresh())
    {
      if(Direction4 != reverse)
      {
        delay(500);
        Serial.println("m4 Forward");
      }
      changeSpeed(newSpeed2, forward, m4); // move the elbow down
    }
    if(error2 > getThresh())
    {
      if(Direction4 != forward)
      {
        delay(500);
        Serial.println("m4 Reverse");
      }
      changeSpeed(newSpeed2, reverse, m4); // raise the elbow up
    }
    if(error2 >= -getThresh() and error2 <= getThresh())
    {
      if(Direction4 != 0)
      {
        brake(m4);
        Direction4 = 0;
        Serial.println("m4 Brake");
      }
    }
  }
}

void motorCommandHandler(char incomingByte)
{
  switch(incomingByte)
  {
    case 'q':
        changeSpeed(getClawSpeed(), forward, m1); // claw
        break;
      case 'w':
        changeSpeed(getSpeed(), forward, m2); // new elbow (old wrist)
        break;
      case 'e':
        //changeSpeed(getSpeed(), forward, m3); // elbow
        break;
      case 'r':
        changeSpeed(getShoulderSpeed(), forward, m4); // shoulder
        break;
      case 't':
        changeSpeed(getWaistSpeed(), forward, m5); // waist
        break;
        
      case 'a':
        changeSpeed(getClawSpeed(), reverse, m1); // claw
        break;
      case 's':
        changeSpeed(getSpeed(), reverse, m2); // new elbow (old wrist)
        break;
      case 'd':
        //changeSpeed(getSpeed(), reverse, m3); // elbow
        break;
      case 'f':
        changeSpeed(getShoulderSpeed(), reverse, m4); // shoulder
        break;
      case 'g':
        changeSpeed(getWaistSpeed(), reverse, m5); // waist
        break;
  
      case 'x':
        fullStop();
        break;
        
      case 'n':
      newSpeed();
      break;
  }
}

void move2Position(int pos)
{
  const int numPoints = 200;
  int errors[numPoints] = {0};
  int newSpeeds[numPoints] = {0};
  int Pvalues[numPoints] = {0};
  int Dvalues[numPoints] = {0};
  int Uvalues[numPoints] = {0};
  double angles[numPoints] = {0};
  double times[numPoints] = {0};
  double angularVelocity[numPoints] = {0};
  int ind = 0;
  
  char inChar = 0;
  int elbow = 0;
  double angle = 0;
  double prevAngle = convert2Angle(readPot(pot3,0));
  int error = 0;
  int prevError = 0;
  double Time = 0;
  double angularVel = 0;
  int newSpeed = 0;
  int MaxSpeed = MAX_SPEED*getMaxSpeedPercent()/100;
  int MinSpeed = MAX_SPEED*getMinSpeedPercent()/100;
  int maxPotDiff = getElbowFlat()-getElbowUp();
  
  int P = 0;
  int D = 0;
  int U = 0;

  int startTime = millis();
  double prevTime = startTime;
  startTime = millis();
  while(1)
  {
    inChar = findChar();
    if(inChar == 'q')
      break;
    if(ind == 1)
    {
      startTime = millis();
      Serial.print("StartTime = ");
      Serial.println((double)startTime/1000.0);
    }
      
    elbow = readPot(pot3,0);
    Time = (double)(millis() - startTime)/1000.0;
    angle = convert2Angle(elbow);
    angularVel = (angle - prevAngle)/(Time - prevTime);
    prevAngle = angle;
    prevTime = Time;
    
    error = elbow - pos;
    
    P = error*getKp();
    D = (error - prevError)*getKd();
    U = P + D;

    prevError = error;
    
    //newSpeed = (int)(((double)abs(error)/(double)maxPotDiff)*(MaxSpeed-MinSpeed)) + MinSpeed;
    newSpeed = (int)(((double)abs(U)/(double)getMaxU())*(MaxSpeed-MinSpeed)) + MinSpeed;
    if(newSpeed > MaxSpeed)
      newSpeed = MaxSpeed;
    if(ind < numPoints)
    {
      errors[ind] = error;
      newSpeeds[ind] = newSpeed;
      Pvalues[ind] = P;
      Dvalues[ind] = D;   
      Uvalues[ind] = U;
      angles[ind] = angle;
      times[ind] = Time;
      angularVelocity[ind] = angularVel;
      ind++;
    }
    
    printPIDData(error, P, D, U, newSpeed, angle, Time, angularVel);
    Serial.println();
    if(error < -getThresh())
    {
      if(Direction2 != reverse)
      {
        delay(50);
        brake(m2);
        lightOff();
        delay(400);
        Serial.println("m2 Reverse");
      }
      changeSpeed(newSpeed, reverse, m2); // move the elbow down
    }
    if(error > getThresh())
    {
      if(Direction2 != forward)
      {
        delay(50);
        brake(m2);
        lightOff();
        delay(400);
        Serial.println("m2 Forward");
      }
      changeSpeed(newSpeed, forward, m2); // raise the elbow up
    }
    if(error >= -getThresh() and error <= getThresh())
    {
      if(Direction2 != 0)
      {
        brake(m2);
        Direction2 = 0;
        lightOn();
        Serial.println("m2 Brake");
      }
    }
  }

  lightOff();
  signalLight();
  brake(m2);
  Direction2 = 0;
  printPots();

//  for(int i = 0; i < ind; i++)
//  {
//    printPIDData(errors[i], Pvalues[i], Dvalues[i],
//                 Uvalues[i], newSpeeds[i], angles[i],
//                 times[i], angularVelocity[i]);
//  }
  printData(times, angles, angularVelocity, ind);
  return;
}

/*---------------------Speeds------------------*/

//void getNewSpeed()
//{
//  Serial.print("Enter a new Speed: ");
//  String newSpeed = readInt();
//  SPEED = newSpeed.toInt();
//  Serial.println(SPEED);
//}

//void getNewClawSpeed()
//{
//  Serial.print("Enter a new Claw Speed: ");
//  String newSpeed = readInt();
//  SPEED = newSpeed.toInt();
//  Serial.println(SPEED);
//}

//void changeWaistSpeed()
//{
//  Serial.println("New Waist Speed: ");
//  String newSpeed = readInt();
//  waist_speed = newSpeed.toInt();
//}

//void printWaistSpeed()
//{
//  Serial.print("Waist Speed = ");
//  Serial.println(waist_speed);
//}
