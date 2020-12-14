//#ifndef MOTOR_h
//#define MOTOR_h

int getPot1(void);
int getPot2(void);
int getPot3(void);
int getPot4(void);
int getPot5(void);

int getClawOpen(void);
int getClawClosed(void);
int getElbowUp(void);
int getElbowFlat(void);

void setClawOpen(int val);
void setClawClosed(int val);
void setElbowUp(int val);
void setElbowFlat(int val);

double convert2Angle(int adcVal);

int readPot(int pin, int printVal);

void printPots(void);
void printAngle(void);

void testPotentiometer(void);
void potentiometerCommandHandler(char incomingByte);
