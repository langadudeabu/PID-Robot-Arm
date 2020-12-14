//#ifndef MOTOR_h
//#define MOTOR_h

int getSpeed(void);
int getShoulderSpeed(void);
int getClawSpeed(void);
int getWaistSpeed(void);
int getMAXSPEED(void);
void newSpeed(void);

void GPIO_setup(void);

void setSpeedM1(int percent, const int dir);
void setSpeedM2(int percent, const int dir);
void setSpeedM3(int percent, const int dir);
void setSpeedM4(int percent, const int dir);
void setSpeedM5(int percent, const int dir);
void changeSpeed(int percent, const int dir, const int motorNum);

void brake(const int motorNum);
void fullStop(void);

void lightOff(void);
void lightOn(void);
void toggleLight(void);
void flashLight(void);
void signalLight(void);

void sleepPosition(void);
void move2Position(int pos);
void motorCommandHandler(char incomingByte);

//Speeds
//void getNewSpeed();
//void getNewClawSpeed();
//void changeWaistSpeed();
//void printWaistSpeed();
