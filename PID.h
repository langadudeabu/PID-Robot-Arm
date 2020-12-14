//#ifndef MOTOR_h
//#define MOTOR_h

int getKp(void);
int getKd(void);
int getMaxSpeedPercent(void);
int getMinSpeedPercent(void);
int getThresh(void);
int getMaxU(void);

void setKp(int val);
void setKd(int val);
void setMaxSpeedPercent(int val);
void setMinSpeedPercent(int val);
void setThresh(int val);
void setMaxU(int val);

void newKp(void);
void newKd(void);
void newMaxSpeed(void);
void newMinSpeed(void);
void newThresh(void);
void newMaxU(void);

void printPIDValues(void);
void printErrors(void);
void printData(double *times, double *angles,
               double *angularVelocities, int len);
void printPIDData(int error, int P, int D, int U,
                  int newSpeed, double angle, double Time,
                  double angVel);
void PIDCommandHandler(char incomingByte);
