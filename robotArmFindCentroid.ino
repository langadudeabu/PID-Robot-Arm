#include "uart.h"
#include "motor.h"
#include "Potentiometer.h"

float XerrorPercent;
float YerrorPercent;
char incomingByte;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  String message = "Starting Program...\n";
  Serial.println(message);

  GPIO_setup();
  fullStop();
  signalLight();
  analogReference(EXTERNAL);
}

void loop() {
  // put your main code here, to run repeatedly:
  incomingByte = findChar();
  commandHandler( incomingByte );
  
}
