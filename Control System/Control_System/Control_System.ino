/*
  File: Control_System.ino

  VAR Rover Senior Design Project

  Arduino UNO Rev3 board
  Code to control all subsystems in rover including:
  - Drivetrain
  - Lift
  - Leveler
  - Power distribution
*/

#include "VARRover.h"

Rover otto;
char pwmPulse;


void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  otto.init();
}

void loop() {
  while(!otto.getRxData()){
      if(otto.failsafe())
        digitalWrite(LED_BUILTIN,HIGH);
      else
        digitalWrite(LED_BUILTIN,LOW);
  }
  
  if (otto.channel(3) > 1200) {
    pwmPulse = map(otto.channel(3), 172, 1811, 125, 254);
    digitalWrite(LED_BUILTIN,HIGH);
  }
  else
    digitalWrite(LED_BUILTIN,LOW);

}
