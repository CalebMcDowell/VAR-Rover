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


void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(5,OUTPUT);
  Serial.begin(9600);
  otto.init();
}

void loop() {
  while(!otto.getRxData()){
      if(otto.failsafe())
        digitalWrite(5,HIGH);
      else
        digitalWrite(5,LOW);
  }
  if(otto.channel(5) >= 1800){
      otto.arm();
  }
  else{
      otto.disarm();
  }
  if(otto.isArmed()){
      otto.drive();
  }
  otto.moveLeveler();
//  otto.printChannels();
}
