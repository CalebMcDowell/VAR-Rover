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
  Serial.begin(9600);
  otto.init();
}

void loop() {
  //check if rover in operational state
  while(otto.getRoverError() || !otto.getRxData() || !otto.getVoltages() || !otto.getRovAngles()){
      if(otto.getRoverError()){
        otto.disarm();
        otto.dispError();
      }
  }
  //determine arm/disarm state
  if(otto.channel(5) >= 1800) otto.arm();
  else  otto.disarm();
  //if armed, rover movement allowed
  if(otto.isArmed()){
      otto.drive();
      otto.moveLeveler();
      otto.lift();
  }
  //update LCD screen (only every timeDelay)
  otto.displayLCD();
  
//  otto.printChannels();
}
