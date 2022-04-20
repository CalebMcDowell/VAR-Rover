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

byte armCount = 0;

void setup() {
  delay(1000);
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.begin(9600);
  otto.init();
}

void loop() {  
  //check if rover in operational state, checks functions from LEFT to RIGHT
  while(!otto.getRxData() || !otto.getVoltages() || !otto.getRovAngles() || otto.getRoverError()){
    if(otto.getRoverError()){
      otto.disarm();
      otto.displayLCD();
    }
//    otto.printChannels();
  }
  //determine arm/disarm state
  if((otto.isArmed()==1) ^ (otto.channel(5)==1811)) //XOR
    armCount++;
  else
    armCount=0;
  if(armCount>2){
    if(otto.isArmed())  otto.disarm();
    else                otto.arm();
    armCount=0;
  }
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
