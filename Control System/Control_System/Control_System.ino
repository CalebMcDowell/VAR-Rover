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

std::array<int16_t, bfs::SbusRx::NUM_CH()> channel; //Array for storing received data
char pwmPulse;

void loop() {
    if(getRxData(channel));

    if(channel[3]>150 && channel[3]<1900){
      pwmPulse = map(channel[3],172,1811,125,254);
      analogWrite(3,pwmPulse);
    }
    else
      analogWrite(3,0);
}
