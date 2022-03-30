/*
VAR Rover Senior Design Project
FrSky Receiver/Arduino UNO Interface
*/

#include "sbus.h"

bfs::SbusRx RX(&Serial);  //Object for receiving
std::array<int16_t, bfs::SbusRx::NUM_CH()> RXData; //Array for storing received data
char ch[17] = {0,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};  //Array to make channel programming easier

char pwmPulse;

void setup() {
    //Start serial
    Serial.begin(115200);
    while(!Serial);

    //Begin SBUS communication
    RX.Begin();
}

void loop() {
    if(RX.Read()) RXData = RX.ch();      

    if(RXData[ch[3]]>150 && RXData[ch[3]]<1900){
      pwmPulse = map(RXData[ch[3]],172,1811,125,254);
      analogWrite(3,pwmPulse);
    }
    else
      analogWrite(3,0);
}
