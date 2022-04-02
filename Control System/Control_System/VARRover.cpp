//File: VARRover.cpp
#include "VARRover.h"

//Constructor. Initialize rover
Rover::Rover(void){
    pinMode(LED_BUILTIN,OUTPUT);
    pinMode(5, OUTPUT);
    //Disarm rover
    armed = 0;
    //Serial SBUS communication between UNO and FrSky receiver
    Serial.begin(115200);
    while(!Serial);
    RX.Begin();
}
//Get an array of the channel values. Returns 0 if error/failsafe/etc
bool Rover::getRxData(){
    //if data to be read and rx not in failsafe mode
    if(RX.Read()){
      //get all channel data
      RxData = RX.ch();
      digitalWrite(LED_BUILTIN,HIGH);
      //success
      return 1;
    }
    else{
        digitalWrite(5,HIGH);
        delay(1000);
        digitalWrite(5,LOW);
        delay(1000);
    }
    //failed
    return 0;
}
//Returns desired channel value. channel(3) return ch3
int Rover::channel(char dch) const{
    if(dch<1 || dch>bfs::SbusRx::NUM_CH() || !getRxData())
      return -1;
    
    return RxData[dch-1];
}
