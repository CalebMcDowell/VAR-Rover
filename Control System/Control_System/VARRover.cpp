//File: VARRover.cpp
#include "VARRover.h"

////Constructor. Initialize rover
Rover::Rover(void){
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
    if(RX.Read() && !RX.failsafe()){
      //get all channel data
      RxData = RX.ch();
      if(RxData[2]>1500){
        digitalWrite(LED_BUILTIN,HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN,LOW);
        delay(1000);
      }
      //success
      return 1;
    }

    //failed
    return 0;
}
//Returns desired channel value. channel(3) return ch3
int Rover::channel(char dch) const{
    if(dch<1 || dch>bfs::SbusRx::NUM_CH())// || !getRxData())
      return -1;
    
    return RxData[dch-1];
}
