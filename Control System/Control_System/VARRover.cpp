//File: VARRover.cpp
#include "VARRover.h"

bfs::SbusRx RX(&Serial);  //Object for receiving

void setup() {
    //Serial communication between UNO and FrSky receiver
    Serial.begin(115200);
    while(!Serial);
    //Begin SBUS communication
    RX.Begin();
}

//Function to get an array of the channel values. Returns 0 if error/failsafe/etc
bool getRxData(std::array<int16_t, bfs::SbusRx::NUM_CH()> &channel){
    if(RX.Read() && !RX.failsafe()){
      channel = RX.ch();
      int16_t temp = channel[bfs::SbusRx::NUM_CH()-1];
      for(int i=bfs::SbusRx::NUM_CH()-1; i>=0; i--){
        if(i==0)
          channel[i] = temp;
        else
          channel[i] = channel[i-1];
      }
    }
    else
      return 0;
}
