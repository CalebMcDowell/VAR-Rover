/*
VAR Rover Senior Design Project
FrSky Receiver/Arduino UNO Interface
*/

#include "sbus.h"

bfs::SbusRx RX(&Serial);  //Object for receiving
bfs::SbusTx TX(&Serial);  //Object for transmitting
std::array<int16_t, bfs::SbusRx::NUM_CH()> RXData; //Array for storing received data


volatile unsigned long pulse;
volatile long sTime;
volatile long cTime;

void setup() {
    //Start serial
    Serial.begin(115200);
    while(!Serial);

    //Begin SBUS communication
    RX.begin();
    TX.begin();

    //LED_BUILTIN
    pinMode(13, OUTPUT);
    
}

void loop() {
    if(RX.read(){
        RXData = RX.ch();
        //Display received data
        for(int8_t i=0; i<bfs::SbusRx::NUM_CH(); i++){
            Serial.print(RXData[i]);
            Serial.print("\t");
        }
        //Display lost frames and failsafe info
        Serial.print(RX.lost_frame());
        Serial.print("\t");
        Serial.println(RX.failsafe());
    }


  
    if(pulse<2500){
        channel = pulse;
        Serial.println(channel);
    }
    
    if(channel>1500 && channel<1900){
        digitalWrite(13, HIGH);
    }
    else
        digitalWrite(13, LOW);
    
    delay(500);
}
