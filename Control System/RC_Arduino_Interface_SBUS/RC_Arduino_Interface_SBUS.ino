/*
VAR Rover Senior Design Project
FrSky Receiver/Arduino UNO Interface
*/

volatile unsigned long pulse;
volatile long sTime;
volatile long cTime;
volatile int ch;
unsigned long channel;

void setup() {
    delay(250);           //sanity delay
    
    pinMode(2,INPUT_PULLUP);
    pinMode(13, OUTPUT);  //LED_BUILTIN
    
    attachInterrupt(digitalPinToInterrupt(2), pulseTimeCh11, CHANGE);
    
    Serial.begin(9600);
}

void loop() {
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

void pulseTimeCh11(){
    cTime = micros();
    if(cTime > sTime){
      pulse = cTime - sTime;
      sTime = cTime;
    }
}
