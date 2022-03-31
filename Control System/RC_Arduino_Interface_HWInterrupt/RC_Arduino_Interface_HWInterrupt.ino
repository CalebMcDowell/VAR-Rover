/*
VAR Rover Senior Design Project
FrSky Receiver/Arduino UNO Interface
*/

volatile unsigned long pulse[3];
volatile long sTime[3];
volatile long cTime[3];
volatile int ch;
unsigned long channel[3];

void setup() {
    delay(250);           //sanity delay
    
    pinMode(2,INPUT_PULLUP);
    pinMode(3,INPUT_PULLUP);
    pinMode(4,INPUT_PULLUP);
    pinMode(13, OUTPUT);  //LED_BUILTIN
    
    attachInterrupt(digitalPinToInterrupt(2), pulseTimeCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), pulseTimeCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(4), pulseTimeCh3, CHANGE);
    
    Serial.begin(9600);
}

void loop() {
    for(int i=0; i<3; i++){
        if(pulse[i]<2500)
          channel[i] = pulse[i];
          
        Serial.print(channel[i]);
        Serial.print(" - ");
    }
    Serial.println();
    
    if(channel[0]>1500 && channel[1]>1900){
        digitalWrite(13, HIGH);
    }
    else
        digitalWrite(13, LOW);
        
}

void pulseTimeCh1(){
    ch = 0;
    cTime[ch] = micros();
    if(cTime[ch] > sTime[ch]){
      pulse[ch] = cTime[ch] - sTime[ch];
      sTime[ch] = cTime[ch];
    }
}
void pulseTimeCh2(){
    ch = 1;
    cTime[ch] = micros();
    if(cTime[ch] > sTime[ch]){
      pulse[ch] = cTime[ch] - sTime[ch];
      sTime[ch] = cTime[ch];
    }
}
void pulseTimeCh3(){
    ch = 2;
    cTime[ch] = micros();
    if(cTime[ch] > sTime[ch]){
      pulse[ch] = cTime[ch] - sTime[ch];
      sTime[ch] = cTime[ch];
    }
}
