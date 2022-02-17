/*
VAR Rover Senior Design Project
cjm6756
RC/Arduino Interface

*/

unsigned long channel[2];

void setup() {
    delay(250);           //sanity delay
    pinMode(2,INPUT);
    pinMode(3,INPUT);
    pinMode(13, OUTPUT);  //LED_BUILTIN
    Serial.begin(9600);
}

void loop() {
    channel[0] = pulseIn(2, HIGH);
    channel[1] = pulseIn(3, HIGH);
    Serial.print(channel[0]);
    Serial.print(" - ");
    Serial.println(channel[1]);
    if(channel[0]>1500 && channel[1]>1900){
        digitalWrite(13, HIGH);
    }
    else
        digitalWrite(13, LOW);
}
