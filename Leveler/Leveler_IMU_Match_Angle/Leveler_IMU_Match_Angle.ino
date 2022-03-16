/*
VAR Rover Senior Design Project
Code to level an IMU on top of the tripod
*/
//////Includes//////
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS 100

//////Linear Actuators//////
    //Front
    // const int enF = 1;
    const int F1 = 2;
    const int F2 = 3;
    //Back
    // const int enBL = 10;
    const int Back1 = 4;
    const int Back2 = 5;
    //Left
    // const int enBR = 9;
    const int L1 = 6;
    const int L2 = 7;
    //Right
    // const int enBR = 9;
    const int R1 = 8;
    const int R2 = 9;
    
//////IMU Sensors//////
    Adafruit_BNO055 topImu = Adafruit_BNO055(55, 0x28);
    
//////Function Declarations//////
void move4Lvl(char ='S', unsigned char =0, int =0);
void Pitch(unsigned char =0, int =0);
void Roll(unsigned char =0, int =0);
float getIMUPitch();
float getIMURoll();


void setup() {
    //Linear Actuators
    pinMode(F1, OUTPUT);
    pinMode(F2, OUTPUT);
    pinMode(Back1, OUTPUT);
    pinMode(Back2, OUTPUT);
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);
    pinMode(R1, OUTPUT);
    pinMode(R2, OUTPUT);
    
    //IMU Sensor Setup
    topImu.begin();
    topImu.setExtCrystalUse(true);
    
    Serial.begin(9600);
}

void loop() {
    unsigned char speed = 1;
    unsigned char angleRange = 1;
    float pitch, roll;
    
    move4Lvl('S');
    while(1){
      pitch = getIMUPitch();
      roll = getIMURoll();
      Serial.print(pitch);
      Serial.print("\t");
      Serial.println(roll);
      if(pitch>angleRange || pitch<-angleRange){
        Pitch(speed, pitch);
      }
      else{
        Pitch(0);
      }
      if(roll>angleRange || roll<-angleRange){
        Roll(speed, roll);
      }
      else
        Roll(0);
    }
    
}

void move4Lvl(char axis='S', unsigned char pace=0, int angle=0){
    //Set direction of actuators
    bool d = 1;  //positive direction
    if(angle<0) d = 0;  //negative direction
    
    //Set axis of rotation/extend/retract/stop
                //NOTES//
          //Extend Actuator
          // digitalWrite(L1,d);
          // digitalWrite(L2,!d);
    switch((int)axis){
      case 'R': //Roll
        digitalWrite(L1,d);
        digitalWrite(L2,!d);
        digitalWrite(R1,d);
        digitalWrite(R2,!d);
        break;
      case 'P': //Pitch
        digitalWrite(F1,!d);
        digitalWrite(F2,d);
        digitalWrite(Back1,!d);
        digitalWrite(Back2,d);
        break;
      case 'H': //Height adjust
        digitalWrite(F1,d);
        digitalWrite(F2,!d);
        digitalWrite(Back1,d);
        digitalWrite(Back2,!d);
        digitalWrite(L1,d);
        digitalWrite(L2,!d);
        digitalWrite(R1,d);
        digitalWrite(R2,!d);
        break;
      case 'S': //Stop
      default:
        pace = 0;
        digitalWrite(F1,0);
        digitalWrite(F2,0);
        digitalWrite(Back1,0);
        digitalWrite(Back2,0);
        digitalWrite(L1,0);
        digitalWrite(L2,0);
        digitalWrite(R1,0);
        digitalWrite(R2,0);
        break;
    }
    
    // //Set speed of motors
    // analogWrite(enA,pace);
    // analogWrite(enB,pace);
}

void Pitch(unsigned char pace=0, int angle=0){
    //stop actuators if no pace
    if(pace==0){
      digitalWrite(F1,0);
      digitalWrite(F2,0);
      digitalWrite(Back1,0);
      digitalWrite(Back2,0);
      return;
    }
    
    //Set direction of movement
    bool d = 1;  //positive direction
    if(angle<0) d = 0;  //negative direction
    digitalWrite(F1,!d);
    digitalWrite(F2,d);
    digitalWrite(Back1,!d);
    digitalWrite(Back2,d);
    
    // //Set speed of motors
    // analogWrite(enA,pace);
    // analogWrite(enB,pace);
}

void Roll(unsigned char pace=0, int angle=0){
    //stop actuators if no pace
    if(pace==0){
      digitalWrite(L1,0);
      digitalWrite(L2,0);
      digitalWrite(R1,0);
      digitalWrite(R2,0);
      return;
    }

    //Set direction of movement
    bool d = 1;  //positive direction
    if(angle<0) d = 0;  //negative direction
    digitalWrite(L1,d);
    digitalWrite(L2,!d);
    digitalWrite(R1,d);
    digitalWrite(R2,!d);
    
    // //Set speed of motors
    // analogWrite(enA,pace);
    // analogWrite(enB,pace);
}


float getIMUPitch(){
//    sensors_event_t event; 
//    bno.getEvent(&event);
//    return event.orientation.x;

    //Get current orientation (Euler angles or degrees), in form of X,Y,Z vector
    imu::Vector<3> euler = topImu.getVector(Adafruit_BNO055::VECTOR_EULER);
    //Return yaw angle
    return euler.y();
}

float getIMURoll(){
//    sensors_event_t event; 
//    bno.getEvent(&event);
//    return event.orientation.x;

    //Get current orientation (Euler angles or degrees), in form of X,Y,Z vector
    imu::Vector<3> euler = topImu.getVector(Adafruit_BNO055::VECTOR_EULER);
    //Return yaw angle
    return euler.z();
}
