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
    //Back Left
    // const int enBL = 10;
    const int L1 = 4;
    const int L2 = 5;
    //Back Right
    // const int enBR = 9;
    const int R1 = 6;
    const int R2 = 7;
    
//////IMU Sensors//////
    Adafruit_BNO055 topImu = Adafruit_BNO055(55, 0x28);
    
//////Function Declarations//////
void moveLvl(char ='S', unsigned char =0, int =0);
float getIMUPitch();
float getIMURoll();


void setup() {
    //Linear Actuators
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);
    pinMode(R1, OUTPUT);
    pinMode(R2, OUTPUT);
    pinMode(F1, OUTPUT);
    pinMode(F2, OUTPUT);
    
    //IMU Sensor Setup
    topImu.begin();
    topImu.setExtCrystalUse(true);
    
    Serial.begin(9600);
}

void loop() {
    unsigned char speed = 0;
    float pitch, roll;
  
  //Init leveler to slightly extended position
    moveLvl('H', speed, 1);
    delay(500);
    moveLvl('H', speed, -1);
    delay(5000);
    moveLvl('H', speed, 1);
    delay(1000);
    moveLvl('S');
    
  
    while(1){
      pitch = getIMUPitch();
      roll = getIMURoll();
      Serial.print(pitch);
      Serial.print(", ");
      Serial.println(roll);
      if(pitch>5 || pitch<-5){
        moveLvl('P', speed, pitch);
      }
      else{
        moveLvl('S');
      }
      if(roll>5 || roll<-5){
        moveLvl('R', speed, roll);
      }
      else
        moveLvl('S');
    }
    
}

void moveLvl(char axis='S', unsigned char pace=0, int angle=0){
    //Set direction of actuators
    bool d = 1;  //positive direction
    if(angle<0) d = 0;  //negative direction
    
    //Set axis of rotation/extend/retract/stop
      //NOTES//
        //Extend Left Actuator
          // digitalWrite(L1,d);
          // digitalWrite(L2,!d);
        //Extend Right Actuator
          // digitalWrite(R1,d);
          // digitalWrite(R2,!d);
        //Extend Front Actuator
          // digitalWrite(F1,d);
          // digitalWrite(F2,!d);
    switch((int)axis){
      case 'R': //Roll
        digitalWrite(L1,!d);
        digitalWrite(L2,d);
        digitalWrite(R1,d);
        digitalWrite(R2,!d);
        break;
      case 'P': //Pitch
        digitalWrite(L1,!d);
        digitalWrite(L2,d);
        digitalWrite(R1,!d);
        digitalWrite(R2,d);
        digitalWrite(F1,d);
        digitalWrite(F2,!d);
        break;
      case 'H': //Height adjust
        digitalWrite(L1,d);
        digitalWrite(L2,!d);
        digitalWrite(R1,d);
        digitalWrite(R2,!d);
        digitalWrite(F1,d);
        digitalWrite(F2,!d);
        break;
      case 'S': //Stop
      default:
        pace = 0;
        digitalWrite(L1,0);
        digitalWrite(L2,0);
        digitalWrite(R1,0);
        digitalWrite(R2,0);
        digitalWrite(F1,0);
        digitalWrite(F2,0);
        break;
    }
    
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
