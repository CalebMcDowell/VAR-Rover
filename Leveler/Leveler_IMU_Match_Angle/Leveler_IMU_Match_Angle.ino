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
    //Left
    const int enL = 9;
    const int L1 = 4;
    const int L2 = 5;
    //Right
    const int enR = 10;
    const int R1 = 6;
    const int R2 = 7;
    //Front
    const int enF = 13;
    const int F1 = 20;
    const int F2 = 21;
    //Back
    const int enB = 11;
    const int Back1 = 18;
    const int Back2 = 19;

//////PID Loop//////
    float pitchTarget = 0;
    float rollTarget = 0;
    float pitch, roll;
    float angleRange = 0.5;
    int pSpeed, rSpeed;
        
    float pKP = 15, pKI = 0.015, pKD = 300;
    float rKP = 15, rKI = 0.015, rKD = 300;
    
//////IMU Sensors//////
    Adafruit_BNO055 topImu = Adafruit_BNO055(55, 0x28);
    
//////Function Declarations//////
void move4Lvl(char ='S', unsigned char =0, int =0);
void Pitch(int =0);
void Roll(int =0);
void getIMUAngles(float &, float &);
int pitchPID(float);
int rollPID(float);


void setup() {
    //Linear Actuators
    pinMode(enF, OUTPUT);
    pinMode(F1, OUTPUT);
    pinMode(F2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(Back1, OUTPUT);
    pinMode(Back2, OUTPUT);
    pinMode(enL, OUTPUT);
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);
    pinMode(enR, OUTPUT);
    pinMode(R1, OUTPUT);
    pinMode(R2, OUTPUT);
    move4Lvl('S');
    
    //IMU Sensor Setup
    topImu.begin();
    topImu.setExtCrystalUse(true);

    delay(1000);
    Serial.begin(9600);
}

void loop() {
      do{
        getIMUAngles(pitch, roll);
      }while(pitch==0 && roll==0);
      
      if(abs(pitch) > angleRange){
        pSpeed = pitchPID(pitch);
        Pitch(pSpeed);
      }
      else{
        Pitch(0);
      }
      if(abs(roll) > angleRange){
        rSpeed = rollPID(roll);
        Roll(rSpeed);
      }
      else{
        Roll(0);
      }
      Serial.print("Pitch:");
      Serial.print(pitch);
      Serial.print("\t");
      Serial.print("Roll:");
      Serial.print(roll);
      Serial.print("\t");
      Serial.print("pSpeed: ");
      Serial.print(pSpeed);
      Serial.print("\t");
      Serial.print("rSpeed: ");
      Serial.print(rSpeed);
      Serial.println("\t");
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
    
     //Set speed of motors
     analogWrite(enL,pace);
     analogWrite(enR,pace);
     analogWrite(enF,pace);
     analogWrite(enB,pace);
}
void Pitch(int pace=0){
    //stop actuators if no pace
    if(pace==0){
      digitalWrite(F1,0);
      digitalWrite(F2,0);
      digitalWrite(Back1,0);
      digitalWrite(Back2,0);
      return;
    }

    pace = constrain(pace, -255, 255);
    //Set direction of movement
    bool d = 1;  //positive direction
    if(pace>0) d = 0;  //negative direction
    digitalWrite(F1,d);
    digitalWrite(F2,!d);
    digitalWrite(Back1,d);
    digitalWrite(Back2,!d);
    //Set speed of motors
    analogWrite(enF,abs(pace));
    analogWrite(enB,abs(pace));
}
void Roll(int pace=0){
    //stop actuators if no pace
    if(pace==0){
      digitalWrite(L1,0);
      digitalWrite(L2,0);
      digitalWrite(R1,0);
      digitalWrite(R2,0);
      return;
    }

    pace = constrain(pace, -255, 255);
    //Set direction of movement
    bool d = 1;  //positive direction
    if(pace>0) d = 0;  //negative direction
    digitalWrite(L1,d);
    digitalWrite(L2,!d);
    digitalWrite(R1,d);
    digitalWrite(R2,!d);
    //Set speed of motors
    analogWrite(enL,abs(pace));
    analogWrite(enR,abs(pace));
}
void getIMUAngles(float &newPitch, float &newRoll){
    //Get current orientation (Euler angles or degrees), in form of X,Y,Z vector
    imu::Vector<3> euler = topImu.getVector(Adafruit_BNO055::VECTOR_EULER);
    
    newPitch = euler.y();   //Pitch
    newRoll = euler.z();    //Roll

//    //Getquaternions and convert to roll and pitch
//    imu::Quaternion quat = topImu.getQuat();
//    newRoll=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.w()*quat.x()+quat.y()*quat.z()));
//    newPitch=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
//    //Convert to degrees
//    newRoll=newRoll/(3.14159265)*180;
//    newPitch=newPitch/(3.14159265)*180;
//    //For quaternions, reference: https://www.youtube.com/watch?v=S77r-P6YxAU&list=PLGs0VKk2DiYwEo-k0mjIkWXlkrJWAU4L9&index=21
}
int rollPID(float curRoll){
    static float rErr = 0, rErrPrev = 0, rIntErr = 0;
    static unsigned long curTime = 0, prevTime = 0;
    
    //Derivative
    rErrPrev = rErr;
    prevTime = curTime;
    curTime = millis();
    //Propotional
    rErr = rollTarget-curRoll;
    //Integral
    if(curRoll<3 && curRoll>-3){
      rIntErr += (rErr*(curTime-prevTime));
    }
    else{
      rIntErr = 0.0;
    }

    float output = (rKP*rErr) + (rKI*rIntErr) + (rKD*((rErr-rErrPrev)/float(curTime-prevTime)));
    return int(constrain(output, -255, 255));
}
int pitchPID(float curPitch){
    static float pErr = 0, pErrPrev = 0, pIntErr = 0;
    static unsigned long curTime = 0, prevTime = 0;
    
    //Derivative
    pErrPrev = pErr;
    prevTime = curTime;
    curTime = millis();
    //Propotional
    pErr = pitchTarget-curPitch;
    //Integral
    if(curPitch<3.0 && curPitch>-3.0){
      pIntErr += (pErr*(curTime-prevTime));
    }
    else{
      pIntErr = 0.0;
    }

    float output = (pKP*pErr) + (pKI*pIntErr) + (pKD*((pErr-pErrPrev)/float(curTime-prevTime)));
    return int(constrain(output, -255, 255));
}
