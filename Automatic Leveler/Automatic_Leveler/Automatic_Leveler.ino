/*
VAR Rover Senior Design Project
Code to level an IMU on top of the tripod
*/
//////Includes//////
#include <Adafruit_Sensor.h>  //Requires "Adafruit Unified Sensor" library (v1.1.5)
#include <Adafruit_BNO055.h>  //Requires "Adafruit BNO055" library (v1.5.2)
#include <Arduino.h>
#include <TinyMPU6050.h>      //Requires "TinyMPU6050" library (v0.5.3)
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
    float levelLimit = 20;
    float pitchTarget = 0;
    float rollTarget = 0;
    float pAdjust = 0;
    float rAdjust = 0;
    float tPitch, tRoll, bPitch, bRoll;
    float angleRange = 0.5;
    int pSpeed, rSpeed;
        
//    float pKP = 18, pKI = 0.015, pKD = 300;
//    float rKP = 18, rKI = 0.015, rKD = 300;
    float pKP = 18, pKI = 0.015, pKD = 0;
    float rKP = 18, rKI = 0.015, rKD = 0;
    
//////IMU Sensors//////
    Adafruit_BNO055 topIMU = Adafruit_BNO055(55, 0x28);
    MPU6050 bottomIMU(Wire);
    
//////Function Declarations//////
void move4Lvl(char ='S', unsigned char =0, int =0);
void Pitch(int =0);
void Roll(int =0);
void getTopIMUAngles(float &, float &);
void getBotIMUAngles(float &, float &);
int pitchPID(float);
int rollPID(float);
int receiveInfo(float &, float &);


void setup() {
    delay(1000);
    //General
    pinMode(LED_BUILTIN, OUTPUT);
    
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
    topIMU.begin();
    topIMU.setExtCrystalUse(true);
    bottomIMU.Initialize();
//    bottomIMU.Calibrate();        //DO NOT MOVE IMU DURING THIS FUNCTION
    Wire.setWireTimeout(3000,true);

    Serial.begin(9600);
    Serial1.begin(9600);
    Serial1.setTimeout(100);
}
void loop() {
  //get command from control system
    while(receiveInfo(pAdjust, rAdjust) == -1){
      Roll(0);
      Pitch(0);
      delay(5000);
    }
  //get top IMU current angles
    do{
      getTopIMUAngles(tPitch, tRoll);
    }while(tPitch==0 && tRoll==0);
  //factor in adjustment from control system
    tPitch -= pAdjust;
    tRoll -= rAdjust;
  //get bottom IMU current angles, adjust limits
    getBotIMUAngles(bPitch, bRoll);
    if(bPitch > levelLimit){
      pitchTarget = bPitch-levelLimit;
    }
    else if(bPitch < -levelLimit){
      pitchTarget = bPitch+levelLimit;
    }
    else{
      pitchTarget = 0;
    }
    if(bRoll > levelLimit){
      rollTarget = bRoll-levelLimit;
    }
    else if(bRoll < -levelLimit){
      rollTarget = bRoll+levelLimit;
    }
    else{
      rollTarget = 0;
    }
  //level to within an acceptable range
    if(abs(tPitch-pitchTarget) > angleRange){
      pSpeed = pitchPID(tPitch);
      Pitch(pSpeed);
    }
    else{
      Pitch(0);
    }
    if(abs(tRoll-rollTarget) > angleRange){
      rSpeed = rollPID(tRoll);
      Roll(rSpeed);
    }
    else{
      Roll(0);
    }
  //print out all info
    Serial.print("tPitch:");
    Serial.print(tPitch);
    Serial.print("\ttRoll:");
    Serial.print(tRoll);
    Serial.print("\tbPitch:");
    Serial.print(bPitch);
    Serial.print("\tbRoll:");
    Serial.print(bRoll);
    Serial.print("\tpAdjust: ");
    Serial.print(pAdjust);
    Serial.print("\trAdjust: ");
    Serial.print(rAdjust);
//    Serial.print("\tpSpeed: ");
//    Serial.print(pSpeed);
//    Serial.print("\trSpeed: ");
//    Serial.print(rSpeed);
//    Serial.print("\tpTarget: ");
//    Serial.print(pitchTarget);
//    Serial.print("\trTarget: ");
//    Serial.print(rollTarget);
    Serial.println();
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
void getTopIMUAngles(float &newTopPitch, float &newTopRoll){
    //Get current orientation (Euler angles or degrees), in form of X,Y,Z vector
    imu::Vector<3> euler = topIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    
    newTopPitch = euler.y();   //Pitch
    newTopRoll = euler.z();    //Roll

//    //Getquaternions and convert to roll and pitch
//    imu::Quaternion quat = topImu.getQuat();
//    newRoll=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.w()*quat.x()+quat.y()*quat.z()));
//    newPitch=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
//    //Convert to degrees
//    newRoll=newRoll/(3.14159265)*180;
//    newPitch=newPitch/(3.14159265)*180;
//    //For quaternions, reference: https://www.youtube.com/watch?v=S77r-P6YxAU&list=PLGs0VKk2DiYwEo-k0mjIkWXlkrJWAU4L9&index=21
}
void getBotIMUAngles(float &newBotPitch, float &newBotRoll){
    Wire.clearWireTimeoutFlag();
    bottomIMU.Execute();
    if(Wire.getWireTimeoutFlag()){
      Serial.print("TIMEOUT ERROR\t");
      return;
    }
    newBotRoll = bottomIMU.GetAngX();
    newBotPitch = bottomIMU.GetAngY();
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
int receiveInfo(float &pToAdjust, float &rToAdjust){
    if(Serial1.available()>0){
      String msg;
      int axis = Serial1.peek();
      if(axis == 'S'){  //Stop leveler
        Pitch(0);
        Roll(0);
        Serial1.readStringUntil('\n');
        return -1;
      }
      else if(axis == 'R'){ //adjust Roll
        Serial1.read();
        msg = Serial1.readStringUntil('\n');
        rToAdjust = msg.toFloat();
      }
      else if(axis == 'P'){ //adjust Pitch
        Serial1.read();
        msg = Serial1.readStringUntil('\n');
        pToAdjust = msg.toFloat();
      }
      else{ //garbage data, ignore it
        Serial1.readStringUntil('\n');
      }
//      Serial.print("\tReceived Roll: ");
//      Serial.print(pToAdjust);
//      Serial.print("\tReceived Pitch: ");
//      Serial.print(rToAdjust);
//      Serial.print("\tReceived message: ");
//      Serial.println(msg);
      return 1;
    }
    else{
      return 0;
    }
}
