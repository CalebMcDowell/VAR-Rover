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
    
    
//////IMU Sensors//////
    Adafruit_BNO055 topImu = Adafruit_BNO055(55, 0x28);
    
//////Function Declarations//////
void move4Lvl(char ='S', unsigned char =0, int =0);
void Pitch(unsigned char =0, int =0);
void Roll(unsigned char =0, int =0);
void getIMUAngles(float &, float &);
void PIDloop(float, float, byte &, byte &);


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
    
    //IMU Sensor Setup
    topImu.begin();
    topImu.setExtCrystalUse(true);

    delay(1000);
    Serial.begin(9600);
}

void loop() {
    byte pSpeed, rSpeed;
    float angleRange = 2;
    float pitch, roll;
    
    move4Lvl('S');
    while(1){
      do{
        getIMUAngles(pitch, roll);
      }while(pitch==0 && roll==0);

      if(roll>angleRange || roll<-angleRange){
        PIDloop(roll, pitch, rSpeed, pSpeed);
      }
      else{
        PIDloop(0,0,rSpeed,pSpeed);
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

//      Pitch(pSpeed, pitch);
      Roll(rSpeed, roll);
//      if(pitch>angleRange || pitch<-angleRange)
//        Pitch(pSpeed, pitch);
//      else
//        Pitch(0);
//      if(roll>angleRange || roll<-angleRange)
//        Roll(rSpeed, roll);
//      else
//        Roll(0);
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
    
     //Set speed of motors
     analogWrite(enL,pace);
     analogWrite(enR,pace);
     analogWrite(enF,pace);
     analogWrite(enB,pace);
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
    digitalWrite(F1,d);
    digitalWrite(F2,!d);
    digitalWrite(Back1,d);
    digitalWrite(Back2,!d);
    
    //Set speed of motors
    analogWrite(enF,pace);
    analogWrite(enB,pace);
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
    
    //Set speed of motors
    analogWrite(enL,pace);
    analogWrite(enR,pace);
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

void PIDloop (float rollACTUAL, float pitchACTUAL, byte &rollPACE, byte &pitchPACE){
   static int tOLD, tNEW, dt;
   
   float kp = 0.1;
   float ki = 0.0;
   float kd = 0.0;
   
   static float rollTARGET = 0.0;
   static float rollERROR, rollERRORold, rolldERROR, rolldERRORdt, rollERRORarea;
   static float rollVal;
   
   static float pitchTARGET = 0.0;
   static float pitchERROR, pitchERRORold, pitchdERROR, pitchdERRORdt, pitchERRORarea;
   static float pitchVal;

    tOLD = tNEW;
    tNEW = millis();
    dt = tNEW - tOLD;
    
    rollERRORold = rollERROR;
    rollERROR = rollTARGET - rollACTUAL;
    rolldERROR = rollERROR - rollERRORold;
    rolldERRORdt = rolldERROR/dt;
    rollERRORarea = rollERRORarea+rollERROR*dt;

    pitchERRORold = pitchERROR;
    pitchERROR = pitchTARGET - pitchACTUAL;
    pitchdERROR = pitchERROR - pitchERRORold;
    pitchdERRORdt = pitchdERROR/dt;
    pitchERRORarea = pitchERRORarea+pitchERROR*dt;

//    rollVal = rollVal + (kp*rollERROR) + (ki*rollERRORarea) + (kd*rolldERRORdt);
//    pitchVal = pitchVal + (kp*pitchERROR) + (ki*pitchERRORarea) + (kd*pitchdERRORdt);
    rollVal = (kp*rollERROR) + (ki*rollERRORarea) + (kd*rolldERRORdt);
    pitchVal = (kp*pitchERROR) + (ki*pitchERRORarea) + (kd*pitchdERRORdt);
    
    float rolllo = 0.0;     //Perfectly level
    float rollhi = 180.0;   // Farthest possible error value? Need to find this maximum.
    float pitchlo = 0.0;    //Perfectly level
    float pitchhi = 180.0;  //Farthest possible error value.

    rollPACE = min(byte(abs(rollVal)), byte(rollhi));
    rollPACE = max(byte(abs(rollVal)), byte(rolllo));
    rollPACE = byte(rollPACE);

    pitchPACE = min(byte(abs(pitchVal)), byte(pitchhi));
    pitchPACE = max(byte(abs(pitchVal)), byte(pitchlo));
    pitchPACE = byte(pitchPACE);
}
