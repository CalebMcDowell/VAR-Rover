/*

*/
//////Linear Actuators//////
    //Back Left
    // const int enBL = 10;
    const int L1 = 4;
    const int L2 = 5;
    //Back Right
    // const int enBR = 9;
    const int R1 = 6;
    const int R2 = 7;
    //Front
    // const int enF = 1;
    const int F1 = 2;
    const int F2 = 3;
    
//////Function Declarations//////
void moveLvl(char ='S', unsigned char =0, int =0);

void setup() {
    //Motors
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);
    pinMode(R1, OUTPUT);
    pinMode(R2, OUTPUT);
    pinMode(F1, OUTPUT);
    pinMode(F2, OUTPUT);
}

void loop() {
    unsigned char speed = 0;
  
  //Start leveler in fully retracted position
    moveLvl('H', speed, -1);
    delay(5000);
    moveLvl('S');
    
  //Extend a little
    moveLvl('H', speed, 1);
    delay(1000);
    moveLvl('S');
    delay(1000);
    
  //Roll left
    moveLvl('R', speed, 1);
    delay(500);
    moveLvl('S');
    delay(1000);
    
  //Roll right
    moveLvl('R', speed, -1);
    delay(500);
    moveLvl('S');
    delay(1000);
    
  //Roll right
    moveLvl('R', speed, -1);
    delay(500);
    moveLvl('S');
    delay(1000);
    
  //Roll left
    moveLvl('R', speed, 1);
    delay(500);
    moveLvl('S');
    delay(1000);
    
  //Pitch down
    moveLvl('P', speed, 1);
    delay(500);
    moveLvl('S');
    delay(1000);
    
  //Pitch up
    moveLvl('P', speed, -1);
    delay(500);
    moveLvl('S');
    delay(1000);
    
  //Pitch up
    moveLvl('P', speed, -1);
    delay(500);
    moveLvl('S');
    delay(1000);
    
  //Pitch down
    moveLvl('P', speed, 1);
    delay(500);
    moveLvl('S');
    delay(1000);
    
  //Extend fully
    moveLvl('H', speed, 1);
    delay(5000);
    moveLvl('S');
    
    while(1); //Wait for reset
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
