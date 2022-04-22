//File: VARRover.cpp
#include "VARRover.h"

///////////////SBUS CLASS METHODS///////////////
void SbusRx::Begin() {
  /* Start the bus */
  Serial1.begin(BAUD_, SERIAL_8E2);
  /* flush the bus */
  Serial1.flush();
}
bool SbusRx::Read() {
  /* Read through all available packets to get the newest */
  new_data_ = false;
  do {
    if (Parse()) {
      new_data_ = true;
    }
  } while (Serial1.available());
  /* Parse new data, if available */
  if (new_data_) {
    /* Grab the channel data */
    ch_[0]  = static_cast<int16_t>(buf_[1]       | buf_[2]  << 8 & 0x07FF);
    ch_[1]  = static_cast<int16_t>(buf_[2]  >> 3 | buf_[3]  << 5 & 0x07FF);
    ch_[2]  = static_cast<int16_t>(buf_[3]  >> 6 | buf_[4]  << 2  |
                                   buf_[5] << 10 & 0x07FF);
    ch_[3]  = static_cast<int16_t>(buf_[5]  >> 1 | buf_[6]  << 7 & 0x07FF);
    ch_[4]  = static_cast<int16_t>(buf_[6]  >> 4 | buf_[7]  << 4 & 0x07FF);
    ch_[5]  = static_cast<int16_t>(buf_[7]  >> 7 | buf_[8]  << 1  |
                                   buf_[9] << 9 & 0x07FF);
    ch_[6]  = static_cast<int16_t>(buf_[9]  >> 2 | buf_[10] << 6 & 0x07FF);
    ch_[7]  = static_cast<int16_t>(buf_[10] >> 5 | buf_[11] << 3 & 0x07FF);
    ch_[8]  = static_cast<int16_t>(buf_[12]      | buf_[13] << 8 & 0x07FF);
    ch_[9]  = static_cast<int16_t>(buf_[13] >> 3 | buf_[14] << 5 & 0x07FF);
    ch_[10] = static_cast<int16_t>(buf_[14] >> 6 | buf_[15] << 2  |
                                   buf_[16] << 10 & 0x07FF);
    ch_[11] = static_cast<int16_t>(buf_[16] >> 1 | buf_[17] << 7 & 0x07FF);
    ch_[12] = static_cast<int16_t>(buf_[17] >> 4 | buf_[18] << 4 & 0x07FF);
    ch_[13] = static_cast<int16_t>(buf_[18] >> 7 | buf_[19] << 1  |
                                   buf_[20] << 9 & 0x07FF);
    ch_[14] = static_cast<int16_t>(buf_[20] >> 2 | buf_[21] << 6 & 0x07FF);
    ch_[15] = static_cast<int16_t>(buf_[21] >> 5 | buf_[22] << 3 & 0x07FF);
    /* CH 17 */
    ch17_ = buf_[23] & CH17_MASK_;
    /* CH 18 */
    ch18_ = buf_[23] & CH18_MASK_;
    /* Grab the lost frame */
    lost_frame_ = buf_[23] & LOST_FRAME_MASK_;
    /* Grab the failsafe */
    failsafe_ = buf_[23] & FAILSAFE_MASK_;
  }
  return new_data_;
}
bool SbusRx::Parse() {
  /* Parse messages */
  while (Serial1.available()) {
    cur_byte_ = Serial1.read();
    if (state_ == 0) {
      if ((cur_byte_ == HEADER_) && ((prev_byte_ == FOOTER_) ||
         ((prev_byte_ & 0x0F) == FOOTER2_))) {
        buf_[state_++] = cur_byte_;
      } else {
        state_ = 0;
      }
    } else {
      if (state_ < BUF_LEN_) {
        buf_[state_++] = cur_byte_;
      } else {
        state_ = 0;
        if ((buf_[BUF_LEN_ - 1] == FOOTER_) ||
           ((buf_[BUF_LEN_ - 1] & 0x0F) == FOOTER2_)) {
          return true;
        } else {
          return false;
        }
      }
    }
    prev_byte_ = cur_byte_;
  }
  return false;
}

///////////////ROVER CLASS METHODS//////////////
//Initialize rover
bool Rover::init(){
    //Start LCD display
    lcd = new LiquidCrystal_I2C(0x27,20,4); //LCD address to 0x27,20chars X 4lines
    lcd->init();
    lcd->backlight();
    dispSplash();
    //Safety IMU setup
    Wire.setWireTimeout(3000,true); //timeout after 3000us, reset on timeout
    safetyIMU = new MPU6050(Wire);
    safetyIMU->Initialize();
//    Serial.println("Beginning safety IMU calibration");
//    safetyIMU->Calibrate();
//    Serial.println("Calibration complete");
    //Drivetrain relay setup
    pinMode(FLR,OUTPUT);
    pinMode(FRR,OUTPUT);
    pinMode(BLR,OUTPUT);
    pinMode(BRR,OUTPUT);
    //Lift setup
    pinMode(LEn,OUTPUT);
    //Leveler setup
    Serial2.begin(9600);
    Serial2.setTimeout(100);
    //Disarm rover
    if(!disarm()){
      return false;
    }
    //Begin SBUS communication with receiver
    Serial1.begin(9600);
    RX.Begin();
    //Init success
    lcd->clear();
    errorCode = 'N';
    return true;
}
//Arms the rover which allows subsystems to run
bool Rover::arm(){
    armed = 1;
    motorRelays(1); //enable motor relays

    Serial.println("IS ARMED");
    return true;
}
//Disarms all rover systems
bool Rover::disarm(){
    armed = 0;
    motorRelays(0); //disable motor relays
    drive(0);
    moveLeveler(0);
    lift(0);

    Serial.println("DISARMED");
    return true;
}
//Get the radio failsafe state, updates by RX.Read()
bool Rover::failsafe() const{
    return RX.failsafe();
}
//Get an array of the channel values. Returns 0 if error/failsafe/etc
bool Rover::getRxData(){
    //if data to be read and rx not in failsafe mode
    if(RX.Read() && !RX.failsafe()){
      //get all channel data
      RxData = RX.ch();
      //success
//      Serial.println("RADIO OK");
      return true;
    }
    //failed
    if(RX.failsafe()){
      Serial.println("No radio");
      errorCode = 'R';  //radio error code
    }
//    Serial.println("RADIO FAILED");
//    errorCode = 'R';
    return false;
}
//Returns desired channel value. channel(3) return ch3
int Rover::channel(byte dch) const{
    if(dch<1 || dch>RX.NUM_CH())// || !getRxData())
      return -1;
    
    return RxData[dch-1];
}
//Print all channel data to serial monitor
void Rover::printChannels() const{
    for(int i=1; i<=10; i++){
      Serial.print("CH");
      Serial.print(i);
      Serial.print(":");
      Serial.print(channel(i));
      Serial.print(" ");
    }
    Serial.println("");
}
//Get if rover has an active error code
bool Rover::getRoverError() const{
  if(errorCode != 'N'){
//    Serial.println("IS ERROR");
    return true;
  }
  else{
//    Serial.println("NO ERROR");
    return false;
  }
}
//Measure battery voltages
bool Rover::getVoltages(){
    int maxAn = 681;   //16.8V (max analog reading)
    int minAn = 536;   //13.2V (min analog reading)
  
    //read sensors
    int FrontAn = analogRead(FBatt);
    int BackAn = analogRead(BBatt);
    int ControlAn = analogRead(CBatt);
    
    //equations based on voltage sensor testing
    //Back and Front different from Control because of wire length
    FBatV = (float(FrontAn)+5.6017)/40.353;
    BBatV = (float(BackAn)+5.6017)/40.353;
    CBatV = (float(ControlAn)+5.6017)/40.353;
    
    //map analog values to percentage charge
    FBatAmt = map(constrain(FrontAn,minAn,maxAn),minAn,maxAn,0,100);
    BBatAmt = map(constrain(BackAn,minAn,maxAn),minAn,maxAn,0,100);
    CBatAmt = map(constrain(ControlAn,minAn,maxAn),minAn,maxAn,0,100);

//    static unsigned long Favg, Bavg, Cavg;
//    static int count = 0;
//    Favg += FrontAn;
//    Bavg += BackAn;
//    Cavg += ControlAn;
//    count++;
//    if(count>=8){
//      Serial.print("\tFV: ");
//      Serial.print(Favg/count);
//      Serial.print("\tBV: ");
//      Serial.print(Bavg/count);
//      Serial.print("\tCV: ");
//      Serial.println(Cavg/count);
//      Favg = 0;
//      Bavg = 0;
//      Cavg = 0;
//      count = 0;
//    }
  //  Serial.print("\tF%: ");
  //  Serial.println(FBatAmt);
  
    //check for low voltage
    if(FrontAn<minAn || BackAn<minAn || ControlAn<minAn){
      Serial.println("Low Battery!!");
      errorCode = 'V';  //voltage error code
      return false;
    }
//    Serial.println("VOLTAGE OK");
    return true;    
}
//Measure rover's angle of incline
bool Rover::getRovAngles(){
    static byte count = 0;            //used for timeout flag
    static float maxIncline = 20.0;   //rover can drive upto max incline
    static unsigned long lvlPrevTime = 0; //previous time the function was called
    static unsigned long timeDelay = 100; //ms to wait before updating LCD
    
    //////////////UNCOMMENT FOR DEBUGGING//////////////////
    return true;
//    Serial.println("trying for angles");
    
    //only get angles every timeDelay ms
    if(millis()-lvlPrevTime<timeDelay)
      return true;
    //update timer
    lvlPrevTime = millis();

    rovPitch = 0;
    rovRoll = 0;
    
    Wire.clearWireTimeoutFlag();
    Serial.println("executing");
    safetyIMU->Execute();
    Serial.println("done executing");
    if(Wire.getWireTimeoutFlag()){
      Serial.print("TIMEOUT");
      count++;
      if(count>5){
        Serial.println("IMU timeout error");
        errorCode = 'T';  //incline error code
        return false;
      }
    }
    else{
      Serial.println("getting angles");
      count = 0;
      rovRoll = safetyIMU->GetAngX();
      rovPitch = safetyIMU->GetAngY();
    }

//    Serial.print("Roll: ");
//    Serial.print(rovRoll);
//    Serial.print("\tPitch: ");
//    Serial.println(rovPitch);

    if(abs(rovRoll)>maxIncline || abs(rovPitch)>maxIncline){
      Serial.println("Too steep!!");
      errorCode = 'I'; //incline error code
      return false;
    }
//    Serial.println("ANGLES OK");
    return true;
}
//Display splash screen
void Rover::dispSplash() const{
    lcd->setCursor(0,0);
    lcd->print("     VAR  Rover     ");
    lcd->setCursor(0,1);
    lcd->print("(: Happy Scanning :)");
    lcd->setCursor(0,3);
    lcd->print(" Calibrating..(20s) ");
}
//Display screen according to operator input, only every timeDelay
void Rover::displayLCD() const{
    static unsigned long lvlPrevTime = 0; //previous time the function was called
    static unsigned long timeDelay = 500; //ms to wait before updating LCD
    static byte curDisp = 0;              //indicates which display currently selected
    
    if(getRoverError()){
      curDisp = -1;
      dispError();
      return;
    }
    
    //only update LCD every timeDelay ms
    if(millis()-lvlPrevTime<timeDelay)
      return;
    //update timer
    lvlPrevTime = millis();

    //display desired screen
    if(channel(7)==172){
      if(1 != curDisp) lcd->clear();
      curDisp = 1;
      dispScr1();
    }
    else if(channel(7)==992){
      if(2 != curDisp) lcd->clear();
      curDisp = 2;
      dispScr2();
    }
    else if(channel(7)==1811){
      if(3 != curDisp) lcd->clear();
      curDisp = 3;
      dispScr3();
    }
    else{
      if(4 != curDisp) lcd->clear();
      curDisp = 4;
      dispScr1();
    }
}
//Display battery voltages/percentages, uptime, and current status to LCD
void Rover::dispScr1() const{
    //display battery info
    lcd->setCursor(0,0);  
    lcd->print("Front: ");
    lcd->print(FBatV);
    lcd->print("V (");
    lcd->print(FBatAmt);
    lcd->print("%)");
    lcd->setCursor(0,1);
    lcd->print("Back:  ");
    lcd->print(BBatV);
    lcd->print("V (");
    lcd->print(BBatAmt);
    lcd->print("%)");
    lcd->setCursor(0,2);
    lcd->print("Ctrl:  ");
    lcd->print(CBatV);
    lcd->print("V (");
    lcd->print(CBatAmt);
    lcd->print("%)");
    
    //calculate and display uptime info
    lcd->setCursor(0,3);
    unsigned long seconds = (millis()/1000);
    unsigned long minutes = seconds/60;
    unsigned long hours = minutes/60;
    seconds %= 60;
    minutes %= 60;
    lcd->print(hours);
    lcd->print("h");
    if(minutes<10)
      lcd->print(0);
    lcd->print(minutes);
    lcd->print("m");
    if(seconds<10)
      lcd->print(0);
    lcd->print(seconds);
    lcd->print("s");

    //display current status of the rover
    lcd->setCursor(12,3);
    if(failsafe()){
      lcd->print("FAILSAFE");
    }
    else if(isArmed()){
      lcd->print("   Armed");
    }
    else{
      lcd->print("Disarmed");
    }
}
//Display pitch/roll info and lift height to LCD
void Rover::dispScr2() const{
    lcd->setCursor(0,0);
    lcd->print("Pitch: ");
    lcd->print(rovPitch);
    lcd->setCursor(0,1);
    lcd->print("Roll:  ");
    lcd->print(rovRoll);
    lcd->setCursor(0,3);
    lcd->print("Height: ");
    lcd->print(int(liftHeight)/12);
    lcd->print("ft");
    if(int(liftHeight)%12 < 10)
      lcd->print("0");
    lcd->print(int(liftHeight)%12);
    lcd->print("in");
}
//Display screen 3 info, currently error screen, to LCD
void Rover::dispScr3() const{
    dispError();
}
//Display error messages and warnings to LCD
void Rover::dispError(){
    static unsigned long lvlPrevTime = 0; //previous time the function was called
    static unsigned long timeDelay = 1000;//ms to wait before updating LCD
    
    //only clear LCD every timeDelay ms
    if(millis()-lvlPrevTime>timeDelay){
      lcd->clear();
      lvlPrevTime = millis();
    }

    //display error
    lcd->setCursor(0,0);
    if(errorCode == 'N'){
      lcd->print("No Errors :)");
    }
    else if(errorCode == 'V'){    //bad voltage, getVoltages()
      lcd->print("Low battery!!");
    }
    else if(errorCode == 'I'){  //bad incline, getRovAngles()
      lcd->print("Maximum incline!!");
      lcd->setCursor(0,1);
      lcd->print("(or bad sensor read)");
      lcd->setCursor(0,2);
      lcd->print("Pitch: ");
      lcd->print(rovPitch);
      lcd->setCursor(0,3);
      lcd->print("Roll: ");
      lcd->print(rovRoll);
    }
    else if(errorCode == 'T'){  //safety IMU timed out, getRovAngles()
      lcd->print("Safety IMU timeout!!");
    }
    else if(errorCode == 'R'){  //bad radio, getRxData()/failsafe()
      lcd->print("No radio signal!!");
    }
    else{
      lcd->print("Unknown error");
      lcd->setCursor(0,1);
      lcd->print("Error Code: ");
      lcd->print(errorCode);
      errorCode = 'N';
    }

    //display current status of the rover
    lcd->setCursor(12,3);
    if(failsafe()){
      lcd->print("FAILSAFE");
    }
    else if(isArmed()){
      lcd->print("   Armed");
    }
    else{
      lcd->print("Disarmed");
    }

    errorCode = 'N';
}
//Enable/disable motors
void Rover::motorRelays(bool enable){
    digitalWrite(FLR,enable);
    digitalWrite(FRR,enable);
    digitalWrite(BLR,enable);
    digitalWrite(BRR,enable);
}
//Drive control
void Rover::drive(bool enable = 1){
  if(failsafe() || !armed || !enable){
    analogWrite(FL,188);
    analogWrite(BL,188);
    analogWrite(FR,188);
    analogWrite(BR,188);
    return;
  }

  const int BUFSIZE = 20;      //size of buffers
  static int bufFB[BUFSIZE];  //buffer to store incoming FWD/BWD values in
  static int bufLR[BUFSIZE];  //buffer to store incoming Left/Right values in
  static int curIndx = 0;     //current index of buffer
  int PWM_FB = 0;
  int PWM_LR = 0;
  
  if(channel(2)>150 && channel(2)<1900 && channel(3)>150 && channel(3)<1900){
    //forward/backward calculations, map RX values to PWM and get offset from center
    int newPWM_FB=int((((float(channel(3))-172)/(1811-172))*(254-122))+122)-188 ;
    //turning calculations, map RX values to PWM and get offset from center
    int newPWM_LR=int(((((float(channel(2))-172))/(1811-172))*(254-122))+122)-188;
    //put new readings in buffer
    bufFB[curIndx] = newPWM_FB;
    bufLR[curIndx] = newPWM_LR;
    //calculate average
    for(int i=0; i<BUFSIZE; i++){
      PWM_FB += bufFB[i];
      PWM_LR += bufLR[i];
    }
    PWM_FB /= BUFSIZE;
    PWM_LR /= BUFSIZE;

    Serial.print("Front/Back: ");
    Serial.print(PWM_FB);
    Serial.print("\tLeft/Right: ");
    Serial.println(PWM_LR);

    //invert LR if driving backwards, within deadzone
    if(PWM_FB<-10)
      PWM_LR = -PWM_LR;
    //write speed to motors, limit to within acceptable PWM range based on height of lift
    byte LeftOutput = constrain(188+PWM_FB+PWM_LR, 122, 254);
    byte RightOutput = constrain(188-PWM_FB+PWM_LR, 122, 254);
    analogWrite(FL,LeftOutput);
    analogWrite(BL,LeftOutput);
    analogWrite(FR,RightOutput);
    analogWrite(BR,RightOutput);
    //increment index for buffers, revolve around BUFSIZE
    curIndx = (curIndx+1)%BUFSIZE;
  }
  else{
    analogWrite(FL,188);
    analogWrite(BL,188);
    analogWrite(FR,188);
    analogWrite(BR,188);
  }
}
//Adjust leveler angle from TX input
void Rover::moveLeveler(bool enable = 1){
  static unsigned long lvlPrevTime = 0; //previous time the function was called
  unsigned long timeDelay = 500;        //ms to wait to send a message
  int maxAngleOffset = 30;              //maximum angle of allowable adjustment
  static float roll = 0;                //roll value adjustment to send to leveler
  static float pitch = 0;               //pitch value adjustment to send to leveler

  //rover in failsafe or not armed
  //stop the leveler from moving 
  if(failsafe() || !armed || !enable){
    Serial2.println("S");
    return;
  }
  //only send a message every timeDelay ms
  if(millis()-lvlPrevTime>timeDelay){
    //update timer
    lvlPrevTime = millis();
    //get increment adjustment
    float increment;
    if(channel(4)>150 && channel(4)<475) increment = -0.5;
    else if(channel(4)>1500 && channel(4)<1900) increment = 0.5;
    else increment = 0;
    //get which axis to increment
    if(channel(6)==172){  //Return to level selected
      roll = 0;
      pitch = 0;
    }
    else if(channel(6)==992){ //Pitch adjust selected
      pitch += increment;
      pitch = constrain(pitch, -maxAngleOffset, maxAngleOffset);
    }
    else if(channel(6)==1811){  //Roll adjust selected
      roll += increment;
      roll = constrain(roll, -maxAngleOffset, maxAngleOffset);
    }
    else{
      roll = 0;
      pitch = 0;
    }
    //send message
    Serial2.print("P");
    Serial2.println(pitch);
    Serial2.print("R");
    Serial2.println(roll);
  }
  return;
}
//Raise and lower lift
void Rover::lift(bool enable = 1){
    //if failsafe or disarmed disable lift
    if(failsafe() || !armed || !enable){
      digitalWrite(LEn,LOW);
      analogWrite(L1Extend,0);
      analogWrite(L1Retract,0);
      analogWrite(L2Extend,0);
      analogWrite(L2Retract,0);
      return;
    }
    
    int PWMVal = 150;  //Speed to run lift at
    int liftMin = 0;    //Minimum position to drive motor to
    int liftMax = 1023;  //Maximum position to drive motor to
    int pos1 = analogRead(L1Pos); //Current position of lift 1
    int pos2 = analogRead(L2Pos); //Current position of lift 2
    int dir1, dir2;     //Direction for actuators: 1extend, -1retract, 0stop
    int liftRange = 10;  //Acceptable allowed range of lift analog readings
    int liftDiff = 5;  //Acceptable allowed difference between two actuator positions
    int desiredPos;     //Desired position of lift
    //map ch1 to lift range
    if(channel(1)>150 && channel(1)<1900)
      desiredPos = map(channel(1),172,1811,liftMin,liftMax);
    else{
      digitalWrite(LEn,0);
      analogWrite(L1Extend,0);
      analogWrite(L1Retract,0);
      analogWrite(L2Extend,0);
      analogWrite(L2Retract,0);
      return;
    }
    
    //enable lift
    digitalWrite(LEn,HIGH);
    //determine LIFT 1 direction
    if(pos1>(desiredPos+liftRange) && (pos2-pos1)<liftDiff)       //extend
      dir1 = 1;
    else if(pos1<(desiredPos-liftRange) && (pos1-pos2)<liftDiff)  //retract
      dir1 = -1;
    else                                                          //stop
      dir1 = 0;
    //determien LIFT 2 direction
    if(pos2>(desiredPos+liftRange) && (pos1-pos2)<liftDiff)       //extend
      dir2 = 1;
    else if(pos2<(desiredPos-liftRange) && (pos2-pos1)<liftDiff)  //retract
      dir2 = -1;
    else                                                          //stop
      dir2 = 0;
    //write direction to actuators
    analogWrite(L1Retract,constrain(PWMVal*-dir1,0,255));
    analogWrite(L1Extend,constrain(PWMVal*dir1,0,255));
    analogWrite(L2Retract,constrain(PWMVal*-dir2,0,255));
    analogWrite(L2Extend,constrain(PWMVal*dir2,0,255));

//    Serial.print(desiredPos);
//    Serial.print("\t");
//    Serial.print(pos1);
//    Serial.print("\t");
//    Serial.println(pos2);
}
