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
    delay(5000);
    lcd->clear();
    //Drivetrain setup
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
    return true;
}
//Arms the rover which allows subsystems to run
bool Rover::arm(){
    armed = 1;
    motorRelays(1); //enable motor relays
}
//Disarms all rover systems
bool Rover::disarm(){
    armed = 0;
    motorRelays(0); //disable motor relays
    drive();
    moveLeveler();
    lift();

    Serial.println("DISARMED");
    return true;
}
//Get an array of the channel values. Returns 0 if error/failsafe/etc
bool Rover::getRxData(){
    //if data to be read and rx not in failsafe mode
    if(RX.Read() && !RX.failsafe()){
      //get all channel data
      RxData = RX.ch();
      //success
      return 1;
    }
    //failed
    return 0;
}
//Returns desired channel value. channel(3) return ch3
int Rover::channel(byte dch) const{
    if(dch<1 || dch>RX.NUM_CH())// || !getRxData())
      return -1;
    
    return RxData[dch-1];
}
//Print all channel data to serial monitor
void Rover::printChannels() const{
    for(int i=1; i<=16; i++){
      Serial.print("CH");
      Serial.print(i);
      Serial.print(":");
      Serial.print(channel(i));
      Serial.print(" ");
    }
    Serial.println("");
}
//Measure battery voltages
bool Rover::getVoltages(){
    int maxAn = 681;   //16.8V (max analog reading)
    int minAn = 536;   //13.2V (min analog reading)
  
    //read sensors
    int FrontAn = analogRead(FBatt);
    int BackAn = analogRead(BBatt);
    int ControlAn = analogRead(CBatt);
    
    //equation based on y = 40.463x+0.9332, derived from voltage sensor testing
    FBatV = (float(FrontAn)-0.9332)/40.463;
    BBatV = (float(BackAn)-0.9332)/40.463;
    CBatV = (float(ControlAn)-0.9332)/40.463;
    
    //map analog values to percentage charge
    FBatAmt = map(constrain(FrontAn,minAn,maxAn),minAn,maxAn,0,100);
    BBatAmt = map(constrain(BackAn,minAn,maxAn),minAn,maxAn,0,100);
    CBatAmt = map(constrain(ControlAn,minAn,maxAn),minAn,maxAn,0,100);
  
  //  Serial.print("FV: ");
  //  Serial.print(FBatV);
  //  Serial.print("\tF%: ");
  //  Serial.println(FBatAmt);
  
    //check for low voltage
    if(FrontAn<minAn){// || BackAn<minAn || ControlAn<minAn){
      Serial.println("Low Battery!!");
      return false;
    }
    return true;    
}
//Display splash screen
void Rover::dispSplash() const{
    lcd->setCursor(0,1);
    lcd->print("     VAR  Rover     ");
    lcd->print("(: Happy Scanning :)");
}
//Display screen according to operator input, only every timeDelay
void Rover::displayLCD() const{
    static unsigned long lvlPrevTime = 0; //previous time the function was called
    static unsigned long timeDelay = 500; //ms to wait before updating LCD
    static byte curDisp = 0;              //indicates which display currently selected
    
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
//    lcd->clear();
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
//    lcd->clear();
    lcd->setCursor(0,0);
    lcd->print("Screen 2");
}
//Display screen 3 info, currently error screen, to LCD
void Rover::dispScr3() const{
    dispError();
}
//Display error messages and warnings to LCD
void Rover::dispError() const{
    static unsigned long lvlPrevTime = 0; //previous time the function was called
    static unsigned long timeDelay = 500; //ms to wait before updating LCD
    
    //only clear LCD every timeDelay ms
    if(millis()-lvlPrevTime>timeDelay){
      lcd->clear();
      lvlPrevTime = millis();
    }

    //display error
    lcd->setCursor(0,0);
    if(roverError){
      lcd->print("ERROR");
    }
    else{
      lcd->print("No errors :)");
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
}
//Enable/disable motors
void Rover::motorRelays(bool enable){
    digitalWrite(FLR,enable);
    digitalWrite(FRR,enable);
    digitalWrite(BLR,enable);
    digitalWrite(BRR,enable);
}
//Drive control
void Rover::drive(){
  if(failsafe() || !armed){
    analogWrite(FL,188);
    analogWrite(BL,188);
    analogWrite(FR,188);
    analogWrite(BR,188);
    return;
  }
  if(channel(2)>150 && channel(2)<1900 && channel(3)>150 && channel(3)<1900){
    //forward/backward calculations, map RX values to PWM and get offset from center
    int PWM_FB=int((((float(channel(3))-172)/(1811-172))*(254-122))+122)-188 ;
    //turning calculations, map RX values to PWM and get offset from center
    int PWM_LR=int(((((float(channel(2))-172))/(1811-172))*(254-122))+122)-188;
    //invert LR if driving backwards
    if(PWM_FB<0)
      PWM_LR = -PWM_LR;
    //write speed to motors, limit to within acceptable PWM range
    byte LeftOutput = constrain(188+PWM_FB+PWM_LR, 122, 254);
    byte RightOutput = constrain(188-PWM_FB+PWM_LR, 122, 254);
    analogWrite(FL,LeftOutput);
    analogWrite(BL,LeftOutput);
    analogWrite(FR,RightOutput);
    analogWrite(BR,RightOutput);
  }
  else{
    analogWrite(FL,188);
    analogWrite(BL,188);
    analogWrite(FR,188);
    analogWrite(BR,188);
  }
}
//Adjust leveler angle from TX input
void Rover::moveLeveler(){
  static unsigned long lvlPrevTime = 0; //previous time the function was called
  unsigned long timeDelay = 500;        //ms to wait to send a message
  int maxAngleOffset = 30;              //maximum angle of allowable adjustment
  static float roll = 0;                //roll value adjustment to send to leveler
  static float pitch = 0;               //pitch value adjustment to send to leveler

  //rover in failsafe or not armed
  //stop the leveler from moving 
  if(failsafe() || !armed){
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
void Rover::lift(){
    //if failsafe or disarmed disable lift
    if(failsafe() || !armed){
      digitalWrite(LEn,LOW);
      analogWrite(LExtend,0);
      analogWrite(LRetract,0);
      return;
    }
    
    byte PWMVal = 255;  //Speed to run lift at
    int liftMin = 0;   //Minimum position to drive motor to
    int liftMax = 1023;  //Maximum position to drive motor to
    int liftPos = analogRead(LPos); //Current position of lift
    int liftRange = 5;  //Acceptable allowed range of lift analog readings
    int desiredPos;     //Desired position of lift
    //map ch1 to lift range
    if(channel(1)>150 && channel(1)<1900)
      desiredPos = map(channel(1),172,1811,liftMin,liftMax);
    else
      return;
    //enable lift
    digitalWrite(LEn,HIGH);
    //retract
    if(liftPos > (desiredPos+liftRange)){
      analogWrite(LExtend,0);
      analogWrite(LRetract,PWMVal);
    }
    //extend
    else if(liftPos < (desiredPos-liftRange)){
      analogWrite(LRetract,0);
      analogWrite(LExtend,PWMVal);
    }
    //within desired range
    else{
      analogWrite(LExtend,0);
      analogWrite(LRetract,0);
    }
}
