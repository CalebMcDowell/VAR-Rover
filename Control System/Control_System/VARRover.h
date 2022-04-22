//File: VARRover.h
#ifndef VARROVER_H
#define VARROVER_H

  //Use "Arduino AVR Board" in Boards Manager (v1.8.3)
  #include <ArduinoSTL.h>         //Requires "ArduinoSTL" library (v1.3.3)
  #include <cmath>
  #include <array>
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>  //Requires "LiquidCrystal I2C" library (v1.1.2)
  #include <TinyMPU6050.h>        //Requires "TinyMPU6050" library (v0.5.3)
//  #include <Adafruit_Sensor.h>
//  #include <Adafruit_BNO055.h>
//  #include <utility/imumaths.h>

  
  //This class is a modified version of the one found in
  //the "Bolder Flight Systems SBUS" library (v7.0.0)
  class SbusRx {
    private:
      /* Communication */
      static constexpr uint32_t BAUD_ = 100000;
      /* Message len */
      static constexpr int8_t BUF_LEN_ = 25;
      /* SBUS message defs */
      static constexpr int8_t NUM_SBUS_CH_ = 16;
      static constexpr uint8_t HEADER_ = 0x0F;
      static constexpr uint8_t FOOTER_ = 0x00;
      static constexpr uint8_t FOOTER2_ = 0x04;
      static constexpr uint8_t CH17_MASK_ = 0x01;
      static constexpr uint8_t CH18_MASK_ = 0x02;
      static constexpr uint8_t LOST_FRAME_MASK_ = 0x04;
      static constexpr uint8_t FAILSAFE_MASK_ = 0x08;
      /* Parsing state tracking */
      int8_t state_ = 0;
      uint8_t prev_byte_ = FOOTER_;
      uint8_t cur_byte_;
      /* Buffer for storing messages */
      uint8_t buf_[BUF_LEN_];
      /* Data */
      bool new_data_;
      std::array<int16_t, NUM_SBUS_CH_> ch_;
      bool failsafe_ = false, lost_frame_ = false, ch17_ = false, ch18_ = false;
      bool Parse();
    public:
      void Begin();
      bool Read();
      static constexpr int8_t NUM_CH() {return NUM_SBUS_CH_;}
      inline std::array<int16_t, NUM_SBUS_CH_> ch() const {return ch_;}
      inline bool failsafe() const {return failsafe_;}
      inline bool lost_frame() const {return lost_frame_;}
      inline bool ch17() const {return ch17_;}
      inline bool ch18() const {return ch18_;}
  }; //end SbusRx
  class Rover{
    private:
      /*  General */
      bool armed;                                   //Rover armed/disarmed
      char errorCode;                         //Character to indicate what the error is
      float FBatV, BBatV, CBatV;                    //Voltage for Front, Back, and Control batteries
      byte FBatAmt, BBatAmt, CBatAmt;               //% charge for Front, Back, and Control batteries
      float rovPitch, rovRoll;                      //Pitch and roll values for rover
      float liftHeight;                             //Height of scissor lift, in inches
      /*  Receiver Communication  */
      SbusRx RX;                                    //Object for receiving
      std::array<int16_t, SbusRx::NUM_CH()> RxData; //Array for storing received data
      /*  LCD Display */
      LiquidCrystal_I2C* lcd;                       //LCD screen pointer, 20chars X 4rows
      /*  Drivetrain  */
      byte FL = 8;                                  //Front Left PWM
      byte FR = 9;                                  //Front Right PWM
      byte BL = 10;                                 //Back Left PWM
      byte BR = 11;                                 //Back Right PWM
      /*  Relays */
      byte FLR = 28;                                //Front Left Relay
      byte FRR = 29;                                //Front Right Relay
      byte BLR = 30;                                //Back Left Relay
      byte BRR = 31;                                //Back Right Relay
      /*  Lift  */
      byte LEn = 2;                                 //Lift Enable
      byte L1Extend = 3;                            //Lift 1 Forward PWM
      byte L1Retract = 4;                           //Lift 1 Retract PWM
      byte L1Pos = A10;                             //Lift 1 analog positional feedback
      byte L2Extend = 5;                            //Lift 2 Forward PWM
      byte L2Retract = 6;                           //Lift 2 Retract PWM
      byte L2Pos = A14;                             //Lift 2 analog positional feedback
      /*  Sensors */
      byte FBatt = A3;                              //Front battery analog reading
      byte BBatt = A7;                              //Back battery analog reading
      byte CBatt = A11;                             //Control system battery analog reading   
      MPU6050* safetyIMU;                           //IMU object for rover's angle of incline
//      Adafruit_BNO055* safetyIMU;
    public:
      /*  General Rover */
      bool init();
      bool isArmed(){return armed;}
      bool arm();
      bool disarm();
      /*  Receiver & Channels */
      bool failsafe() const;
      bool getRxData();
      int channel(byte) const;
      void printChannels() const;
      /*  Safety and Display  */
      bool getRoverError() const;
      bool getVoltages();
      bool getRovAngles();
      void motorRelays(bool);
      void dispSplash() const;
      void displayLCD() const;
      void dispScr1() const;
      void dispScr2() const;
      void dispScr3() const;
      void dispError();
      /*  Move Stuff  */
      void drive(bool = 1);
      void moveLeveler(bool = 1);
      void lift(bool = 1);
  }; //end Rover

#endif  // VARROVER_H
