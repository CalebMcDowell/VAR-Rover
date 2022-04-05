//File: VARRover.h
#ifndef VARROVER_H
#define VARROVER_H

  #include <ArduinoSTL.h> //Requires "ArduinoSTL" library (v1.3.3)
  #include <cmath>
  #include <array>
 
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
      //General
      bool armed;                                   //rover armed
      //Receiver communication
      SbusRx RX;                                    //Object for receiving
      std::array<int16_t, SbusRx::NUM_CH()> RxData; //Array for storing received data
      //Drivetrain pins
      byte FL = 1                                   //Front Left
          ,FR = 2                                   //Front Right
          ,BL = 3                                   //Back Left
          ,BR = 4;                                  //Back Right
      //Lift pins
      byte LIFT = 5;
      //Sensor pins

      //Relay pins
      
    public:
      bool init();
      bool isArmed(){return armed;}
      bool failsafe(){return RX.failsafe();};
      bool getRxData();
      int channel(byte) const;
  }; //end Rover

#endif  // VARROVER_H
