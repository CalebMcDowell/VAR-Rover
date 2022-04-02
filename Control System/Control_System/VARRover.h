//File: VARRover.h
#ifndef VARROVER_H
#define VARROVER_H

  #include "sbus.h" //Requires modified "Bolder Flight Systems SBUS" library (v7.0.0)
                    //Requires "ArduinoSTL" library (v1.3.3)
                    //To modify, find "sbus.h" file in library. Line 34 add "#include <Arduino.h>". Lines 35&36 comment out.

  class Rover{
    private:
      bool armed;                                         //rover armed
      std::array<int16_t, bfs::SbusRx::NUM_CH()> RxData;  //Array for storing received data
      bfs::SbusRx RX{&Serial};  //Object for receiving
    public:
      Rover();
      bool isArmed(){return armed;}
      bool getRxData();
      int channel(char) const;
  }; //end Rover

#endif  //_VARROVER_H
