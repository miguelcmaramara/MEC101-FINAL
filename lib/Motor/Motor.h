/*
  Motor.h: a library for operating an dc motor through an h-bridge
  Created by Miguel Maramara October, 2019
*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor
{
  public:
    Motor(int enable, int logicF, int logicB);

    void hardStop();
    void forward();
    void backward();
    void changeDir();
    void driveSpeed(int speed);//input range is 0 to 10

    void drive(int speed); // input range is -10 to 10
  private:
    int _enable;
    int _logicF;
    int _logicB;

    bool _direc;

    int _speed;
    int _outputVol;
};

#endif
