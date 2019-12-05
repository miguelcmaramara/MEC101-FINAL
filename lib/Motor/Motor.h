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
    Motor(int enable, int logicF, int logicB, int parameter);

    void hardStop();
    void forward();
    void backward();
    void changeDir();
    void driveSpeed(int speed);//input range is 0 to parameter

    void drive(int speed); // input range is -parameter to parameter
  private:
    int _enable;
    int _logicF;
    int _logicB;
    int _parameter;

    bool _direc;

    int _speed;
    int _outputVol;
};

#endif
