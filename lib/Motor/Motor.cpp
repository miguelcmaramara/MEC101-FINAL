/*
  Motor.h: a library for operating an dc motor through an h-bridge
  Created by Miguel Maramara October, 2019
*/

#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int enable, int logicF, int logicB, int parameter)
{
  pinMode(enable, OUTPUT);
  pinMode(logicF, OUTPUT);
  pinMode(logicB, OUTPUT);

  _enable = enable;
  _logicF = logicF;
  _logicB = logicB;
  _parameter = parameter;
}

void Motor::hardStop() {
  digitalWrite(_logicF,LOW);
  digitalWrite(_logicF,LOW);
}

void Motor::forward() {
  digitalWrite(_logicF, HIGH);
  digitalWrite(_logicB, LOW);

  _direc = 1;
}

void Motor::backward(){
  digitalWrite(_logicB, HIGH);
  digitalWrite(_logicF, LOW);

  _direc = 0;
}

void Motor::changeDir(){
  if (_direc == 0){
    forward();
  } else {
    backward();
  }
}

void Motor::driveSpeed(int speed){
  _speed = speed;
  _speed = abs(_speed);
  _speed = constrain(_speed, 0, _parameter);
  _outputVol = map(_speed, 0, _parameter, 0, 255);
  analogWrite(_enable, _outputVol);
}

void Motor::drive(int speed){
  if (speed > 0){
    forward();
    driveSpeed(speed);
  } else if(speed < 0){
    backward();
    driveSpeed(speed);
  } else{
    hardStop();
  }
}
