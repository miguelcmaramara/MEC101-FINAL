#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Pixy.h>
#include <Motor.h>

/*

Notes: when power is plugged into the motor correctly, the output shaft will
always turn away from the red wire, towards the red wire.

*/

/************************************************************************

  Config and Setup

***********************************************************************/

//Pin Attachment
  const int servoJackPin = 0;

  const int leftDriverPin[3] = {11 , 6, 7};   //const int driverName[3] = [enable, forward, backward]
  const int rightDriverPin[3] = {3 , 4, 5};
  const int scooperPin[3] = {0 , 0, 0};

  const int leftTrig = 46; // attach pin 46 to Trig
  const int leftEcho = 47; //attach pin 47 to Echo
  const int rightTrig = 48; //attach pin 48 to Trig
  const int rightEcho = 49; //attach pin 49 to Echo

  const int modePin = 0;
  const int powerPin = 0;

  const int tiltSwitch = 0;

//class creation
  Servo servoJack;

  Motor leftMotor(leftDriverPin[0], leftDriverPin[1], leftDriverPin[2]);      //format: Motor myMotor(enable, forwardPin, backwardPin)
  Motor rightMotor(rightDriverPin[0], rightDriverPin[1], rightDriverPin[2]);
  Motor scooperMotor(scooperPin[0], scooperPin[1], scooperPin[2]);

//parameter setter
int scooperMotorSpeed = 10;
int tiltRefresh = 200;

/************************************************************************

Function Definition

************************************************************************/


  void scoopToStop(int iterations){
    iterations = constrain(iterations , 0, 10);
    int scoopCounter = 0;

    while(true){
      scooperMotor.drive(scooperMotorSpeed);

      if (digitalRead(tiltSwitch) == HIGH) {
        scoopCounter++;
        if(scoopCounter >= iterations){
          break;
        }
        delay(tiltRefresh);
      }
    }
  }

  void diffDrive(int leftSpeed, int rightSpeed) {   //optimally, input is between -10, 10 each
    leftMotor.drive(leftSpeed);
    rightMotor.drive(rightSpeed);
    Serial.println(leftSpeed);
  }

      long microsecondsToInches(long microseconds) {
        return microseconds / 74 / 2;
      }
      void sendPulse(int trigPin) {
        // The output is triggered by a HIGH pulse of 2 or more microseconds.
        // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:

        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(trigPin, LOW);
      }
      long detectAndConvert(int echoPin){
          // The echo pin is used to read the signal: a HIGH
          // pulse whose duration is the time (in microseconds) from the sending
          // of the ping to the reception of its echo off of an object.

          long duration = pulseIn(echoPin, HIGH);

          // convert the time into a distance
          return microsecondsToInches(duration);
        }
  boolean objectAquired() {
    sendPulse(leftTrig);
    long inchesL = detectAndConvert(leftEcho);
    sendPulse(rightTrig);
    long inchesR = detectAndConvert(rightEcho);

    if (inchesL >= 3.25 || inchesR >= 3.25) {
      return HIGH;
    } else {
      return LOW;
    }
  }

  boolean fetch(){
    return(modePin == 1);
  }
  boolean clean(){
    return(modePin == 0);
  }

  void resetPosition(){
    scoopToStop(1);
  }

/************************************************************************

SETUP AND LOOP

************************************************************************/
void setup() {
  //pin assignment
  servoJack.attach(servoJackPin);

  pinMode(leftTrig, OUTPUT);
  pinMode(rightTrig, OUTPUT);
  pinMode(leftEcho,INPUT);
  pinMode(rightEcho,INPUT);

  pinMode(modePin, INPUT);
  pinMode(powerPin, INPUT);

  pinMode(tiltSwitch, INPUT);

  Serial.begin(9600);
}



void loop() {

  if(powerPin == LOW){  //checks for power every 10 seconds
    while (powerPin == LOW){
      delay(10000);
    }
  }



  if (powerPin == HIGH) {    //executes actual code only if power is on
    while (powerPin == HIGH) {
    //code that matters starts here

      if(fetch()){
        resetPosition();

        while(fetch()){

        }
      }

      if(clean()){
        resetPosition();

        while (clean()) {
          /* code */
        }
      }

    }
  }

}
