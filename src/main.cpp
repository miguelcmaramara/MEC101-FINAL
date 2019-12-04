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

  const int leftDriverPin[3] = {5 , 22, 23};   //const int driverName[3] = [enable, forward, backward]
  const int rightDriverPin[3] = {6 , 24, 25};
  const int scooperPin[3] = {7 , 26, 27};

  const int leftTrig = 46; // attach pin 46 to Trig
  const int leftEcho = 47; //attach pin 47 to Echo
  const int rightTrig = 48; //attach pin 48 to Trig
  const int rightEcho = 49; //attach pin 49 to Echo

  const int modePin = 0;
  const int powerPin = 40;

  const int tiltSwitch = 41;

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

//motor control
  void diffDrive(int leftSpeed, int rightSpeed) {   //optimally, input is between -10, 10 each
    leftMotor.drive(leftSpeed);
    rightMotor.drive(rightSpeed);
    Serial.println(leftSpeed);
  }

//ultrasonic sensor detect

      long microsecondsToCentimeters(long microseconds) {
        return microseconds / 29 / 2;
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
          return microsecondsToCentimeters(duration);
        }
  boolean objectAquired() {
    sendPulse(leftTrig);
    long inchesL = detectAndConvert(leftEcho);
    sendPulse(rightTrig);
    long inchesR = detectAndConvert(rightEcho);

    if (inchesL >= 4.5 || inchesR >= 4.5) {
      return HIGH;
    } else {
      return LOW;
    }
  }

//pin mode states
  boolean fetch(){
    return(modePin == 1);
  }
  boolean clean(){
    return(modePin == 0);
  }

//scoops for x iterations
    void scoopToStop(int iterations){
    iterations = constrain(iterations , 0, 10);
    int scoopCounter = 0;

    while(true){
      scooperMotor.drive(scooperMotorSpeed);

      if (digitalRead(tiltSwitch) == HIGH) {
        scoopCounter++;
        if(scoopCounter >= iterations){
          scooperMotor.drive(0);
          break;
        }
        delay(tiltRefresh);
      }
    }
  }


  void resetPosition(){
    scoopToStop(1);
  }

//once object detected inside carriage, scooping stops
  void scoopToObject(){
    scooperMotor.drive(scooperMotorSpeed);

    while (true) {
      if (objectAquired()){
        break;
      }
    }
    scoopToStop(1);
  }

  void liftback(){
    servoJack.write(55);
    delay(1500);
    servoJack.write(0);
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
sendPulse(leftTrig);
long centimeters = detectAndConvert(leftEcho);
Serial.println(centimeters);


/*
  if(digitalRead(powerPin) == LOW){  //checks for power every 10 seconds
    while (digitalRead(powerPin) == LOW){
      delay(10000);
      Serial.println(digitalRead(powerPin));
    }
  }
  */
  //else if (digitalRead(powerPin) == HIGH) {    //executes actual code only if power is on
    //while (digitalRead(powerPin) == HIGH) {
    //code that matters starts here
    //delay(500);
    /*
    resetPosition();
    delay(1000);

    Serial.println(digitalRead(powerPin));
    diffDrive(10, 10);
    //delay(10000);
    scoopToObject();
    */
    //diffDrive(0,0);
    //liftback();
    //servoJack.write(55);
    //delay(1500);
    //servoJack.write(0);
    //delay(1000);
    /*
      if(fetch()){
        resetPosition();

        while(fetch()){

        }
      }

      if(clean()){
        resetPosition();

        while (clean()) {

        }
      }
      */
    //}
  //}

}
