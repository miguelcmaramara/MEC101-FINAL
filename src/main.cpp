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
//const int driverName[3] = [enable, forward, backward]

  const int leftDriverPin[3] = {11 , 6, 7};
  const int rightDriverPin[3] = {0 , 0, 0};
  const int scooperPin[3] = {0 , 0, 0};
  const int autoDoorPin[3] = {0 , 0, 0};


//class creation
Servo rightServo;
Servo leftServo;


//format: Motor myMotor(enable, forwardPin, backwardPin)
Motor leftMotor(leftDriverPin[0], leftDriverPin[1], leftDriverPin[2]);
Motor rightMotor(3, 4, 5);
Motor scooper(0, 0, 0);

//Pin z

const int rightServoPin = 10;
const int leftServoPin = 9;

int trig1 = 46; // attach pin 46 to Trig
int echo1 = 47; //attach pin 47 to Echo
int trig2 = 48; //attach pin 48 to Trig
int echo2 = 49; //attach pin 49 to Echo
int ledPin = 13;
 
void setup()
{
 // initialize serial communication:
 Serial.begin(9600);
}

void ultrasonicDetect()
{
  long durationR, inchesR, durationL, inchesL;
  
  // The output is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
 
  pinMode(trig1, OUTPUT);
  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig1, LOW);
 
  // The echo pin is used to read the signal: a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
 
  pinMode(echo1,INPUT);
  durationL = pulseIn(echo1, HIGH);
 
  // convert the time into a distance
 
  inchesL = microsecondsToInches(durationL);

  Serial.print("distance sensor 1:");
  Serial.print(inchesL);
  Serial.print("in, ");
  Serial.println();
  delay(100);
 
  pinMode(trig2, OUTPUT);
  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig2, LOW);
 
  pinMode(echo2,INPUT);
  durationR = pulseIn(echo2, HIGH);
 
  inchesR = microsecondsToInches(durationR);

  Serial.print("distance sensor 2:");
  Serial.print(inchesR);
  Serial.print("in, ");
  Serial.println();
  delay(100);

   if (inchesL >= 3.25 || inchesR >= 3.25)
 { 
    Serial.print("Object obtained")
    digitalWrite(ledPin, HIGH);
 }
  else {
    digitalWrite(ledPin, LOW);
  }
}

void loop()
{
  ultrasonicDetect();
}
 
long microsecondsToInches(long microseconds)
{
return microseconds / 74 / 2;
}

/************************************************************************

Function Definition

************************************************************************/


  //optimally, input is between -10, 10 each
  void diffDrive(int leftSpeed, int rightSpeed) {
    leftMotor.drive(leftSpeed);
    rightMotor.drive(rightSpeed);
    Serial.println(leftSpeed);
  }


/************************************************************************

SETUP AND LOOP

************************************************************************/
void setup() {
  //pin attachment
  /*
    for(int i = 0; i <3; i++){
      pinMode(leftDriverPin[i], OUTPUT);
      pinMode(rightDriverPin[i], OUTPUT);
      pinMode(scooperPin[i], OUTPUT);
      pinMode(autoDoorPin[i], OUTPUT);
    }
  */
  rightServo.attach(rightServoPin);
  leftServo.attach(leftServoPin);

  Serial.begin(9600);
}



void loop() {
  rightServo.write(70);
  leftServo.write(70);
  diffDrive(10 ,10);

  delay(2000);
  leftServo.write(30);
  rightServo.write(100);
  diffDrive(-5,-5);
  delay(1000);
  // put your main code here, to run repeatedly:

  //leftMotor.drive(10);
  //analog.analogWrite()
}

/************************************************************************

FUNCTIONS AND RELATED CODE

************************************************************************/



/*

void forward(int speed){
  digitalWrite(motorOne_InputOne, HIGH);
  digitalWrite(motorOne_InputTwo, LOW);
  digitalWrite(motorTwo_InputOne, HIGH);
  digitalWrite(motorTwo_InputTwo, LOW);

  analogWrite (motorOne_Enable, speed);
  analogWrite (motorTwo_Enable, speed);
}

void reverse(int speed){
  digitalWrite(motorOne_InputOne, LOW);
  digitalWrite(motorOne_InputTwo, HIGH);
  digitalWrite(motorTwo_InputOne, LOW);
  digitalWrite(motorTwo_InputTwo, HIGH);

  analogWrite (motorOne_Enable, speed);
  analogWrite (motorTwo_Enable, speed);
}

void turnRight(int speed){
  digitalWrite(motorOne_InputOne, HIGH);
  digitalWrite(motorOne_InputTwo, LOW);
  digitalWrite(motorTwo_InputOne, LOW);
  digitalWrite(motorTwo_InputTwo, HIGH);

  analogWrite (motorOne_Enable, speed);
  analogWrite (motorTwo_Enable, speed);

  delay(moveTime);
}

void turnLeft(int speed){
  digitalWrite(motorOne_InputOne, LOW);
  digitalWrite(motorOne_InputTwo, HIGH);
  digitalWrite(motorTwo_InputOne, HIGH);
  digitalWrite(motorTwo_InputTwo, LOW);

  analogWrite (motorOne_Enable, speed);
  analogWrite (motorTwo_Enable, speed);

  delay(moveTime);
}

void accelerate(){
  for(int i = 100; i < 225; i++){
   analogWrite (motorOne_Enable, i);
   analogWrite (motorTwo_Enable, i);
  }
}

void decelerate(){
   for(int i = 225; i > 100; i--){
   analogWrite (motorOne_Enable, i);
   analogWrite (motorTwo_Enable, i);
  }
}
*/
