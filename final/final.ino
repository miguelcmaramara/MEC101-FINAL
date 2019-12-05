#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Pixy2.h>
#include <Motor.h>

/*

Notes: when power is plugged into the motor correctly, the output shaft will
always turn away from the red wire, towards the red wire.

*/

/************************************************************************

  Config and Setup

***********************************************************************/

//Pin Attachment
        const int jackPin = 9;
      
        const int leftDriverPin[3] = {5 , 22, 23};   //const int driverName[3] = [enable, forward, backward]
        const int rightDriverPin[3] = {6 , 24, 25};
        const int scooperPin[3] = {7 , 26, 27};
      
        const int trig = 46; // attach pin 46 to Trig
        const int echo = 47; //attach pin 47 to Echo
      
        const int modePin = 0;
        const int powerPin = 40;
      
        const int tiltSwitch = 41;

 //const int
        const int red =1;
        const int green = 2;
        const int yellow = 3;
  
//parameter setter
        int scooperMotorSpeed = 10;
        int tiltRefresh = 200;
        int ultrasonicRefresh = 100;
        int motorParameter = 1000;
        long pgain = 10;
        long dgain = 1;
        int midX = (316/2);
        int midY = (208/2);
        long vel = 0;

//class creation
        Servo jack;
      
        Motor leftMotor(leftDriverPin[0], leftDriverPin[1], leftDriverPin[2], motorParameter);      //format: Motor myMotor(enable, forwardPin, backwardPin)
        Motor rightMotor(rightDriverPin[0], rightDriverPin[1], rightDriverPin[2], motorParameter);
        Motor scooperMotor(scooperPin[0], scooperPin[1], scooperPin[2], motorParameter);
      
        Pixy2 pixy;

/************************************************************************

Function Definition

************************************************************************/

//pixy shortcuts  
    int objSig(int ID){
       pixy.ccc.blocks[ID].m_signature;
    }
    
    int objX(int ID){
      return pixy.ccc.blocks[ID].m_x;
    }

    int objY(int ID){
      return pixy.ccc.blocks[ID].m_y;
    }

    int objWidth(int ID){
         return pixy.ccc.blocks[ID].m_width;
    }

    int objHeight(int ID){
        return pixy.ccc.blocks[ID].m_height;
    }

    int objIndex(int ID){
        return pixy.ccc.blocks[ID].m_index;
    }

    int objAge(int ID) {
        return pixy.ccc.blocks[ID].m_age;
    }

//motor control
      void diffDrive(int leftSpeed, int rightSpeed) {
        leftMotor.drive(leftSpeed);
        rightMotor.drive(rightSpeed);
        Serial.println(leftSpeed);
      }

//      void forwardDrive(int forwardparameter){
//        
//      }

//ultrasonic sensor detect

      long microsecondsToCentimeters(long microseconds) {
        return microseconds / 29 / 2;
      }
      long ultrasonic(int trigPin, int echoPin) {
        // The output is triggered by a HIGH pulse of 2 or more microseconds.
        // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:

        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(trigPin, LOW);

        // The echo pin is used to read the signal: a HIGH
        // pulse whose duration is the time (in microseconds) from the sending
        // of the ping to the reception of its echo off of an object.

        long duration = pulseIn(echoPin, HIGH);

        // convert the time into a distance
        return microsecondsToCentimeters(duration);
      }

      boolean objectAquired() {
          long centimeters = ultrasonic(trig, echo);
          Serial.println(centimeters);
          if (centimeters < 12) {
            delay(ultrasonicRefresh);
      
            if (centimeters < 10) {
              return HIGH;
            } else {
              return LOW;
            }
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
      
            Serial.println(ultrasonic(trig,echo));
        }
        scoopToStop(0);
      }
    
      void liftback(){
          jack.write(55);
          delay(1500);
          jack.write(0);
          delay(1500);
      }

    int findObject(int objectNeeded){
       int blockSig = -1;
        // grab blocks!
        pixy.ccc.getBlocks();
        
        for(int i = 0; i < pixy.ccc.numBlocks; i++){
            if(objectNeeded == pixy.ccc.blocks[i].m_signature){
                blockSig = objIndex(i);

            }
        }
        return blockSig;
      }

     int findLargestObject(int objectNeeded){
       int blockSig = -1;
       int winningSize = -1;
        // grab blocks!
        pixy.ccc.getBlocks();
         //this will look for the object needed
          for(int i = 0; i < pixy.ccc.numBlocks; i++){
              if(objectNeeded == pixy.ccc.blocks[i].m_signature ){
                  int objSize = pixy.ccc.blocks[i].m_height;
                  
                  if(winningSize < objSize){
                    blockSig = objIndex(i);
                    winningSize = objSize;
                  }
              }
          }
          return blockSig;
      }  

      int findLargestRedGreen(){
       int blockSig = -1;
       int winningSize = -1;
        // grab blocks!
        pixy.ccc.getBlocks();
         //this will look for the object needed
          for(int i = 0; i < pixy.ccc.numBlocks; i++){
              if(red == pixy.ccc.blocks[i].m_signature || green == pixy.ccc.blocks[i].m_signature){
                  int objSize = pixy.ccc.blocks[i].m_height;
                  
                  if(winningSize < objSize){
                    blockSig = objSig(i);
                    winningSize = objSize;
                  }
              }
          }
          return blockSig;
      }  

      int findRedGreenX(){
       int blockX = -1;
       int winningSize = -1;
        // grab blocks!
        pixy.ccc.getBlocks();
         //this will look for the object needed
          for(int i = 0; i < pixy.ccc.numBlocks; i++){
              if(red == pixy.ccc.blocks[i].m_signature || green == pixy.ccc.blocks[i].m_signature){
                  int objSize = pixy.ccc.blocks[i].m_height;
                  
                  if(winningSize < objSize){
                    blockX = objX(i);
                    winningSize = objSize;
                  }
              }
          }
          return blockX;
      }  

     int findLargestBlind(){
       int blockSig = -1;
       long winningSize = -1;
        // grab blocks!
        pixy.ccc.getBlocks();
         //this will look for the object needed
          for(int i = 0; i < pixy.ccc.numBlocks; i++){
              long objSize = pixy.ccc.blocks[i].m_height;
              
              if(winningSize < objSize){
                blockSig = objIndex(i);
                winningSize = objSize;
              }
          }
          
          return blockSig;
      }

      void searchRedGreen(){ 
        while(true){
            
            
            if(findLargestRedGreen() !=-1){
              Serial.println("found");
              
              delay(10000);
              break;
            }
Serial.println(findLargestRedGreen());
            diffDrive(-500,500);
            delay(100);
            diffDrive(0,0);
            Serial.println("turning");
         }
      }

      void search() {
        int i;
        pixy.ccc.getBlocks();
          for (int i = 0; i<= pixy.ccc.numBlocks; i++) {
            if(pixy.ccc.blocks[i].m_signature != 1 || pixy.ccc.blocks[i].m_signature != 2){
              diffDrive(500, 0);
            }
          }
      }

     void objectDrive(){\
        
        if(findRedGreenX() != -1){
            long error = (findRedGreenX() - midX);
//            vel = pgain*error;
//            if(error>0){
//              vel = ((pgain*error) - (dgain*vel));
//            } else if (error<0){
//              vel = ((pgain*error) + (dgain*vel));
//            }
    
           diffDrive((500+error),(500-error));
            
            Serial.println(error);
            delay(100);
        } else {
          Serial.println("no object Found");
        }
     }
/************************************************************************

SETUP AND LOOP

************************************************************************/
void setup() {
  //pin assignment
  jack.attach(jackPin);

  pinMode(trig, OUTPUT);
  pinMode(echo,INPUT);

  pinMode(modePin, INPUT);
  pinMode(powerPin, INPUT);

  pinMode(tiltSwitch, INPUT);

  //Serial.begin(9600);

  Serial.begin(115200);
  Serial.print("Starting...\n");

  pixy.init();


}

void loop() {
   //searchRedGreen();
   // Serial.println(findLargestRedGreen());
   //search();
    //Serial.println(findRedGreenX());
   objectDrive();
 
}
