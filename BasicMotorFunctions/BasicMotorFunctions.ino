/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->  http://www.adafruit.com/products/1438
*/


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

// Create the motor shield object with default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

//200 steps per revolution (1.8 degrees/step) to M1 and M2 (port 1)
Adafruit_StepperMotor *leftMotor = AFMS.getStepper(200, 1);
//200 steps per revolution (1.8 degrees/step) to M3 and M4 (port 2)
Adafruit_StepperMotor *rightMotor = AFMS.getStepper(200, 2);

void turnLeft();
void turnRight();
void doA180();
void goForward();
void goBackward();
void sweepServo(Servo servo);

//Servo motor setup
int pos = 0;
Servo servo;

void setup() {
  //Initialize Serial object for debug/info
  Serial.begin(115200);

  //Motor shield and motor initialization
  AFMS.begin();
  leftMotor->setSpeed(100);
  rightMotor->setSpeed(100);

  //Turn on LEDs
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);

  //Set up servo motor
  //servo.attach(4);

  sweepServo(servo);

  /*for(int i = 0; i < 4; i++){
    goForward();
    turnLeft();
  }
  */
}

void loop() {
}

//Turn 90 degrees right
void turnRight(){
  for(int i = 0; i < 210; i++){
    leftMotor->step(1, FORWARD, INTERLEAVE);
    rightMotor->step(1, FORWARD, INTERLEAVE);
    delayMicroseconds(150);
  }
}

//TUrn 90 degrees left
void turnLeft(){
  for(int i = 0; i < 210; i++){
    leftMotor->step(1, BACKWARD, INTERLEAVE);
    rightMotor->step(1, BACKWARD, INTERLEAVE);
    delayMicroseconds(150);
  }
}

//Turn counter-clockwise 180 degrees
void doA180(){
  turnLeft();
  turnLeft();
}

//Move forward one foot
void goForward(){
  for(int i = 0; i < 560; i++){
    leftMotor->step(1, FORWARD, INTERLEAVE);
    rightMotor->step(1, BACKWARD, INTERLEAVE);
    delayMicroseconds(150);
  }
}

//Move backward one foot
void goBackward(){
  for(int i = 0; i < 560; i++){
    leftMotor->step(1, BACKWARD, INTERLEAVE);
    rightMotor->step(1, FORWARD, INTERLEAVE);
    delayMicroseconds(150);
  }
}


//Sweep servo motor from 45 to 135 and back
void sweepServo(Servo servo){
  servo.write(90);
  for(pos = 90; pos <= 135; pos++){
    servo.write(pos);
    delay(30);
  }
  for(pos = 135; pos >= 45; pos--){
    servo.write(pos);
    delay(30);
  }
  for(pos = 45; pos <= 90; pos++){
    servo.write(pos);
    delay(30);
  }
}

