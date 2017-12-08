#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <MultiStepper.h>
#include <AccelStepper.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <Servo.h>

//Define pins
#define LED1PIN 2
#define LED2PIN 3
#define SERVOPIN 4
#define ULTRASONICPIN 5
#define PHOTOPIN 0

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
//void sweepServo(Servo servo); //Temporarily obsolete, keep for P2
bool checkForObstacles(int ultraSonicPin);
int ultrasonicDistance(int ultraSonicPin);
int checkForDestination(int photoPin);

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
  pinMode(LED1PIN, OUTPUT);
  pinMode(LED2PIN, OUTPUT);
  digitalWrite(LED1PIN, HIGH);
  digitalWrite(LED2PIN, HIGH);

  //Set up servo motor
  servo.attach(SERVOPIN);

  //Set up ultrasonic
  pinMode(ULTRASONICPIN, OUTPUT);

  //Debug REMOVE ME
  //while(1);
}

void loop(){
  Serial.println(analogRead(PHOTOPIN));
  delay(500);
  /* //"Wander protocol"
  if(checkForObstacles(ULTRASONICPIN)){
    srand(micros());
    if((rand() % 2) == 0){
      turnLeft();
    }
    else{
      turnRight();
    }
  }
  if(!checkForObstacles(ULTRASONICPIN)){
    goForward();
  }*/
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

//Temporarily obsolete, keep for robot project 2
/*//Sweep servo motor from 45 to 135 and back
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
}*/

bool checkForObstacles(int ultraSonicPin){
  int i = 0;
  servo.write(90);
  if(ultrasonicDistance(ULTRASONICPIN) < 13){
      i++;
  }
  for(pos = 90; pos <= 115; pos++){
    servo.write(pos);
    delay(30);
  }
  if(ultrasonicDistance(ULTRASONICPIN) <= 13){
      i++;
  }
  for(pos = 115; pos >= 65; pos--){
    servo.write(pos);
    delay(30);
  }
  if(ultrasonicDistance(ULTRASONICPIN) <= 13){
      i++;
  }
  for(pos = 65; pos <= 90; pos++){
    servo.write(pos);
    delay(30);
  }
  bool found = false;
  if(i > 1){
    found = true;
  }
  return found;
}

int ultrasonicDistance(int ultraSonicPin){
  pinMode(ultraSonicPin, OUTPUT);
  digitalWrite(ultraSonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultraSonicPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(ultraSonicPin, LOW);
  pinMode(ultraSonicPin, INPUT);
  int duration = pulseIn(ultraSonicPin, HIGH);
  return duration / 74 / 2;
}


int checkForDestination(int photoPin){
  
}

