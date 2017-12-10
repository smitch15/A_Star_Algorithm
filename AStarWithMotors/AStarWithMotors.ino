
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <MultiStepper.h>
#include <AccelStepper.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <Servo.h>

#define OBSTACLE_SIZE 4
#define ORIGINALSOURCENODE

//Define pins
#define LED1PIN 2
#define LED2PIN 3
#define SERVOPIN 4
#define ULTRASONICPIN 5
#define PHOTOPIN 0

void aStar(int sourceX, int SourceY);
void turnLeft();
void turnRight();
void doA180();
void goForward();
void goBackward();
//void sweepServo(Servo servo); //Temporarily obsolete, keep for P2 + source code
bool checkForObstacles();
int ultrasonicDistance();
bool checkForDestination(int destX, int destY, int srcX, int srcY);
void dance();

//Class for keeping track of nodes in graph
class Node{
 public:
  byte parentXY;
  byte hCost;
  byte gCost;
  byte infoBits = 0;
  /*Info bits is used as follows:
      -infoBits[0] = If node is currently in the queue
      -infoBits[1] = If node has been through the queue yet
      -infoBits<2:3> = Direction of next node in path
      -infoBits[4] = If node is in path
      -infoBits[5] = If node is obstacle
      -infoBits[6] = If node is source
      -infoBits[7] = If node is destination
   */
};

//Initialize graph
Node graph[16][16];

//Servo motor setup
Servo servo;

//Set source and destination
byte srcXandY[] = {0,1};
byte destXandY[] = {3,3};

//Array for x,y of nodes in path
byte inPath[255];

//Holds direction relative to graph. 0 = N, 1 = E, 2 = S, 3 = W
byte initDir = 0;

// Create the motor shield object with default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

//200 steps per revolution (1.8 degrees/step) to M1 and M2 (port 1)
Adafruit_StepperMotor *leftMotor = AFMS.getStepper(200, 1);
//200 steps per revolution (1.8 degrees/step) to M3 and M4 (port 2)
Adafruit_StepperMotor *rightMotor = AFMS.getStepper(200, 2);

void setup() {
  //Initialize Serial object for debug/info
  Serial.begin(115200);

  //Motor shield and motor initialization
  AFMS.begin();
  leftMotor->setSpeed(120);
  rightMotor->setSpeed(120);
  Wire.setClock(400000);

  //Turn on LEDs
  pinMode(LED1PIN, OUTPUT);
  pinMode(LED2PIN, OUTPUT);
  digitalWrite(LED1PIN, HIGH);
  digitalWrite(LED2PIN, HIGH);

  //Set up servo motor
  servo.attach(SERVOPIN);
  servo.write(90);

  //Set up ultrasonic
  pinMode(ULTRASONICPIN, OUTPUT);

//  int num = 0;
//  for(int i = 0; i < 16; i++){
//     for(int j = 0; j < 16; j++){
//        if (graph[i][j].infoBits & 1000){
//          num++;
//        }
//      }
//   }

  //Build path with A*
  aStar(srcXandY[0], srcXandY[1]);
}

void loop() {
  //POINT OF POTENTIAL TOM FOOLERY --> check format of array
  //Loop through array, check next node in path, check if obstacle, if not then move to it. Else replan
  unsigned int currentX = srcXandY[0];
  unsigned int currentY = srcXandY[1];
  bool atDest = true;
  for (int i = inPath[0]*2 - 1; i >= 3; i = i){
    Serial.print(currentX); Serial.print(", "); Serial.println(currentY);
    //If x's are different
    if (inPath[i] != inPath[i-2]){
      //Needs to go left
      if (inPath[i] > inPath[i-2]){
        //Orient bot
        if (initDir != 3){
          switch(initDir){
            case 0:
              turnLeft();
              break;
            case 1:
              doA180();
              break;
            case 2:
              turnRight();
              break;
            default:
              break;
          }
        }

        //Update direction
        initDir = 3;
      }
      
      //Needs to go right
      else{
        //Orient bot
        if (initDir != 1){
          switch(initDir){
            case 0:
              turnRight();
              break;
            case 3:
              doA180();
              break;
            case 2:
              turnLeft();
              break;
            default:
              break;
          }
        }

        //Update direction
        initDir = 1;
      }
    }

    //If y's are different
    else{
      //Needs to go up
      if (inPath[i+1] < inPath[i-1]){
        //Orient bot
        if (initDir != 0){
          switch(initDir){
            case 3:
              turnRight();
              break;
            case 1:
              turnLeft();
              break;
            case 2:
              doA180();
              break;
            default:
              break;
          }
        }
        
        //Update direction
        initDir = 0;
      }

      //Needs to go down
      else{
        //Orient bot
        if (initDir != 2){
          switch(initDir){
            case 0:
              doA180();
              break;
            case 3:
              turnLeft();
              break;
            case 1:
              turnRight();
              break;
            default:
              break;
          }
        }
        //Update direction
        initDir = 2;
      }
    }

    //Check if next node is an obstacle -- temporarily removed for debugging
    if(checkForObstacles()){
      //Add the obstacle
      graph[inPath[i - 2]][inPath[i - 1]].infoBits |= 0b00000100;
      
      //Reset the graph
      for(int j = 0; j < 16; j++){
        for(int k = 0; k < 16; k++){
          graph[j][k].infoBits &= 0b00000101;
        }
      }

      //Update source
      graph[currentX][currentY].infoBits |= 0b00000010;
      
      //Dynamic replan with new obstacle
      aStar(currentX, currentY);
      i = inPath[0] * 2 - 1;
    }

    //If not an obstacle, go forward
    else{
        goForward();
        currentX = inPath[i - 2];
        currentY = inPath[i - 1];
        if(checkForDestination(destXandY[0], destXandY[1], currentX, currentY)){
          Serial.println("Arrived at destination!");
          atDest = true;
        }
        i-=2;
    }
  }
  while(atDest){
    dance();
  }
}



void aStar(int sourceX, int sourceY){
  //Set obstacle x's and y's 
  const PROGMEM uint8_t obstacleX[] = {0,2,2,2};
  const PROGMEM uint8_t obstacleY[] = {2,3,2,0};
  
  //Set all g and h costs of each node in graph to infinity(255)
  for (byte i = 0; i < 16; i++){
    for (byte j = 0; j < 16; j++){
        graph[i][j].gCost = 255;
        graph[i][j].hCost = 255;
    }
  }
  
  //Set start node g cost to zero
  graph[sourceX][sourceY].gCost = 0;
  
  //Set heuristic value of start node
  graph[sourceX][sourceY].hCost = abs((destXandY[0] - sourceX)) + abs((destXandY[1] - sourceY));

  //Put the start node in the queue, set the source node bit
  graph[sourceX][sourceY].infoBits |= 0b10001010; 
    
  //Set the info bit for known node obstacles
  for (byte i = 0; i < OBSTACLE_SIZE; i++){
    (graph[obstacleX[i]][obstacleY[i]]).infoBits |= 0b100;
  }

  //Set the destination node bits
  graph[destXandY[0]][destXandY[1]].infoBits |= 0b1001;

  //Set the in queue bit for the start node
  graph[sourceX][sourceY].infoBits |= 0b10000000;
  //Set source's parent x y to itself
  graph[sourceX][sourceY].parentXY = (sourceX << 4) | sourceY;
  //Set direction bit of source (started out facing east)
  graph[sourceX][sourceY].infoBits |= 0b000000;


  
  //Start A star algorithm
  //Setup
  bool queueEmpty = true;
  for (byte i = 0; i < 16; i++){
    for (byte j = 0; j < 16; j++){
      if (graph[i][j].infoBits & 0b10000000){
        queueEmpty = false;
      }
    }
  }
  
  //Algorithm start
  while (!queueEmpty){
    Node *current;
    uint16_t minScore = 510;
    byte xMin, yMin = 17;
    for (byte i = 0; i < 16; i++){
      for (byte j = 0; j < 16; j++){
        if (graph[i][j].hCost + graph[i][j].gCost < minScore && 
        graph[i][j].infoBits & 0b10000000){
          xMin = i; 
          yMin = j;
          current = &graph[i][j];
          minScore = current->hCost + current->gCost;
        }
      }
    }
    if (current->infoBits & 0b1){ // if current is destination
      byte sourceXY = (sourceX << 4) | sourceY;
      Serial.println();

      Serial.println();
      Serial.println("Found Destination Node");
      Serial.println(); 
      byte count = 0;
      byte index = 3; 
      inPath[1] = destXandY[0];
      inPath[2] = destXandY[1];
      while (current->parentXY != sourceXY){
        current->infoBits |= 0b1000; // add to path
        count++;
        inPath[index] = current->parentXY >> 4;
        index++;
        inPath[index] = current->parentXY &0b1111;
        index++;
        current = &(graph[current->parentXY >> 4][current->parentXY & 0b1111]);
        if (current->infoBits & 0b1000){
          Serial.println("ADDED TO PATH");
        }
        Serial.print(count);
        Serial.println(" nodes in path, excluding source/destination");
        Serial.print("Current parent X, Y = ");
        Serial.print(current->parentXY >> 4);
        Serial.print(", ");
        Serial.println(current->parentXY & 0b1111);
        Serial.print("Source X, Y = ");
        Serial.print(sourceX);
        Serial.print(", ");
        Serial.println(sourceY);
        Serial.println();
      }
      inPath[0] = count + 2;
      inPath[count*2+3] = sourceX;
      inPath[count*2+4] = sourceY;
      Serial.print(count);
      Serial.println(" ADDED TO PATH");
      Serial.println("SUCCESS");
      Serial.println();
      //Debug
      Serial.println("AFTER A STAR");
      Serial.println("in path coords.");
      for (int k = 0; k <= inPath[0] * 2; k++){
        Serial.print(inPath[k]);
        if(k == (inPath[0] * 2)){
          Serial.println();
        }
        else if((k % 2) == 1){
          Serial.print(", ");
        }
        else{
          Serial.print("; ");
        }
      }
      Serial.println();
      return;
    }
    
    //Remove min score from queue
    current->infoBits &= 0b01111111;  //Remove from queue bit
    current->infoBits |= 0b01000000;  //Add to visited 
    
    ////////// setup for loop //////////////////////
    //Check if neighbor is new, then add it to the open queue 
    //Not in the queue and not been visited
    Node *rightNeb, *leftNeb, *topNeb, *botNeb;
    Node* nebArr[4];
    for (int i=0;i<4;i++){
      nebArr[i] = 0;
    }
    //Must initialize all the neighbors' direction bits
    if (xMin != 15){
      rightNeb = &graph[xMin+1][yMin];
      rightNeb->infoBits |= 0b010000;
      nebArr[0] = rightNeb;
      
    }
    if (xMin != 0){
      leftNeb = &graph[xMin-1][yMin];
      leftNeb->infoBits |= 0b110000;
      nebArr[1] = leftNeb;
    }
    if (yMin != 0){
      topNeb = &graph[xMin][yMin-1];
      topNeb->infoBits |= 0b000000;
      nebArr[2] = topNeb;
    }
    if (yMin != 15){
      botNeb = &graph[xMin][yMin+1];
      botNeb->infoBits |= 0b100000;
      nebArr[3] = botNeb;
    }
    byte dist, h_offset;
    for (int i=0;i<4;i++){
      if (nebArr[i] == 0)  continue;  // no neighbor
      if (nebArr[i]->infoBits & 0b100)  continue; // is an obstacle
      if (nebArr[i]->infoBits & 0b01000000) continue; // in closed set
      if (!(nebArr[i]->infoBits & 0b10000000)) 
        nebArr[i]->infoBits |= 0b10000000; // add to queue
      
      //Update direction bits
      //Define the cost of each neighbor
      switch (((current->infoBits & 0b110000) ^ (nebArr[i]->infoBits & 0b110000)) >> 4){
        case 0:
          dist = 1;
//          current node stays the same direction
          break;
        case 1:
          dist = 2;
//          current->infoBits |= nebArr[i]->infoBits & 0b110000;
          break;
        case 2:
          dist = 3;
//          current->infoBits |= nebArr[i]->infoBits & 0b110000;
          break;
        case 3:
          dist = 2;
//          current->infoBits |= nebArr[i]->infoBits & 0b110000;
          break;
      }
      while (dist == 100) Serial.println("PROBLEM WITH DIST SWITCH STATEMENT");
      byte tent_cost = current->gCost + dist;
      if (tent_cost >= nebArr[i]->gCost)  continue;
      nebArr[i]->parentXY = (xMin << 4) | yMin;
      
      nebArr[i]->gCost = tent_cost;
      nebArr[i]->hCost = abs(destXandY[0] - xMin) + abs(destXandY[1] - yMin) -1;
     
    }

    queueEmpty = true;
    for (byte i = 0; i < 16; i++){
      for (byte j = 0; j < 16; j++){
        if (graph[i][j].infoBits & 0b10000000){
          queueEmpty = false;
        } 
      }
    }
    
  }
}

//Turn 90 degrees right
void turnRight(){
  for(int i = 0; i < 212; i++){
    leftMotor->step(1, FORWARD, INTERLEAVE);
    rightMotor->step(1, FORWARD, INTERLEAVE);
    delayMicroseconds(250);
  }
}

//TUrn 90 degrees left
void turnLeft(){
  for(int i = 0; i < 212; i++){
    leftMotor->step(1, BACKWARD, INTERLEAVE);
    rightMotor->step(1, BACKWARD, INTERLEAVE);
    delayMicroseconds(250);
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
    delayMicroseconds(250);
  }
}

//Move backward one foot
void goBackward(){
  for(int i = 0; i < 560; i++){
    leftMotor->step(1, BACKWARD, INTERLEAVE);
    rightMotor->step(1, FORWARD, INTERLEAVE);
    delayMicroseconds(250);
  }
}

//Temporarily obsolete, keep for robot project 2
/*//Sweep servo motor from 45 to 135 and back
void sweepServo(Servo servo){
  servo.write(90);
  for(int pos = 90; pos <= 135; pos++){
    servo.write(pos);
    delay(30);
  }
  for(int pos = 135; pos >= 45; pos--){
    servo.write(pos);
    delay(30);
  }
  for(int pos = 45; pos <= 90; pos++){
    servo.write(pos);
    delay(30);
  }
}*/


//Checks for obstacles, returns true if one is found, false otherwise
bool checkForObstacles(){
  //Tracks # of times an obstacle is detected
  int detected = 0;

  //Set servo to 90 degree position
  servo.write(90);

  //Check if obstacle is in the way
  if(ultrasonicDistance() < 12){
      detected++;
  }

  //Rotate servo from 90 - 115 degrees
  servo.write(115);
  delay(100);

  //Check if an obstacle is in the way
  if(ultrasonicDistance() <= 12){
      detected++;
  }

  //Rotate servo from 115 - 65 degrees
  servo.write(65);
  delay(100);

  //Check if obstacle is in the way
  if(ultrasonicDistance() <= 12){
      detected++;
  }

  //Rotate servo from 65 back to 90 degrees
  servo.write(90);
  delay(100);

  //If an obstacle has been detected more than once, indicate that there is an obstacle in the path
  bool found = false;

  //Since the ultrasonic occasionally produces odd results, this attempts to filter out false hits
  if(detected > 1){
    found = true;
  }
  return found;
}

//Returns distance from ultrasonic sensor to nearest obstacle in inches
int ultrasonicDistance(){
  //Set ultrasonic to output to activate it
  pinMode(ULTRASONICPIN, OUTPUT);

  //Ensure clean HIGH pulse
  digitalWrite(ULTRASONICPIN, LOW);
  delayMicroseconds(2);

  //Activate ultrasonic with 5us HIGH pulse
  digitalWrite(ULTRASONICPIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(ULTRASONICPIN, LOW);

  //Swap ultrasonic pin mode to "catch" echo
  pinMode(ULTRASONICPIN, INPUT);

  //Convert microseconds returned by pulseIn to inches
  return pulseIn(ULTRASONICPIN, HIGH) / 74 / 2;
}

//At the moment this just checks if the coordinates are equal. For project 2, include photosensor readings.
bool checkForDestination(int destX, int destY, int srcX, int srcY){
  if((destX == srcX) && (destY == srcY)){
    Serial.println("Destination found!");
    return true;
  }
  return false;
}

//The most important function
void dance(){
  leftMotor->setSpeed(120);
  rightMotor->setSpeed(120);
  for(int i = 0; i < 100; i++){
    leftMotor->step(1, FORWARD, INTERLEAVE);
    rightMotor->step(1, FORWARD, INTERLEAVE);
    delayMicroseconds(250);
  }
  leftMotor->step(100, FORWARD, INTERLEAVE);
  leftMotor->step(100, BACKWARD, INTERLEAVE);
  rightMotor->step(100, BACKWARD, INTERLEAVE);
  rightMotor->step(100, FORWARD, INTERLEAVE);
  for(int i = 0; i < 100; i++){
    leftMotor->step(1, BACKWARD, INTERLEAVE);
    rightMotor->step(1, BACKWARD, INTERLEAVE);
    delayMicroseconds(250);
  }
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  doA180();
  leftMotor->setSpeed(100);
  rightMotor->setSpeed(100);
}

