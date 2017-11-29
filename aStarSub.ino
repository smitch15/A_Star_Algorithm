#define OBSTACLE_SIZE 3

class Node{
 public:
  byte parentXY;
  byte hCost;
  byte gCost;
  byte infoBits = 0;
};


void setup() {
  Serial.begin(9600);
  Serial.print('a');
  // obstacle X's and Y's 
  const PROGMEM uint8_t obstacleX[] = {};
  const PROGMEM uint8_t obstacleY[] = {};
  const PROGMEM uint8_t srcXandY[] = {0,0};
  const PROGMEM uint8_t destXandY[] = {1,1};

  Node graph[16][16];
  Serial.print('b');
  // set all g and h costs of each node in graph to infinity(255)
  for (byte i = 0; i < 16; i++){
    for (byte j = 0; j < 16; j++){
        graph[i][j].gCost = 255;
        graph[i][j].hCost = 255;
    }
  }
  Serial.print('c');
  
  // set start node g cost to zero
  graph[srcXandY[0]][srcXandY[1]].gCost = 0;
  // set heuristic value of start node
  graph[srcXandY[0]][srcXandY[1]].hCost = abs((srcXandY[0] - destXandY[0])) + abs((srcXandY[1] - destXandY[1]));
  
  // put the startnode in the queue, set the source node bit
  graph[srcXandY[0]][srcXandY[1]].infoBits |= 0b10001010; 
    
  // set the info bit for known node obstacles
  for (byte i = 0; i < OBSTACLE_SIZE; i++){
    (graph[obstacleX[i]][obstacleY[i]]).infoBits |= 0b100;
  }

  // set the destination node bits
  graph[destXandY[0]][destXandY[1]].infoBits |= 0b1001;

  // set the in queue bit for the start node
  graph[srcXandY[0]][srcXandY[1]].infoBits |= 0b10000000;
  
  // start A star algorithm
  // setup
  bool queueEmpty = true;
  for (byte i = 0; i < 16; i++){
    for (byte j = 0; j < 16; j++){
      if (graph[i][j].infoBits & 0b10000000){
        queueEmpty = false;
      }
    }
  }
  Serial.print('d');
  // algorithm processing
  while (!queueEmpty){
    Serial.print('e');
    uint16_t minScore = 65535;

    byte xMin = 17;
    byte yMin = 17;

    for (byte i = 0; i < 16; i++){
      for (byte j = 0; j < 16; j++){
        if (graph[i][j].hCost + graph[i][j].gCost < minScore){
          xMin = i;
          yMin = j;
        }
      }
    }
        
    while (xMin == 17 || yMin == 17){
      Serial.println("x and y min not initialized");  
    }

    Node current = graph[xMin][yMin];
    if (current.infoBits |= 0b1){ // if current is destination
      //TODO:
      // reconstruct_path function implementation
      byte sourceXY = (srcXandY[0] << 4) & srcXandY[1];
      byte num = 0;
      while (current.parentXY != sourceXY){
        Serial.println("curent's parent: ");
        Serial.println(current.parentXY);
        Serial.println("source xy: ");
        Serial.println(sourceXY);
        current.infoBits &= 0b1000;
        current = graph[current.parentXY >> 4][current.parentXY & 0b1111];
        Serial.println("node num "); Serial.println(num);
        Serial.println("path x, y"); Serial.println(current.parentXY >> 4); Serial.println(current.parentXY & 0b1111);
        num++;
        
      }
      return;
    }
    
    // remove min score from queue
    graph[xMin][yMin].infoBits &= 0b01111111;  //remove from queue bit
    graph[xMin][yMin].infoBits |= 0b01000000;  //add to visited 

    // check if neighbor is new, then add it to the open queue 
    // not in the queue and not been visited
    Node *rightNeb, *leftNeb, *topNeb, *botNeb;
    rightNeb = &graph[xMin+1][yMin];
    leftNeb = &graph[xMin-1][yMin];
    topNeb = &graph[xMin][yMin-1];
    botNeb = &graph[xMin][yMin+1];


    // check right neighbor 
    if (!(rightNeb->infoBits & 0b10000000) && !(rightNeb->infoBits & 0b01000000)){ 
      rightNeb->infoBits |= 0b10001010; // add to queue
      // update the gCosts of the neighbors
      // check right one
      if ((current.gCost + 1) < rightNeb->gCost){
        rightNeb->gCost = current.gCost + 1;
        rightNeb->parentXY = current.parentXY+ 0b10000;
      }
    }
    // check left neighbor 
    if (!(leftNeb->infoBits & 0b10000000) && !(leftNeb->infoBits & 0b01000000)) { 
      leftNeb->infoBits |= 0b10001010; 
      // check left one
      if ((current.gCost + 2) < leftNeb->gCost){
        leftNeb->gCost = current.gCost + 2;
        leftNeb->parentXY = current.parentXY - 0b10000;

      }
    }
    // check bottom neighbor 
    if (!(botNeb->infoBits & 0b10000000) && !(botNeb->infoBits & 0b01000000)) { 
      botNeb->infoBits |= 0b10001010; 
      // check bottom one
      if ((current.gCost + 2) < botNeb->gCost){
        botNeb->gCost = current.gCost + 2;
        botNeb->parentXY = current.parentXY + 0b1;
      }
    }
    // check top neighbor 
    if (!(topNeb->infoBits & 0b10000000) && !(topNeb->infoBits & 0b01000000)) { 
      topNeb->infoBits |= 0b10001010; 
      // check top one
      if ((current.gCost + 3) < topNeb->gCost){
        topNeb->gCost = current.gCost + 3;
        topNeb->parentXY = current.parentXY - 0b1;
      }
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



void loop(){}

