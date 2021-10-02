//Agents
static int maxNumAgents = 10;
int numAgents = 0;

float agentRad = 15;
float goalSpeed = 500;

//The agent states
Vec2[] agentPos = new Vec2[maxNumAgents];
Vec2[] agentVel = new Vec2[maxNumAgents];
int[] lastNode = new int[maxNumAgents];
int[] nextNode = new int[maxNumAgents];

//The agent starts and goals
Vec2[] startPos = new Vec2[maxNumAgents];
Vec2[] goalPos = new Vec2[maxNumAgents];

//The agents' pathes
ArrayList<ArrayList<Integer>> curPath = 
                  new ArrayList<ArrayList<Integer>>(maxNumAgents);

void setup() {
  size(1024,768);
  PRM();
  //set agent's position to the start positions
  for(int i = 0; i < numAgents; i++) {
      agentPos[i] = new Vec2(startPos[i].x, startPos[i].y);
      if (curPath.get(i).size() >0 && curPath.get(i).get(0) == -1){}
      else{
        lastNode[i] = curPath.get(i).get(0);
        nextNode[i] = curPath.get(i).get(1);
      }
  }
}

void moveAgent(float dt){
  //Update position and velocity using (Eulerian) numerical integration
  
  for (int i = 0; i < numAgents; i++){
    if(startPos[i] != null && goalPos[i] != null) {
     if (curPath.get(i).size() >0 && curPath.get(i).get(0) == -1){} //no path: don't update position and velocity
     else{
        if(agentPos[i].x == startPos[i].x && agentPos[i].y == startPos[i].y) {  //gives the agent an initial velocity for them to start
             agentVel[i] = nodePos[curPath.get(i).get(1)].minus(nodePos[curPath.get(i).get(0)]).normalized();
             if (agentVel[i].length() > 0) 
                  agentVel[i].setToLength(goalSpeed);
             agentPos[i].add(agentVel[i].times(dt));
        }
        else if(agentPos[i].distanceTo(startPos[i]) >= goalPos[i].distanceTo(startPos[i])) { //for agents that are close to the goal, fix their position and stop updating their positions
           agentPos[i] = new Vec2(goalPos[i].x, goalPos[i].y);
        }
        else{  //for agents that are on their way
           float dis = agentPos[i].distanceTo(nodePos[lastNode[i]]);
           float segLength = nodePos[nextNode[i]].distanceTo(nodePos[lastNode[i]]);
           for(int j = 0; j < curPath.get(i).size()-2; j++){
              if(lastNode[i] == curPath.get(i).get(j) && dis <= segLength) {
                  agentVel[i] = nodePos[nextNode[i]].minus(nodePos[lastNode[i]]).normalized();
                   if (agentVel[i].length() > 0) 
                     agentVel[i].setToLength(goalSpeed);
                  break;
              }
              else if(lastNode[i] == curPath.get(i).get(j) && dis > segLength){
                  agentPos[i] = new Vec2(nodePos[nextNode[i]].x,nodePos[nextNode[i]].y);
                  lastNode[i] = curPath.get(i).get(j+1);
                  nextNode[i] = curPath.get(i).get(j+2);
                  agentVel[i] = nodePos[nextNode[i]].minus(nodePos[lastNode[i]]).normalized();
                  if (agentVel[i].length() > 0) 
                     agentVel[i].setToLength(goalSpeed);
                  break;
              }
           }
           agentPos[i].add(agentVel[i].times(dt));
        }
     }
  }
  }
}

//Obstacles and Nodes
int numObstacles = 1;
int numNodes  = 100;
boolean moveObstacle = false;
//A list of circle obstacles
static int maxNumObstacles = 1000;
Vec2 circlePos[] = new Vec2[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii
float configureRad[] =  new float[maxNumObstacles]; //configuration space for every obstacle
color colors[] = new color[maxNumObstacles];
static int maxNumNodes = 1000;
Vec2[] nodePos = new Vec2[maxNumNodes];

//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii){
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    while (insideAnyCircle){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    }
    nodePos[i] = randPos;
  }
}

void placeRandomObstacles(int numObstacles){
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++){
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = (10+40*pow(random(1),3));
    colors[i] = color(255);
  }
  circleRad[0] = 30; //Make the first obstacle big
}

int strokeWidth = 2;

Vec2 sampleFreePos(){
  Vec2 randPos = new Vec2(random(width),random(height));
  boolean insideAnyCircle = pointInCircleList(circlePos,configureRad,numObstacles,randPos,2);
  while (insideAnyCircle){
    randPos = new Vec2(random(width),random(height));
    insideAnyCircle = pointInCircleList(circlePos,configureRad,numObstacles,randPos,2);
  }
  return randPos;
}

void PRM(){
  placeRandomObstacles(numObstacles);
  for(int i = 0; i < numObstacles; i++) {
     configureRad[i] = circleRad[i] + agentRad*1.08;
  }
  for(int i = 0; i < numAgents; i++) {
     startPos[i] = sampleFreePos();
     
     goalPos[i] = sampleFreePos();
  }
  
  generateRandomNodes(numNodes, circlePos, configureRad);
  connectNeighbors(circlePos, configureRad, numObstacles, nodePos, numNodes);
  
  for(int i = 0; i < numAgents; i++) {
     ArrayList<Integer> temp = planPath(startPos[i], goalPos[i], circlePos, configureRad, numObstacles, nodePos, numNodes);
     curPath.add(temp);
  }
}

boolean paused = true;
boolean changeColor = false;
void draw(){
  //System.out.println("nodeNum: " + numNodes);
  //println("FrameRate:",frameRate);
  strokeWeight(1);
  background(200); //Grey background
  stroke(0,0,0);
  fill(255,255,255);
  
  //update agent
  if(!paused) {
    moveAgent(1.0/(frameRate*10));
  }
  
  if(moveObstacle) {
    for(int j = 0; j < numAgents; j++) {
          startPos[j] = new Vec2(agentPos[j].x, agentPos[j].y);
     }
      generateRandomNodes(numNodes, circlePos, configureRad);
       connectNeighbors(circlePos, configureRad, numObstacles, nodePos, numNodes);
       for(int i= 0; i < numAgents; i++) {
           ArrayList<Integer> temp = planPath(startPos[i], goalPos[i], circlePos, configureRad, numObstacles, nodePos, numNodes);
           curPath.set(i, temp);
           if (curPath.get(i).size() >0 && curPath.get(i).get(0) == -1){}
           else{
             lastNode[i] = curPath.get(i).get(0);
            nextNode[i] = curPath.get(i).get(1);
           }
         }
         moveObstacle = false;
        
       
     }
  
  
  
  //Draw the circle obstacles
  Vec2 mousePos = new Vec2(mouseX, mouseY);
  for (int i = 0; i < numObstacles; i++){
    
    if(pointInCircle(circlePos[i], circleRad[i], mousePos, 1)){
      stroke(255,0,0);
      if(changeColor) {
         colors[i] = color(random(255), random(255),random(255));
         changeColor = false;
      }
    }
     
    Vec2 c = circlePos[i];
    float r = circleRad[i];
    fill(colors[i]);
    circle(c.x,c.y,r*2);
    stroke(0);
    
  }
  //Draw the first circle a little special b/c the user controls it
  fill(240);
  strokeWeight(2);
  //circle(circlePos[0].x,circlePos[0].y,circleRad[0]*2);
  strokeWeight(1);
  
  //Draw PRM Nodes
  fill(0);
  for (int i = 0; i < numNodes; i++){
    circle(nodePos[i].x,nodePos[i].y,5);
  }
  
  //Draw graph
  stroke(100,100,100);
  strokeWeight(1);
  for (int i = 0; i < numNodes; i++){
    for (int j : neighbors[i]){
      line(nodePos[i].x,nodePos[i].y,nodePos[j].x,nodePos[j].y);
    }
  }
  //Draw Agent
  fill(0,0,255);
  for (int i = 0; i < numAgents; i++){
    if(pointInCircle(agentPos[i], agentRad, mousePos, 1)){
      fill(255,0,0);
    }
    circle(agentPos[i].x, agentPos[i].y, agentRad*2);
    fill(0,0,255);
  }
  
  //Draw Start and Goal
  fill(20,60,250);
  for (int i = 0; i < numAgents; i++){
     circle(startPos[i].x,startPos[i].y,20);
  }
  
  fill(250,30,50);
  for (int i = 0; i < numAgents; i++){
    if(goalPos[i] != null) {
     circle(goalPos[i].x,goalPos[i].y,20);
    }
  }
 
  
  
  //Draw Planned Path
  stroke(20,255,40);
  strokeWeight(5);
 
  for (int i = 0; i < numAgents; i++){
     //System.out.println(numAgents);
    if(startPos[i] != null && goalPos[i] != null) {
     if (curPath.get(i).size() == 0){
      line(startPos[i].x,startPos[i].y,goalPos[i].x,goalPos[i].y);
     }
    }
  }
  
 for(int j = 0; j < numAgents; j++) {
    if(startPos[j] != null && goalPos[j] != null) {
     if (curPath.get(j).size() >0 && curPath.get(j).get(0) == -1){}
     else{
       stroke(255-j*50, j*60 + 10,40);
     line(startPos[j].x,startPos[j].y,nodePos[curPath.get(j).get(0)].x,nodePos[curPath.get(j).get(0)].y);
     for (int i = 0; i < curPath.get(j).size()-1; i++){
      int curNode = curPath.get(j).get(i);
      int nextNode = curPath.get(j).get(i+1);
      line(nodePos[curNode].x,nodePos[curNode].y,nodePos[nextNode].x,nodePos[nextNode].y);
     }
     line(goalPos[j].x,goalPos[j].y,nodePos[curPath.get(j).get(curPath.get(j).size()-1)].x,nodePos[curPath.get(j).get(curPath.get(j).size()-1)].y);
     }
     }
 }
}

void keyPressed(){
   System.out.println(key);
  if (key == ' ') paused = !paused;
  if(keyCode == 8){
     boolean deleteObstacle = false;
    
     for(int i = 0; i < numObstacles; i++) {
       if(pointInCircle(circlePos[i], circleRad[i],new Vec2(mouseX, mouseY) , 1)) {
         
         for(int j = i+1; j < numObstacles; j++) {
             circlePos[j-1] = circlePos[j];
             circleRad[j-1] = circleRad[j];
             configureRad[j-1] =  configureRad[j];
             colors[j-1] = colors[j];
         }
         deleteObstacle = true;
         break;
       }
     }
     for(int i = 0; i < numAgents; i++) {
         if(pointInCircle(agentPos[i], agentRad,new Vec2(mouseX, mouseY) , 1)) {
         
         for(int j = i+1; j < numAgents; j++) {
             agentPos[j-1] = agentPos[j];
             agentVel[j-1] = agentVel[j];
             startPos[j-1] = startPos[j];
             goalPos[j-1] = goalPos[j];
             lastNode[j-1] = lastNode[j];
             nextNode[j-1] = nextNode[j];
             
             
         }
         startPos[numAgents-1] = null;
         goalPos[numAgents-1] = null;
         curPath.remove(i);
         numAgents --;
         break;
       }
     }
     if(deleteObstacle) {
       numObstacles--;
       for(int i = 0; i < numAgents; i++) {
          startPos[i] = new Vec2(agentPos[i].x, agentPos[i].y);
       }
       generateRandomNodes(numNodes, circlePos, configureRad);
       connectNeighbors(circlePos, configureRad, numObstacles, nodePos, numNodes);
       for(int i = 0; i < numAgents; i++) {
        ArrayList<Integer> temp = planPath(startPos[i], goalPos[i], circlePos, configureRad, numObstacles, nodePos, numNodes);
        curPath.set(i, temp);
        if (curPath.get(i).size() >0 && curPath.get(i).get(0) == -1){}
        else{
        lastNode[i] = curPath.get(i).get(0);
        nextNode[i] = curPath.get(i).get(1);
        }
       }
     }
    
  
    //System.out.println(numAgents);
  
  }
  
  if(keyCode == UP){
     changeColor = true;
  }
}

int leftClicked = 0;
void mouseClicked(){
  if (mouseButton == RIGHT){   //right click to place obstacle
    if(numObstacles + 1 <= maxNumObstacles) {
       circlePos[numObstacles] = new Vec2(mouseX, mouseY);//Circle positions
       circleRad[numObstacles] = (10+40*pow(random(1),3));  //Circle radii
       configureRad[numObstacles] = circleRad[numObstacles] + agentRad*1.08;
       colors[numObstacles] = color(255);
       numObstacles ++;
    }
    else{
      circlePos[numObstacles-1] = new Vec2(mouseX, mouseY);
      circleRad[numObstacles-1] = (10+40*pow(random(1),3));
      configureRad[numObstacles-1] = circleRad[numObstacles] + agentRad*1.08;
      colors[numObstacles-1] = color(255);
    }
    
    
    for(int i = 0; i < numAgents; i++) {
     startPos[i] = new Vec2(agentPos[i].x, agentPos[i].y);
    }
    generateRandomNodes(numNodes, circlePos, configureRad);
    connectNeighbors(circlePos, configureRad, numObstacles, nodePos, numNodes);
    for(int i = 0; i < numAgents; i++) {
      ArrayList<Integer> temp = planPath(startPos[i], goalPos[i], circlePos, configureRad, numObstacles, nodePos, numNodes);
      curPath.set(i, temp);
      if (curPath.get(i).size() >0 && curPath.get(i).get(0) == -1){}
      else{
      lastNode[i] = curPath.get(i).get(0);
      nextNode[i] = curPath.get(i).get(1);
      }
    }
  }
  else if (mouseButton == LEFT){  //left click to place agent
    leftClicked ++;
    if(leftClicked % 2 != 0) {
      System.out.println("odd");
     if(numAgents + 1 <= maxNumAgents) {
       agentPos[numAgents] = new Vec2(mouseX, mouseY);//agent's positions
       startPos[numAgents] = new Vec2(mouseX, mouseY);//update its startPos
       numAgents++;
     }
     else{
      System.out.println("Can't have more agents!!!");
     }
    }
    else{
      System.out.println("even");
       goalPos[numAgents-1] = new Vec2(mouseX, mouseY);
       ArrayList<Integer> temp = planPath(startPos[numAgents-1], goalPos[numAgents-1], circlePos, configureRad, numObstacles, nodePos, numNodes);
       curPath.add(temp);
       if (curPath.get(numAgents-1).size() >0 && curPath.get(numAgents-1).get(0) == -1){}
       else{
        lastNode[numAgents-1] = curPath.get(numAgents-1).get(0);
        nextNode[numAgents-1] = curPath.get(numAgents-1).get(1);
       }
       System.out.println(numAgents + " curPath");
       
    }
  }
  
}

void mouseDragged() 
{
  
  for(int i = 0; i < numObstacles; i++) {
       if(pointInCircle(circlePos[i], circleRad[i],new Vec2(mouseX, mouseY) , 1)) {
         
         circlePos[i] = new Vec2(mouseX, mouseY);
         moveObstacle = true;
         break;
       }
   }
  for(int i = 0; i < numAgents; i++) {
       if(pointInCircle(agentPos[i], agentRad,new Vec2(mouseX, mouseY) , 1)) {
             agentPos[i] = new Vec2(mouseX, mouseY);
             startPos[i] = new Vec2(mouseX, mouseY);
             ArrayList<Integer> temp = planPath(startPos[i], goalPos[i], circlePos, configureRad, numObstacles, nodePos, numNodes);
             curPath.set(i, temp);
            if (curPath.get(i).size() >0 && curPath.get(numAgents-1).get(0) == -1){}
            else{
              lastNode[i] = curPath.get(i).get(0);
              nextNode[i] = curPath.get(i).get(1);
            }
             break;
        }
   }
 }
    
  
   
  
  
