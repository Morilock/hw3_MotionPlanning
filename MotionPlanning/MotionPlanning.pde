import java.util.*; 

PVector GOAL1 = new PVector(900,500);
PVector GOAL2 = new PVector(500,100);

Milestone[] sample_points;
int sample_number = 300;
int[][] has_path = new int[sample_number+2][sample_number+2];
ArrayList<Agent> agents = new ArrayList<Agent>();
ArrayList<Obstacle> obstacles =new  ArrayList<Obstacle>();

float obstacle_radius = 50;
float sample_radius = 7.5;
float agent_radius = 7.5;
boolean move = false;

float check_Distance(PVector p1, PVector p2){
  return PVector.sub(p1, p2).mag();
}

boolean check_Feasibility(PVector sample_point) {
  for(int i = 0; i< obstacles.size(); i ++){
    if(check_Distance(obstacles.get(i).center, sample_point) < (obstacle_radius + sample_radius)) 
      return false;
  }
  return true;
}

boolean check_Path(PVector p1, PVector p2){  
  float a,b,c;
  float i,sol1, sol2;
  PVector dp,center;  
  float r;
  
  for (int n = 0; n < obstacles.size(); n++){
    center = obstacles.get(n).center;
    r = obstacle_radius + sample_radius;
        
    dp = PVector.sub(p2,p1);
    a = dp.x*dp.x + dp.y * dp.y;
    b = 2*(dp.x*(p1.x - center.x) +dp.y*(p1.y - center.y));
    c = PVector.dot(center,center);
    c += PVector.dot(p1,p1);
    c -= 2* PVector.dot(center,p1);
    c -= r*r;
    i = b*b - 4*a*c;
    if(i>=0)  
    sol1 =0;
    sol2 = 0;
      
    if(i == 0){
      sol1 = -b/2/a;
      if(sol1> 0 && sol1 <1) 
        return false;   
    } 
    
    else{
      sol1 = (-b + sqrt(i))/2/a;
      sol2 = (-b - sqrt(i))/2/a;
      if((sol1 >0 && sol1<1) || (sol2>0&&sol2<1)) 
        return false;    
    }
  }
    return true;
}
/*
boolean check_Path(PVector p1, PVector p2){  
  float k,b, dist;
  PVector dp, center;  
  
  for (int n = 0; n < obstacles.size(); n++){
    center = obstacles.get(n).center;
    dp = PVector.sub(p2,p1);
    k = dp.y / dp.x;
    b = p1.y - k*p1.x;
    dist = ((-k)*center.x + center.y - b) / sqrt(k*k + b*b);
    if (dist <= obstacle_radius + sample_radius)
      return false;    
  }
  return true;
}
*/

void setup() {  
  size(1000,1000);  

  for(int i = 0; i < 20; i ++){
    agents.add(new Agent(new PVector(80, 300 + 20*i),agent_radius));
  } 
 for(int i = 0; i < 20; i ++){
    agents.add(new Agent(new PVector(500 + 20*((int)i/4), 850 + 20* ((int)i%4)), agent_radius));
  }
  
  sample_points = new Milestone[sample_number];
  
  for (int i = 0; i < sample_number; i++) {
    PVector one_point = new PVector(random(20,980), random(20,980));    
    while (check_Feasibility(one_point) == false) {
      one_point = new PVector(random(20,980), random(20,980));
    }    
    sample_points[i] = new  Milestone(one_point);
  }
  
  for (int i = 0; i < sample_number; i++) {
    for (int j = 0; j < sample_number; j++) {
       has_path[i][j] = 0;
    }
  }  
  
  for (int i = 0; i < sample_number; i++) {
    for (int j = 0; j < sample_number; j++) {      
      if (check_Distance(sample_points[i].pos, sample_points[j].pos) > 15) {                
         if (check_Path(sample_points[i].pos, sample_points[j].pos)) {           
           sample_points[i].neighbors.add(sample_points[j]);
           has_path[i][j] = 1;        
        }
      }      
    }
  }
  
  for(int i = 0; i < 20; i++){
    if(!agents.get(i).reachGoal){ 
      agents.get(i).searchPath(agents.get(i).pos,GOAL1);      
    }
  }

  for(int i = 20; i < agents.size(); i++){
    if(!agents.get(i).reachGoal){ 
      agents.get(i).searchPath(agents.get(i).pos,GOAL2);
    }
  }
    
}

void draw() {  
  fill(255,255,255);
  rect(0,0,1000,1000);
  
  for(int i = 0; i< obstacles.size(); i ++){
    fill(0,255,0);
    circle(obstacles.get(i).center.x,obstacles.get(i).center.y, obstacles.get(i).r * 2);    
  }

  for (int i = 0; i < sample_number; i++) {
    fill(0,0,0);
    circle(sample_points[i].pos.x, sample_points[i].pos.y, 15);
  }
    
  fill(200,200,20);
  stroke(200,200,20);  

  for(int i = 0; i < agents.size(); i ++){
      if(move) {
        agents.get(i).compute_vel_change();  
      }
  }
   
  for(int i = 0; i < agents.size(); i ++){
      if(move)
      {
        //println("Begin",agents.get(i).this_Path.size());
        agents.get(i).update();
      }
      if (i<20){
        fill(0,0,255);
        stroke(255,0,0);
        circle(agents.get(i).pos.x, agents.get(i).pos.y, 2*agent_radius);
      }
      else {
        fill(150,150,150);
        stroke(255,0,0);
        circle(agents.get(i).pos.x, agents.get(i).pos.y, 2*agent_radius);        
      }
  }
 
  fill(255,255,0);
  circle(agents.get(0).this_Goal.pos.x, agents.get(0).this_Goal.pos.y,30); 
  fill(255,0,0);
  circle(agents.get(20).this_Goal.pos.x, agents.get(20).this_Goal.pos.y,30); 
}

void check_newObstacle(){
    for (int i = 0; i < sample_number; i++) {
      if(!check_Feasibility(sample_points[i].pos)){
        PVector one_point = new PVector(random(20,980), random(20,980));
        while (check_Feasibility(one_point) == false) {
          one_point = new PVector(random(20,980), random(20,980));
        }
        sample_points[i] = new Milestone(one_point);
      }      
      sample_points[i].neighbors = new ArrayList<Milestone>();      
    for (int j = 0; j < sample_number; j++) {      
      if (check_Distance(sample_points[i].pos, sample_points[j].pos) > sample_radius*2) {               
         if (check_Path(sample_points[i].pos, sample_points[j].pos)) {           
           sample_points[i].neighbors.add(sample_points[j]);
           has_path[i][j] = 1;        
        }
      }        
    }
  }  
}

void Agent_replan(){
  for(int i = 0; i < 20; i++){
    if(!agents.get(i).reachGoal)
    { 
      //println("Begin",agents.get(i).this_Path.size());
      agents.get(i).index_setzero();
      agents.get(i).searchPath(agents.get(i).pos,GOAL1);   
    }
  }

  for(int i = 20; i < agents.size(); i++){
    if(!agents.get(i).reachGoal)
    { 
      agents.get(i).index_setzero();
      agents.get(i).searchPath(agents.get(i).pos,GOAL2);
    }
  }
}

void Search_again(){
  for(int i = 0; i < agents.size(); i++){
    if(!agents.get(i).reachGoal && agents.get(i).vel.mag() < 0.5){
      agents.get(i).searchPath(agents.get(i).pos,agents.get(i).this_Goal.pos);
    }
  }
}

void keyPressed(){
  if (key == 'w'){
    obstacles.get(0).move(0,-20);
    check_newObstacle();
  }
  else if (key == 's'){
    obstacles.get(0).move(0,20);
    check_newObstacle();
  }
  else if (key == 'a'){
    obstacles.get(0).move(-20,0);
    check_newObstacle();
  }
  else if (key == 'd'){
    obstacles.get(0).move(20,0);
    check_newObstacle();
  }    
  else if(keyCode == ENTER){
    move = !move;
  }
  else if (keyCode == UP){
    for(int i = 0; i < 20; i ++){
      agents.get(i).set_Goal(PVector.add(new PVector(0,-10),agents.get(i).this_Goal.pos));
    }
  }
  else if (keyCode == DOWN){
    for(int i = 0; i < 20; i ++){
      agents.get(i).set_Goal(PVector.add(new PVector(0,10),agents.get(i).this_Goal.pos));
    }
  }
  else if (keyCode == LEFT){
    for(int i = 0; i < 20; i ++){
      agents.get(i).set_Goal(PVector.add(new PVector(-10,0),agents.get(i).this_Goal.pos));
    }
  }
  else if (keyCode == RIGHT){
    for(int i = 0; i < 20; i ++){
      agents.get(i).set_Goal(PVector.add(new PVector(10,0),agents.get(i).this_Goal.pos));
    }
  }
  Search_again();  
}



void mousePressed(){
  if (mouseButton == LEFT){
    obstacles.add(new Obstacle(40, mouseX, mouseY));
    check_newObstacle();
    //Agent_replan();
  }
  if (mouseButton == RIGHT){
    agents.add(new Agent(new PVector(mouseX, mouseY),agent_radius));
    agents.get(agents.size()-1).searchPath(agents.get(agents.size()-1).pos,GOAL2);
  }
  
  Search_again();  
  
}
