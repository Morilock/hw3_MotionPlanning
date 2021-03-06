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
  float delta,sol1, sol2;
  PVector dp,center;  
  float r;        
  for (int n = 0; n < obstacles.size(); n++){
    center = obstacles.get(n).center;
    r = obstacle_radius + agent_radius;
        
    dp = PVector.sub(p2,p1);
    a = dp.x*dp.x + dp.y * dp.y;
    b = 2*(dp.x*(p1.x - center.x) +dp.y*(p1.y - center.y));
    c = PVector.dot(center,center)+ PVector.dot(p1,p1) - 2* PVector.dot(center,p1) - r*r;
    delta = b*b - 4*a*c;  
    sol1 = 0;
    sol2 = 0;    
    if(delta == 0){
      sol1 = -b/2/a;
      if(sol1> 0 && sol1 <1) 
        return false;
    }  
    else{
      sol1 = (-b + sqrt(delta))/2/a;
      sol2 = (-b - sqrt(delta))/2/a;
      if((sol1 >0 && sol1<1) || (sol2>0 && sol2<1)) 
        return false;
    }
  }
  return true;
}
/*
boolean check_Path(PVector p1, PVector p2){  
  float k,b, dist;
  PVector dp, center;  
  PVector np1 = new PVector(p1.x, -p1.y);
  PVector np2 = new PVector(p2.x, -p2.y);   
  
  for (int n = 0; n < obstacles.size(); n++){
    center = obstacles.get(n).center;
    PVector nc = new PVector(center.x, -center.y);
    dp = PVector.sub(np2,np1);
    k = dp.y / dp.x;
    b = np1.y - k*np1.x;
    dist = abs(k*nc.x - nc.y + b) / sqrt(k*k + 1);
    if (dist <= obstacle_radius + sample_radius)
      return false;    
  }
  return true;
}
*/

void setup() {  
  size(1000,1000, P3D);  

  for(int i = 0; i < 10; i ++){
    agents.add(new Agent(new PVector(80, 300 + 20*i),agent_radius));
  } 
 for(int i = 0; i < 10; i ++){
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
  
  for(int i = 0; i < 10; i++){
    if(!agents.get(i).reachGoal){ 
      agents.get(i).searchPath(agents.get(i).pos,GOAL1);      
    }
  }

  for(int i = 10; i < agents.size(); i++){
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
    drawCylinderOBS(int(obstacles.get(i).center.x), int(obstacles.get(i).center.y), obstacles.get(i).r, 80);  
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
      if (i<10){
        fill(0,0,255);
        stroke(255,0,0);
        drawCylinderA1(int(agents.get(i).pos.x), int(agents.get(i).pos.y), agent_radius, 20);
      }
      else {
        fill(150,150,150);
        stroke(255,0,0);
        drawCylinderA2(int(agents.get(i).pos.x), int(agents.get(i).pos.y), agent_radius, 20);        
      }
  }
 
  fill(255,255,0);
  circle(agents.get(0).this_Goal.pos.x, agents.get(0).this_Goal.pos.y,30); 
  fill(255,0,0);
  circle(agents.get(10).this_Goal.pos.x, agents.get(10).this_Goal.pos.y,30); 
  //Search_again();
}

void drawCylinderOBS(int center_x, int center_y, float r, int h)
{
    float angle = 1;
    beginShape();
    fill(255,0,0);
    for (int i = 0; i < 360; i++) {
        float x = center_x + cos( radians( i * angle ) ) * r;
        float y = center_y + sin( radians( i * angle ) ) * r;
        vertex(x, y, 0);    
    }
    endShape(CLOSE);

    beginShape();
    fill(0,255,0);
    for (int i = 0; i < 360; i++) {
        float x = center_x + cos(radians( i * angle ) ) * r;
        float y = center_y + sin(radians( i * angle ) ) * r;
        vertex(x, y, h);    
    }
    endShape(CLOSE);

    beginShape(TRIANGLE_STRIP);
    fill(0,0,255);
    for (int i = 0; i < 360; i++) {
      float x = center_x + cos(radians( i * angle ) ) * r;
      float y = center_y + sin(radians( i * angle ) ) * r;
      vertex(x, y, h);
      vertex(x, y, 0);    
    }
    endShape(CLOSE);
}

void drawCylinderA1(int center_x, int center_y, float r, int h)
{
    float angle = 6;
    beginShape();
    fill(0,255,0);
    for (int i = 0; i < 60; i++) {
        float x = center_x + cos( radians( i * angle ) ) * r;
        float y = center_y + sin( radians( i * angle ) ) * r;
        vertex(x, y, 0);    
    }
    endShape(CLOSE);

    beginShape();
    fill(0,0,255);
    for (int i = 0; i < 60; i++) {
        float x = center_x + cos(radians( i * angle ) ) * r;
        float y = center_y + sin(radians( i * angle ) ) * r;
        vertex(x, y, h);    
    }
    endShape(CLOSE);

    beginShape(TRIANGLE_STRIP);
    fill(255,0,0);
    for (int i = 0; i < 60; i++) {
      float x = center_x + cos(radians( i * angle ) ) * r;
      float y = center_y + sin(radians( i * angle ) ) * r;
      vertex(x, y, h);
      vertex(x, y, 0);    
    }
    endShape(CLOSE);
}

void drawCylinderA2(int center_x, int center_y, float r, int h)
{
    float angle = 6;
    beginShape();
    fill(150,150,150);
    for (int i = 0; i < 60; i++) {
        float x = center_x + cos( radians( i * angle ) ) * r;
        float y = center_y + sin( radians( i * angle ) ) * r;
        vertex(x, y, 0);    
    }
    endShape(CLOSE);

    beginShape();
    fill(150,150,150);
    for (int i = 0; i < 60; i++) {
        float x = center_x + cos(radians( i * angle ) ) * r;
        float y = center_y + sin(radians( i * angle ) ) * r;
        vertex(x, y, h);    
    }
    endShape(CLOSE);

    beginShape(TRIANGLE_STRIP);
    fill(0,255,0);
    for (int i = 0; i < 60; i++) {
      float x = center_x + cos(radians( i * angle ) ) * r;
      float y = center_y + sin(radians( i * angle ) ) * r;
      vertex(x, y, h);
      vertex(x, y, 0);    
    }
    endShape(CLOSE);
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
    if(!agents.get(i).reachGoal && agents.get(i).vel.mag() < 0.01){
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
