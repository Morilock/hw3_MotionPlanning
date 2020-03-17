import java.util.*; 

Character Character;
boolean move = false;
boolean find = false;

int character_x = 0;
int character_y = 0;
int grid_size = 30;

PVector start;
PVector goal;

Milestone GOAL;
Milestone START;

PVector obstacle_center;
int sample_number = 80;
int obstacle_radius = 2 * grid_size;
int character_radius = 15;
Milestone[] sample_points;
int[][] paths_status = new int[sample_number+2][sample_number+2];
PriorityQueue<Milestone> fringe = new PriorityQueue<Milestone>(new Compare());
ArrayList<Milestone> path = new ArrayList<Milestone>();

float vel = 3;
int idx = 0;

class Character{
  PVector pos;
  PVector vel;
  float speed = 3;
  
  Character(PVector position){
    pos = new PVector(position.x, position.y);
    vel = new PVector(0,0);
  }
  
  void update(){
    pos.add(vel);
    if(check_Distance(path.get(idx).pos,pos) < 3){
      if(path.get(idx).isGoal) Character.Set_Velocity(new PVector(0,0,0)) ;
      else{
        Character.Set_Velocity(PVector.sub(path.get(idx+1).pos,path.get(idx).pos).normalize());
        idx ++;
      }
    }
  }
  
  void Set_Velocity(PVector dir){
    vel = PVector.mult(dir,speed);
  }
}

class Milestone{  
  boolean isGoal = false;
  boolean isVisited = false;
  float g;
  float h;
  float f;
  PVector pos;  
  ArrayList<Milestone> neighbors = new ArrayList<Milestone>();
  Milestone parent;
  
  Milestone(PVector position){
    pos = position;
    g =  PVector.sub(pos,start).mag();
    h = PVector.sub(pos,goal).mag();  
    f = g + h;
  }
}

class Compare implements Comparator<Milestone>{
    public int compare(Milestone m1, Milestone m2) {
        float f1 = m1.f;
        float f2 = m2.f;
        return Float.compare(f1,f2);
    }  
}

float check_Distance(PVector vector_1, PVector vector_2) {
  float x_distance = (vector_1.x - vector_2.x) * (vector_1.x - vector_2.x);
  float y_distance = (vector_1.y - vector_2.y) * (vector_1.y - vector_2.y);
  float distance = sqrt(x_distance + y_distance);
  return distance;
}

boolean check_Feasibility(PVector sample_point) {
  boolean result = false;
  float distance = check_Distance(obstacle_center, sample_point);
  if (distance > (obstacle_radius + character_radius)) { 
    result = true;
  } 
  return result;
}

 
boolean check_Path(PVector point_1, PVector point_2) {
  boolean result = true;

  PVector check_point = new PVector(point_1.x, point_1.y);
  float x_distance = point_2.x - point_1.x;
  float y_distance = point_2.y - point_1.y;  
  float step_num = max(abs(x_distance), abs(y_distance));
  float x_step_size = x_distance / step_num;
  float y_step_size = y_distance / step_num;
  for (int i = 1; i < step_num; i++) {
    check_point.x += x_step_size;
    check_point.y += y_step_size;
    if (check_Feasibility(check_point) == false) {
      result = false;
      break;
    }    
  }  
  return result;
}


boolean A_Star(){  
  START.isVisited = true;
  fringe.add(START);
  int iter = 0; 
  while (fringe.size() >0){
    iter++;    
    Milestone curr = fringe.poll();
    if(curr.isGoal) {
      return true;
    }
    int num_neighbors = curr.neighbors.size();
    for(int i = 0; i < num_neighbors; i++){
      Milestone child = curr.neighbors.get(i);
      if(!child.isVisited){
        child.parent = curr;
        child.isVisited = true;
        fringe.add(child);
      }
    }
  }  
  return true;
}

void setup() {  
  size(700,700);
  character_x = 50 + grid_size;
  character_y = 50 + 19 * grid_size;
  start = new PVector(character_x, character_y);
  goal = new PVector(600,100);
  obstacle_center = new PVector(width/2, height/2);  
  sample_points = new Milestone[sample_number+2];
  
  GOAL = new  Milestone(goal);
  START = new  Milestone(start);
  GOAL.isGoal = true;
  
  sample_points[0] = START;
  sample_points[1] = GOAL;

  PVector sample_point;
  for (int i = 0; i < sample_number+2; i++) {
    for (int j = 0; j < sample_number+2; j++) {
       paths_status[i][j] = 0;
    }
  }
  
  for (int i = 2; i < sample_number+2; i++) {
    sample_point = new PVector(random(55,595), random(55,595));    
    while (check_Feasibility(sample_point) == false) {
      sample_point = new PVector(random(55,595), random(55,595));
    }    
    sample_points[i] = new  Milestone(sample_point);     
  }
  
  for (int i = 0; i < sample_number + 2; i++) {
    for (int j = 0; j < sample_number + 2; j++) {      
      if (check_Distance(sample_points[i].pos, sample_points[j].pos) > 10) {
         if (check_Path(sample_points[i].pos, sample_points[j].pos) == true) {          
           sample_points[i].neighbors.add(sample_points[j]);
           paths_status[i][j] = 1;        
        }
      }      
    }
  }    
  
  A_Star();  
  Milestone curr = GOAL;
  while(curr.parent!=null){
     path.add(curr);
     curr = curr.parent;
  }
  path.add(curr);
  Collections.reverse(path); 
  Character = new Character(start);
  Character.Set_Velocity(PVector.sub(path.get(idx+1).pos,path.get(idx).pos).normalize());
  idx++;
}

void draw() {
  fill(255,255,255);
  rect(0,0,700,700);   
  fill(0,255,0);
  circle(width/2, height/2, obstacle_radius);  
  for (int i = 2; i < sample_number+2; i++) {
    fill(0,0,0);
    circle(sample_points[i].pos.x, sample_points[i].pos.y, 15);
  }   
  fill(255,0,0);
  circle(600, 100, 15);   
  fill(0,0,255);
  circle(character_x, character_y, 0.5 * grid_size); 
  noStroke();  
  if(find){
    find_road();
  }
}

void find_road(){
  fill(200,200,0);
  stroke(100,100,100); 
  Milestone curr = GOAL;
  Milestone prev = GOAL; 
  while(curr.parent!=null){
     circle(curr.pos.x, curr.pos.y, 15);
     prev = curr;
     curr = curr.parent;
     line(prev.pos.x, prev.pos.y, curr.pos.x,curr.pos.y);
  }
  fill(0,0,255);
  if(move){
    Character.update();
    circle(Character.pos.x, Character.pos.y, 15);
  }
}

void keyPressed(){
  if(keyCode == ENTER){
    find = !find;
  }  
  if(keyCode == SHIFT){
    move = !move;
  }
}
