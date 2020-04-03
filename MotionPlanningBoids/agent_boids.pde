class Agent{
  PVector pos;
  PVector goal_vel;
  Milestone this_Goal;
  Milestone this_Start;
  ArrayList<Milestone> this_Path = new ArrayList<Milestone>();
  int index = 0;
  PVector vel = new PVector(0,0);
  PVector vc = new PVector(0,0);
  
  float inter_Radius = 80;
  boolean reachGoal = false;
  float next_vel = 0;
  //float startTime = 0;   
  float dt = 0.5;
  //float r;

  
  Agent(PVector position){
    pos = new PVector(position.x, position.y);
    goal_vel = new PVector(0,0);
  }
  
  void index_setzero(){
    index = 0;
  }
  
  void update(){
    //println("Begin",this_Path.size());
    if(this_Path.size() <1)
      return;    
    if(!reachGoal && PVector.sub(this_Goal.pos, pos).mag() < 22.5) {
        searchPath(pos,this_Goal.pos);
        return;
     }
    float dist_F = check_Distance(this_Path.get(index).pos,pos);
    //println(dist_F);
    if(dist_F < 2*sample_radius && index < this_Path.size()-1){
        next_node_vel(PVector.sub(this_Path.get(index+1).pos,this_Path.get(index).pos));
        index ++;
    }
    
    goal_vel = PVector.sub(this_Path.get(index).pos,pos).normalize().mult(5).sub(vel);
    if(vc.mag() > 0)  
      vel.add(PVector.mult(vc,dt));
    vel.add(PVector.mult(goal_vel,dt));    
    vel.limit(5);
    pos.add(PVector.mult(vel,dt));
    vc = new PVector(0,0);    
  }

  PVector compute_vel_change(){
    PVector seperation  = new PVector(0,0);
    PVector cohesion  = new PVector(0,0);
    PVector alignment  = new PVector(0,0);
    PVector vc  = new PVector(0,0);
    int count = 0;   
    for(int i = 0; i < agents.size(); i++){
     Agent current = agents.get(i);
     float dist = PVector.sub(pos, current.pos).mag();
     if(dist < inter_Radius && dist > 2*agent_radius){
        count++;
        seperation.add(PVector.sub(current.pos,pos).mult(10/(dist*dist)));
        alignment.add(current.vel.copy());
        cohesion.add(current.pos);
     }       
     else if (dist > 0 && dist < 2*agent_radius){
        pos.add(PVector.sub(pos, current.pos).mult((2*agent_radius-dist)/dist));
     }       
    }
    
    for(int i = 0; i < obstacles.size(); i++){
     Obstacle current = obstacles.get(i);
     float dist = PVector.sub(pos, current.center).mag();
     if(dist < inter_Radius && dist > agent_radius + obstacle_radius){
        count++;
        seperation.add(PVector.sub(current.center,pos).mult(20/(dist*dist)));
     }       
     else if (dist > 0 && dist < agent_radius + obstacle_radius){
        pos.add(PVector.sub(pos, current.center).mult((agent_radius + obstacle_radius - dist)/dist));
     }       
    }
    
    if(count == 0) 
      return null;

    seperation.div(count);
    alignment.div(count);
    cohesion.div(count).mult(10);

    vc.add(alignment).add(seperation).add(cohesion).div(30);
    return vc;
  }

  void set_Goal(PVector goal){
    this_Goal = new Milestone(goal.copy());
    this_Goal.find_Goal = true;
    searchPath(pos,this_Goal.pos);
  }

  void next_node_vel(PVector dir){
    next_vel = dir.mag()*10/dt/dt;
    //startTime = 0;
  }
  
  
  void searchPath(PVector start, PVector goal){
    this_Path = new ArrayList<Milestone>();
    index = 0;
    this_Start = new Milestone(start.copy());
    this_Goal = new Milestone(goal.copy());
    this_Goal.find_Goal =true;
    //this_Start.set_Node(start,goal);
    //this_Goal.set_Node(start,goal);
    
    //boolean feasible = check_Path(this_Goal.pos, this_Start.pos);
    vel = new PVector(0,0);
     
    if(check_Path(this_Goal.pos, this_Start.pos)){
      this_Goal.parent = this_Start;
      Link_milestons();
      //println("Now",this_Path.size());
      return;
    }
       
    for(int i = 0; i < sample_number; i ++){
      
      Milestone current = sample_points[i];
      current.set_Node(start,goal);
      current.heuristic = 0;
      boolean feasibleStart = check_Path(this_Start.pos, current.pos);
      boolean feasibleGoal = check_Path(this_Goal.pos, current.pos);    
      if (feasibleStart) {
        this_Start.neighbors.add(current);
        current.neighbors.add(this_Start);
      }
      if(feasibleGoal) {
        this_Goal.neighbors.add(current);
        current.neighbors.add(this_Goal);
      }
    }    
    if(bfs())
      Link_milestons();
  }
  
  void Link_milestons(){    
    Milestone current = this_Goal;    
    this_Path = new ArrayList<Milestone>();
    index = 0;
    while(current.parent!=null){
     this_Path.add(current);
     current = current.parent;
    }
    this_Path.add(current);
    Collections.reverse(this_Path);
    if(this_Path.size()> 1){
    next_node_vel(PVector.sub(this_Path.get(index+1).pos,this_Path.get(index).pos));
    }
    index++;
  }
  
  boolean bfs(){ 
    PriorityQueue<Milestone> fringe = new PriorityQueue<Milestone>(new Compare());
    this_Start.Visited = true;
    fringe.add(this_Start);   
    while (fringe.size() >0){
      Milestone current = fringe.poll();
      if(current.find_Goal) {
        return true;
      }
      for(int i = 0; i < current.neighbors.size(); i++){
        Milestone child = current.neighbors.get(i);
        if(!child.Visited){
          child.parent = current;
          child.cost = current.cost + check_Distance(child.pos,current.pos);
          child.Visited = true;
          fringe.add(child);
        }
      }
    }  
    return false;
  }
}
