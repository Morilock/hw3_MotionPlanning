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
  float expectedDuration = 0;
  boolean reachGoal = false; 
  float dt = 0.5;
  float r;

  
  Agent(PVector position, float radius){
    pos = new PVector(position.x, position.y);
    goal_vel = new PVector(0,0);
    r = radius;
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
     /*
    if (this_Path.size() == 1){
      PVector thisgoal = this_Path.get(0).pos;
      searchPath(pos, thisgoal);
    }*/
    //println(this_Path.get(0).pos);
    float dist_F = check_Distance(this_Path.get(index).pos,pos);
    //println(dist_F);
    
    if(dist_F < 2*sample_radius && index < this_Path.size() -1){
        next_node_vel(PVector.sub(this_Path.get(index+1).pos,this_Path.get(index).pos));
        index ++;
    }
    
    goal_vel = PVector.sub(this_Path.get(index).pos,pos).normalize().mult(5).sub(vel);
    if(vc.mag() > 0)  vel.add(PVector.mult(vc,dt));
    vel.add(PVector.mult(goal_vel,dt));    
    vel.limit(5);
    pos.add(PVector.mult(vel,dt));
    vc = new PVector(0,0);    
  }

  void compute_vel_change(){
    PVector escape = new PVector(0,0);      
      for(int i = 0; i < agents.size(); i++){
       Agent current = agents.get(i);
       float dist = PVector.sub(pos, current.pos).mag();
       if(dist < inter_Radius && dist > 0){
          escape.add(vel_change(current.pos, current.vel, current.r));
       }
    }
    
    for(int i = 0; i < obstacles.size(); i++){
      Obstacle current = obstacles.get(i);
      escape.add(vel_change(current.center, new PVector(0,0), current.r));
    }
    
    vc.add(escape);
    vc.limit(20);
  }
  
  PVector vel_change(PVector Pos2, PVector Vel2, Float R2){
    PVector escape = new PVector(0,0);
    float coll_time = Float.MAX_VALUE;    
    PVector dpos =  PVector.sub(Pos2, pos);
    PVector dvel = PVector.sub(vel,Vel2);

    float i = sq(PVector.dot(dpos,dvel)) - PVector.dot(dvel,dvel) * (PVector.dot(dpos,dpos) - sq(R2 + r));
    
    if(i > 0) 
      coll_time = (PVector.dot(dpos,dvel) - sqrt(i))/PVector.dot(dvel,dvel);
    else 
      return escape;
    
    float MAX_coll_time = 100;
    if(coll_time > 0 && coll_time < MAX_coll_time) {
      PVector MyFuturePos = PVector.add(pos, PVector.mult(vel,coll_time));
      PVector ObsFuturePos = PVector.add(Pos2, PVector.mult(Vel2,coll_time));
      escape = PVector.sub(MyFuturePos,ObsFuturePos).normalize();
    }
    
    float multi_t, max_times = 1000;
    if(coll_time > 0){
      multi_t = (MAX_coll_time - coll_time)/(coll_time +0.001);
      if(multi_t > max_times) 
        multi_t = max_times;       
    }
    else{
      multi_t = max_times;
    }
    escape.mult(multi_t);
    return escape.copy();
  }

  void set_Goal(PVector goal){
    this_Goal = new Milestone(goal.copy());
    this_Goal.find_Goal = true;
    searchPath(pos,this_Goal.pos);
  }
  
  void next_node_vel(PVector dir){
    expectedDuration = dir.mag()*5/dt/dt*1.5;
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
  /*
  boolean A_star(){
    PriorityQueue<Milestone> fringe = new PriorityQueue<Milestone>(new Compare());
    
  }*/
}
