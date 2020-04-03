class Obstacle{
  float r;
  PVector center;
  
  Obstacle(float radius, float x, float y){
    r = radius;
    center = new PVector(x,y);
  }
  
  void move(float x, float y){
    center = PVector.add(new PVector(x,y),center);
  }
}

class Milestone{
  PVector pos;
  float cost;
  float heuristic;
  float f;
  boolean find_Goal = false;
  boolean Visited = false;  
  
  ArrayList<Milestone> neighbors = new ArrayList<Milestone>();
  Milestone parent;
  
  Milestone(PVector position){
    pos = position;
  }
  
  void update_f(float g){
    cost = g;
    f = cost + heuristic;
  }
    
  void set_Node(PVector start, PVector goal){
    cost = PVector.sub(pos,start).mag();
    heuristic = PVector.sub(pos,goal).mag();
    f = cost + heuristic;
    parent = null;
    Visited = false;
  }

}

class Compare implements Comparator<Milestone>{
    public int compare(Milestone m1, Milestone m2) {
        return Float.compare(m1.f, m2.f);
    }  
}
