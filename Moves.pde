
abstract class Move {
  
  Map<String,Parameter> parameters = new LinkedHashMap();
  PVector[][] trajectories;
  String name;
  
  abstract float getPeriod();
  
  abstract PVector getSpeed(float phase);
  
  abstract float getRotation(float phase);
  
  abstract PVector getFootPosition(int i, float phase);
  
  void update(){
    trajectories = getFeetTrajectories();
  }
  
  private PVector[][] getFeetTrajectories(){
    int npoints = min((int) (FPS * getPeriod()), 200);
    PVector[][] pts = new PVector[4][npoints];
    for(int i=0; i<4; i++)
      for(int j=0; j<npoints; j++)
        pts[i][j] = getFootPosition(i, ((float)j)/npoints);
    return pts;
  }
  
  Trajectory makeStepTrajectory(float dx, float dy, float dr, float h, float k){
    Trajectory t = new Trajectory();
    t.addSegment(new PVector(dx, dy + dx * tan(dr), 0.0), new PVector(-dx, dy - dx * tan(dr), 0.0));
    t.addSegment(new PVector(-dx, dy - dx * tan(dr), 0.0), new PVector(-dx, dy - dx * tan(dr), h), new PVector(-h, dy -h * tan(dr), h), new PVector(0, dy, h));
    t.addSegment(new PVector(0, dy, h), new PVector(dx, dy + dx * tan(dr), h));
    t.addSegment(new PVector(dx, dy + dx * tan(dr), h), new PVector(dx+h, dy + (dx+h) * tan(dr), h), new PVector(dx+h, dy + (dx+h) * tan(dr), 0), new PVector(dx, dy + dx * tan(dr), 0));
    t.setGroundRatio(k);
    return t;
  }
  
  void set(String name, float value){
    parameters.get(name).value = value;
    update();
  }
  
  float get(String name){
    return parameters.get(name).value;
  }
  
  void put(Parameter p){
    parameters.put(p.name, p);
  }
}

abstract class Steady extends Move {
  
  abstract PVector getSpeed();
  
  abstract float getRotation();
  
  PVector getSpeed(float phase){ return getSpeed();}
  
  float getRotation(float phase){ return getRotation();}
  
}

abstract class Transient extends Move {
    
}

class Walk extends Steady {
  
  Trajectory[] t = new Trajectory[4];
    
  Walk(){ this(0.0, 0.0); }
  
  Walk(float turn_radius){ this(turn_radius, 0.0); }
  
  Walk(float turn_radius, float dr){ this("WALK", turn_radius, dr); }
  
  Walk(String name, float turn_radius){ this(name, turn_radius, 0); }
  
  Walk(String name, float turn_radius, float dr){
    put(new Parameter("speed", 20.0, 0.0, 100.0));
    put(new Parameter("dx", 20.0, 0.0, 50.0));
    put(new Parameter("dy", 15.0, 0.0, 50.0));
    put(new Parameter("dz", 10.0, 0.0, 50.0));
    put(new Parameter("dr", dr, 0.0, 60.0));
    put(new Parameter("k_ground", 0.8, 0.0, 1.0));
    put(new Parameter("radius", turn_radius, 0, 1000));
    put(new Parameter("phase1", 0.25, 0, 1.0));
    put(new Parameter("phase2", 0.00, 0, 1.0));
    put(new Parameter("phase3", 0.75, 0, 1.0));
    put(new Parameter("phase4", 0.50, 0, 1.0));
    this.name = name;
    update();
  }
  
  void update(){
    float dx = SCALE*get("dx");
    float dy = SCALE*get("dy");
    float dz = SCALE*get("dz");
    float dr = radians(get("dr"));
    float k = get("k_ground");
    float radius = SCALE*get("radius");
    if(radius==0.0){
      t[0] = makeStepTrajectory(dx, -dy, dr, dz, k);  // Left forward
      t[1] = makeStepTrajectory(dx, -dy, dr, dz, k);  // Left back
      t[2] = makeStepTrajectory(dx,  dy, dr, dz, k);  // Right forward
      t[3] = makeStepTrajectory(dx,  dy, dr, dz, k);  // Right back
    }else{
      float r1 = sqrt(pow(radius-dy,2) + pow(frame.getLength(),2)/4);
      float r2 = sqrt(pow(radius+frame.getWidth()+dy,2) + pow(frame.getLength(),2)/4);
      float omega = getRotation();
      float dr1 = atan(frame.getLength()*0.5/(radius-dy));
      float dr2 = atan(frame.getLength()*0.5/(radius+frame.getWidth()+dy));
      float v1 = r1 * omega;
      float v2 = r2 * omega;
      float dx1 = k * getPeriod() * v1 * 0.5 * cos(dr1);
      float dx2 = k * getPeriod() * v2 * 0.5 * cos(dr2);
      t[0] = makeStepTrajectory(dx2, -dy, -dr2, dz, k);  // Left forward
      t[1] = makeStepTrajectory(dx2, -dy,  dr2, dz, k);  // Left back
      t[2] = makeStepTrajectory(dx1,  dy, -dr1, dz, k);  // Right forward
      t[3] = makeStepTrajectory(dx1,  dy,  dr1, dz, k);  // Right back
    }
    super.update();
  }
  
  float getPeriod(){  // Period of the movement
    return (SCALE*get("dx") / cos(radians(get("dr")))) / (0.5 * get("k_ground") * getSpeed().mag());
  }
  
  PVector getFootPosition(int i, float phase){
    float ph = (phase + get("phase"+(i+1))) % 1.0;
    return t[i].point(ph).add(legs[i].slot).add(0,0,-frame.getHeight());
  }
  
  PVector getSpeed(){ return new PVector(SCALE*get("speed"), SCALE*get("speed")*tan(radians(get("dr"))), 0.0); }
  
  float getRotation(){ return get("radius")==0.0? 0.0 : getSpeed().x / (SCALE*get("radius")+0.5*frame.getWidth()); }
}


class Stand extends Steady {
    
  Stand(){
    this.name = "STAND"; 
  }
  
  float getPeriod(){ return 1.0; }
  
  PVector getFootPosition(int i, float phase){ return legs[i].slot.add(0,0,-frame.getHeight()); }
  
  PVector getSpeed(){ return new PVector(0.0, 0.0, 0.0); }
  
  float getRotation(){ return 0.0; }
  
}

/*
class Start extends Transient {
  
  Start(double duration){
    super(duration);
  }
  
}
*/

class Parameter {
  String name;
  float value, min, max;
  
  Parameter(String n, float v, float min, float max){
    this.name = n;
    this.value = v;
    this.min = min;
    this.max = max;
  }
}