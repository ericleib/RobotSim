
class MovePlanner {
  
  List<Move> moves = new ArrayList();
  List<Float> durations = new ArrayList();
  List<Float> starts = new ArrayList();
  int imove = 0;
  boolean constantMove = true;
  
  void addMove(Move move, float duration){
    moves.add(move);
    durations.add(duration);
    if(moves.size()==1)
      starts.add(0.0);
  }
  
  void addMove(Move move){
    addMove(move, 0.0);
  }
  
  void addMove(Transient move){
    addMove(move, move.getPeriod());
  }
  
  void reset(){
    starts.clear();
    imove = 0;
    starts.add(0.0);
  }
  
  void setMove(){
    if(constantMove){
      if(move != moves.get(0)){
        move = moves.get(0);
        createSliders();
      }
    } else {
      move = moves.get(imove);
      float duration = durations.get(imove);
      float start = starts.get(imove);
      //println(move.name+" "+time+" "+start+" "+duration);
      if(duration>0 && time - start >= duration && move.finished(phase)){  // Note: this blocks assumes that times goes positively... Next button fails...
        imove++;
        starts.add(time);
        if(move instanceof Transient)
          phase = ((Transient)move).phase2;  // If the previous move is a transient, we reset the phase to the initial phase of the next move
        move = moves.get(imove);
        if(move instanceof Transient)  // If the new move is a transient, set the phase to 0
          phase = 0.0;
          
      }else if(time < start){
        imove--;
        starts.remove(starts.size()-1);
        setMove();
      }
    }
  }
}

abstract class Move {
  
  Map<String,Parameter> parameters = new LinkedHashMap();
  PVector[][] trajectories;
  String name;
  
  abstract float getPeriod();
  
  abstract PVector getSpeed(float phase);
  
  abstract float getRotation(float phase);
  
  abstract PVector getFootPosition(int i, float phase);
  
  abstract boolean finished(float phase);
  
  void update(boolean quick){
    if(!quick)
      trajectories = getFeetTrajectories();
  }
  
  void update(){
    update(false);
  }
  
  protected PVector[][] getFeetTrajectories(){
    int npoints = min((int) (FPS * getPeriod()), 200);
    PVector[][] pts = new PVector[4][npoints];
    //println("get trajectories: "+getPeriod()+" "+npoints);
    for(int i=0; i<4; i++){
      //println("leg: "+i);
      for(int j=0; j<npoints; j++)
        pts[i][j] = getFootPosition(i, ((float)j)/npoints);
    }
    return pts;
  }
  
  Trajectory makeStepTrajectory(float dx, float dy, float dr, float h, float k){
    Trajectory t = new Trajectory();
    if(dx!=0){
      t.addSegment(new PVector(dx, dy + dx * tan(dr), 0.0), new PVector(-dx, dy - dx * tan(dr), 0.0));
      t.addSegment(new PVector(-dx, dy - dx * tan(dr), 0.0), new PVector(-dx, dy - dx * tan(dr), h), new PVector(-h, dy -h * tan(dr), h), new PVector(0, dy, h));
      t.addSegment(new PVector(0, dy, h), new PVector(dx, dy + dx * tan(dr), h));
      t.addSegment(new PVector(dx, dy + dx * tan(dr), h), new PVector(dx+h, dy + (dx+h) * tan(dr), h), new PVector(dx+h, dy + (dx+h) * tan(dr), 0), new PVector(dx, dy + dx * tan(dr), 0));
    }else{
      t.addSegment(new PVector(0, dy, 0));
    }
    t.setGroundRatio(k);
    return t;
  }
  
  PVector getSpeed(float phase, PVector point){
    return getSpeed(phase).add(frame.getCG().sub(point).cross(new PVector(0,0,getRotation(phase))));  // Speed of the point = Speed(CG) + point->cg ^ Rotation
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
  
  PVector getSpeed(PVector point){ return getSpeed().add(frame.getCG().sub(point).cross(new PVector(0,0,getRotation()))); }  // Speed of the point = Speed(CG) + point->cg ^ Rotation
  
  PVector getSpeed(float phase){ return getSpeed();}
  
  float getRotation(float phase){ return getRotation();}
  
  boolean finished(float phase){ return true; }
}

abstract class Transient extends Move {
  
  Steady move1, move2;
  float phase1, phase2;
  float t1, t2;
  
  Transient(Steady move1, Steady move2, float phase1, float phase2){
    this.move1 = move1;
    this.move2 = move2;
    this.phase1 = phase1;
    this.phase2 = phase2;
    t1 = move1.getPeriod();
    t2 = move2.getPeriod();
  }
  
  Transient(Steady move1, Steady move2){
    this(move1, move2, 1.0, 0.0);
  }
  
}

class Walk extends Steady {
  
  Trajectory[] t = new Trajectory[4];
    
  Walk(){ this(0.0, 0.0); }
  
  Walk(float rotation){ this(rotation, 0.0); }
  
  Walk(float rotation, float dr){
    put(new Parameter("speed", 20.0, 0.0, 100.0));
    put(new Parameter("dx", 20.0, 0.0, 50.0));
    put(new Parameter("dy", 15.0, 0.0, 50.0));
    put(new Parameter("dz", 10.0, 0.0, 50.0));
    put(new Parameter("dr", dr, 0.0, 60.0));
    put(new Parameter("k_ground", 0.8, 0.0, 1.0));
    put(new Parameter("rotation", rotation, 0.0, 10.0));
    put(new Parameter("phase1", 0.25, 0, 1.0));
    put(new Parameter("phase2", 0.00, 0, 1.0));
    put(new Parameter("phase3", 0.75, 0, 1.0));
    put(new Parameter("phase4", 0.50, 0, 1.0));
    update();
  }
  
  void update(boolean quick){
    name = getRotation()==0.0 ? (getSpeed().x==0? "Standing" : (get("dr")==0? "Straight " : "Crab ")+"Walk") : "Turn "+nf(degrees(getRotation()),1,2)+"°/s";
    float dy = SCALE*get("dy");
    float k = get("k_ground");
    for(int i=0; i<4; i++){      // for each leg
      PVector v = getSpeed(legs[i].slot.copy().add(0, legs[i].right? dy : -dy, 0));  // Local speed of the leg
      float dx = SCALE*get("dx") * (getRotation()==0.0 ? 1.0 : v.x / getSpeed().x);
      float dr = getRotation()==0.0 ? radians(get("dr")) : atan(v.y/v.x); // warning: some particular cases where rotation!=0 and v.x = 0
      //println(dx+" "+dy+" "+degrees(dr)+" "+get("dz")+" "+k);
      t[i] = makeStepTrajectory(dx, legs[i].right? dy : -dy, dr, SCALE*get("dz"), k);  // Left forward //<>//
    }
    super.update(quick);
  }
  
  float getPeriod(){  // Period of the movement
    if(get("dx")==0.0) return 1.0; // Case of degenerate trajectory
    return SCALE*get("dx") / (0.5 * get("k_ground") * getSpeed().x);
  }
  
  PVector getFootPosition(int i, float phase){
    float ph = (phase + get("phase"+(i+1))) % 1.0;
    return t[i].point(ph).add(legs[i].slot).add(0,0,-frame.getHeight());
  }
  
  PVector getSpeed(){ return new PVector(SCALE*get("speed"), SCALE*get("speed")*tan(radians(get("dr"))), 0.0); }
  
  float getRotation(){ return radians(get("rotation")); }
}


class Stand extends Walk {
    
  Stand(){
    super();
    parameters.get("speed").value = 0.0;
    parameters.get("dx").value = 0.0;    // dx set to zero so that the period = 1.0
    parameters.get("dz").value = 0.0;
    update();
  }
  
}

class LinearTransient extends Transient {
  
  
  int npoints;
  float dp;
  PVector[][][] alltrajectories;
  
  Move move;
  float phase; // Only used to avoid redundant executions
  
  LinearTransient(Steady move1, Steady move2, float phase1, float phase2, float duration){
    super(move1, move2, phase1, phase2);
    put(new Parameter("duration", duration, 1.0, 20.0));
    println("Transient: "+phase1+" "+phase2);
    this.name = "Transient";
    try{
      move = move1.getClass().getDeclaredConstructor(Robot.this.getClass()).newInstance(Robot.this);
    }catch(Exception e){
      e.printStackTrace();
      println(move1.getClass().toString()+" must have a default constructor!");
    }
    npoints = (int) (getPeriod() * FPS);
    dp = 1.0 / npoints;
    trajectories = getFeetTrajectories();
    setParameters(0.0);
    update(true);
  }
  
  LinearTransient(Steady move1, Steady move2){
    this(move1, move2, 1.0, 0.0, 5.0);
  }
    
  void setParameters(float phase){
    if(LinearTransient.this.phase != phase){
      this.phase = phase;
      for(Parameter p : move1.parameters.values())
        move.parameters.get(p.name).value = phase(phase, p.value, move2.get(p.name));
      move.update(true);
      trajectories = alltrajectories[min(getNPeriod(phase),alltrajectories.length-1)]; //<>//
    }
  }
  
  protected PVector[][] getFeetTrajectories(){
    int nperiods = getNPeriod(1.0)+1, i=0;
    alltrajectories = new PVector[nperiods][4][];
    List<PVector[]> list = new ArrayList();
    for(float p=0.0; i < nperiods; p+=dp){
      PVector[] pts = new PVector[4];
      for(int j=0; j<4; j++)
        pts[j] = getFootPosition(j, p);
      list.add(pts);
      if(getNPeriod(p) > i){ //|| p+dp>1.0
        for(int j=0; j<4; j++){
          alltrajectories[i][j] = new PVector[list.size()];
          for(int k=0; k<list.size(); k++){
            alltrajectories[i][j][k] = list.get(k)[j];
          }
        }
        i++;
        list.clear();
      }
      //println(p+" "+getNPeriod(p)+" "+i+"/"+nperiods);
    }
    return alltrajectories[0];
  }
    
  int getNPeriod(float phase){
    float ip=0.0, ip_1=0.0;
    int period = 0;
    for(float p=0.0; p<=phase; p+=dp, ip_1 = ip){
      ip = getInnerPhase(p) % 1.0;
      if(ip < ip_1) period++;
      //println(p+" "+ip+" "+period+" "+t1+" "+t2);
    }
    return period;
  }
  
  float phase(float p, float v1, float v2){
    return (1.0-min(p,1.0)) * v1 + min(p,1.0) * v2;
  }
  
  float getInnerPeriod(float phase){
    return phase(phase, t1, t2);
  }
  
  float getInnerPhase(float phase){
    if(t1==t2) return phase * get("duration") / t1;
    if(phase<=1.0) return get("duration") / (t2-t1) * log( 1 + phase*(t2-t1)/t1 );
    float p = get("duration") / (t2-t1) * log( 1 + (t2-t1)/t1 );  // Innerphase up to phase = 1
    return p + (phase-1.0) * get("duration") / t2;  // Remainder where period is constant = t2
  }
  
  float getPeriod(){
    return get("duration");
  }
  
  PVector getSpeed(float phase){
    setParameters(phase);
    return move.getSpeed(getInnerPhase(phase));
  }
  
  float getRotation(float phase){
    setParameters(phase);
    return move.getRotation(getInnerPhase(phase));
  }
  
  PVector getFootPosition(int i, float phase){
    setParameters(phase);
    return move.getFootPosition(i, getInnerPhase(phase));
  }
  
  boolean finished(float phase){
    return phase >= 1.0 && (getInnerPhase(phase) - phase2) % 1.0 <= dt / getInnerPeriod(phase);
  }
  
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