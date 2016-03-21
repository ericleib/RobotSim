
class MovePlanner {
  
  List<Move> moves = new ArrayList();   // List of planned moves
  
  void addMove(Move move, float duration) {  // Moves are added at the beginning
    move.duration = duration;
    move.phase0 = moves.size() == 0 ? 0.0 : moves.get(moves.size()-1).phase0 + moves.get(moves.size()-1).duration;
    move.update();  // init trajectories, taking start phase into account
    moves.add(move);
  }
  
  void addMove(Move move){ addMove(move, 0.0); }
  
  void addMoveWithTransient(Steady move, float transientDuration, float duration){
    Steady last = (Steady) moves.get(moves.size()-1);
    addMove(new LinearTransient(last, move), transientDuration);
    addMove(move, duration);
  }
  
  Move getMove(float phase){  // Moves are retrieved in function of phase
    Move move = moves.get(0);
    for(Move m : moves){
      if(m.phase0 > phase)
        break;
      move = m;
    }
    if(move == moves.get(moves.size()-1))
      UI.createSliders(move);    // Sliders are added only for the last move
    else
      UI.removeSliders();
    return move;
  }
}


abstract class Move {
  
  Map<String,Parameter> parameters = new LinkedHashMap();
  PVector[][] trajectories;
  float phase0, duration;
  
  abstract String getName();
  
  abstract float getPeriod(float phase);
  
  abstract PVector getSpeed(float phase);
  
  abstract float getRotation(float phase);
  
  abstract PVector getFootPosition(int i, float phase);
    
  void update(boolean quick){
    if(!quick)
      trajectories = getFeetTrajectories(phase0);
  }
  
  void update(){
    update(false);
  }
  
  void apply(float phase){
    for(int i=0; i<4; i++){  // For each leg
      ROBOT.legs[i].foot = getFootPosition(i, phase);  // Trajectory planning
      ROBOT.legs[i].footTrajectory = MOVE.trajectories[i]; 
      ROBOT.legs[i].resolve();  // Inverse kinematics & CG
    }
    ROBOT.resolve();  // Compute CG and stability triangles
    ROBOT.ground.resolve(this, phase, TIME.dt);  // Compute ground position variation 
  }
  
  protected PVector[][] getFeetTrajectories(float phase){
    int npoints = min((int) (TIME.fps * getPeriod(phase)), 200);
    PVector[][] pts = new PVector[4][npoints];
    for(int i=0; i<4; i++)
      for(float j=0; j<npoints; j++)
        pts[i][(int)j] = getFootPosition(i, phase + j/npoints);
    return pts;
  }
  
  Trajectory makeStep(PVector foot, float dx, float dr, float h, float k){
    Trajectory t = new Trajectory();
    if(dx!=0){
      float dy = dx * tan(dr);
      float dyh = dy * h / dx;
      t.addSegment(foot.copy().add(dx, dy, 0),     foot.copy().add(-dx, -dy, 0));
      t.addSegment(foot.copy().add(-dx, -dy, 0.0), foot.copy().add(-dx, -dy, h),     foot.copy().add(-h, - dyh, h),    foot.copy().add(0, 0, h));
      t.addSegment(foot.copy().add(0, 0, h),       foot.copy().add(dx, dy, h));
      t.addSegment(foot.copy().add(dx, dy, h),     foot.copy().add(dx+h, dy+dyh, h), foot.copy().add(dx+h, dy+dyh, 0), foot.copy().add(dx, dy, 0));
    }else{
      t.addSegment(foot);
    }
    t.setGroundRatio(k);
    return t;
  }
  
  PVector getSpeed(float phase, PVector point){
    return getSpeed(phase).add(ROBOT.ref.copy().sub(point).cross(new PVector(0,0,getRotation(phase))));  // Speed of the point = Speed(CG) + point->cg ^ Rotation
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


// STEADY MOVES

abstract class Steady extends Move {
  
  abstract float getPeriod();
  
  abstract PVector getSpeed();
  
  abstract float getRotation();
  
  float getPeriod(float phase){ return getPeriod();}
  
  PVector getSpeed(PVector point){ return getSpeed(0.0, point); }  // Speed of the point = Speed(CG) + point->cg ^ Rotation
  
  PVector getSpeed(float phase){ return getSpeed();}
  
  float getRotation(float phase){ return getRotation();}
  
}

class Walk extends Steady {
  
  Trajectory[] t = new Trajectory[4];
  Oscillation osc;
    
  Walk(){ this(0.0, 0.0); }
  
  Walk(float rotation){ this(rotation, 0.0); }
  
  Walk(float rotation, float dr){
    put(new Parameter("speed", 20.0, 0.0, 100.0));
    put(new Parameter("shift_x", -2.0, -20.0, 20.0));
    put(new Parameter("shift_y", 0.0, -20.0, 20.0));
    put(new Parameter("dx", 40.0, 0.0, 50.0));
    put(new Parameter("dy", 25.0, 0.0, 50.0));
    put(new Parameter("dz", 10.0, 0.0, 50.0));
    put(new Parameter("dr", dr, 0.0, 60.0));
    put(new Parameter("k_ground", 0.8, 0.0, 1.0));
    put(new Parameter("rotation", rotation, 0.0, 10.0));
    put(new Parameter("phase1", 0.25, 0, 1.0));
    put(new Parameter("phase2", 0.00, 0, 1.0));
    put(new Parameter("phase3", 0.75, 0, 1.0));
    put(new Parameter("phase4", 0.50, 0, 1.0));
    put(new Parameter("phase_osc", 0.20, 0, 1.0));
    put(new Parameter("ampl_osc", 10.0, 0, 20.0));
  }
  
  String getName(){
    return getRotation()==0.0 ? (get("speed")==0? "Standing" : (get("dr")==0? "Straight " : "Crab ")+"Walk") : "Turn "+nf(degrees(getRotation()),1,2)+"°/s";
  }
  
  void update(boolean quick){
    osc = new Oscillation(new PVector(0,0,0), new PVector(0,0,0)); // No oscillation to define the nominal trajectories of the legs (for turns)
    for(int i=0; i<4; i++){      // for each leg
      Leg leg = ROBOT.legs[i];
      PVector foot_ref = leg.slot.copy().add(get("shift_x"), get("shift_y") + get("dy") * leg.right(), -ROBOT.height);
      PVector v = getSpeed(foot_ref);  // Local speed of the leg : Problem: this includes the oscillations...
      float dx = get("dx") * (getRotation()==0.0 ? 1.0 : v.x / (get("speed")));
      float dr = getRotation()==0.0 ? radians(get("dr")) : atan(v.y/v.x); // warning: some particular cases where rotation!=0 and v.x = 0
      //println(dx+" "+dy+" "+degrees(dr)+" "+get("dz")+" "+k);
      t[i] = makeStep(foot_ref, dx, dr, get("dz"), get("k_ground"));  // Left forward //<>//
    }
    osc = new Oscillation(new PVector(0,-get("ampl_osc"),0), new PVector(0,get("ampl_osc"),0), get("phase_osc"), 1.0);  // Now we add oscillations
    super.update(quick);
  }
  
  float getPeriod(){  // Period of the movement
    if(get("dx")==0.0) return 1.0; // Case of degenerate trajectory
    return get("dx") / (0.5 * get("k_ground") * get("speed"));
  }
  
  PVector getFootPosition(int i, float phase){
    float ph = (phase + get("phase"+(i+1))) % 1.0;
    return t[i].point(ph).add(osc.pointLin(phase));
  }
  
  PVector getSpeed(){ return new PVector(get("speed"), get("speed")*tan(radians(get("dr"))), 0.0).add(osc.speed(TIME.phase).mult(1.0/getPeriod())); }
  
  float getRotation(){ return radians(get("rotation")); }
}


class Stand extends Walk {
    
  Stand(){
    super();
    parameters.get("speed").value = 0.0;
    parameters.get("dx").value = 0.0;    // dx set to zero so that the period = 1.0
    parameters.get("dz").value = 0.0;
    parameters.get("ampl_osc").value = 0.0;
  }
  
}

// TRANSIENT MOVES

abstract class Transient extends Move {
  
  Steady move1, move2;
  
  Transient(Steady move1, Steady move2){
    this.move1 = move1;
    this.move2 = move2;
  }
    
}

class LinearTransient extends Transient {
    
  PVector[][][] alltrajectories;
  
  Move move;
  float phase_ = -1.0;
  
  LinearTransient(Steady move1, Steady move2){
    super(move1, move2);
    try{
      move = move1.getClass().getDeclaredConstructor(Robot.this.getClass()).newInstance(Robot.this);
    }catch(Exception e){
      e.printStackTrace();
      println(move1.getClass().toString()+" must have a default constructor!");
    }
  }
  
  String getName(){
    return move1.getName() + " > " + move2.getName();
  }
  
  void update(boolean quick){
    alltrajectories = new PVector[(int) duration][][];
    setParameters(phase0);
    for(float i=0; i<alltrajectories.length; i++)
      alltrajectories[(int)i] = getFeetTrajectories(phase0+i);
  }
  
  void setParameters(float phase){
    if(this.phase_ != phase){
      phase_ = phase;
      float phase_in = (phase-phase0)/duration;
      for(Parameter p : move1.parameters.values())
        move.parameters.get(p.name).value = (1.0-phase_in) * p.value + phase_in * move2.get(p.name);
      move.update(true); // Recompute move's trajectories
      trajectories = alltrajectories[(int)(phase-phase0)]; //<>//
    }
  }
  
  float getPeriod(float phase){
    setParameters(phase);
    return move.getPeriod(phase);
  }
  
  PVector getSpeed(float phase){
    setParameters(phase);
    return move.getSpeed(phase);
  }
  
  float getRotation(float phase){
    setParameters(phase);
    return move.getRotation(phase);
  }
  
  PVector getFootPosition(int i, float phase){
    setParameters(phase);
    return move.getFootPosition(i, phase);
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