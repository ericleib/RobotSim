
class MovePlanner {
  
  List<Move> moves = new ArrayList();   // List of planned moves
  Move move;
  
  void addMove(Move move, float duration) {  // Moves are added at the beginning
    move.duration = duration;
    move.phase0 = moves.size() == 0 ? 0.0 : moves.get(moves.size()-1).phase0 + moves.get(moves.size()-1).duration;
    move.update();  // init trajectories, taking start phase into account
    moves.add(move);
  }
  
  void addMove(Move move){ addMove(move, 0.0); }
  
  void addMoveWithTransient(Move move, float transientDuration, float duration){
    Move last = moves.get(moves.size()-1);
    addMove(new LinearTransient(last, move), transientDuration);
    addMove(move, duration);
  }
  
  Move getMove(float phase){  // Moves are retrieved in function of phase
    move = moves.get(0);
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
  
  abstract float getHeight(float phase);
  
  abstract float getPitch(float phase);
  
  abstract float getRoll(float phase);
  
  abstract PVector getFootPosition(int i, float phase);
    
  void update(boolean quick){
    if(!quick)
      trajectories = getFeetTrajectories(phase0);
  }
  
  void update(){
    update(false);
  }
    
  protected PVector[][] getFeetTrajectories(float phase){
    int npoints = min((int) (TIME.fps * getPeriod(phase)), 200);
    PVector[][] pts = new PVector[4][npoints];
    for(int i=0; i<4; i++)
      for(float j=0; j<npoints; j++)
        pts[i][(int)j] = getFootPosition(i, phase + j/npoints);
    return pts;
  }
  
  Trajectory makeStep(PVector foot, float dx, float dr, float dz, float k){
    Trajectory t = new Trajectory();
    if(dx!=0 || dz!=0){
      float dy = dx * tan(dr);
      t.addSegment(foot.copy().add(dx, dy, 0),     foot.copy().add(-dx, -dy, 0), true);
      t.addSegment(foot.copy().add(-dx, -dy, 0.0), foot.copy().add(-dx, -dy, dz), foot.copy().add(0, 0, dz), false);
      t.addSegment(foot.copy().add(0, 0, dz),      foot.copy().add(dx, dy, dz),   foot.copy().add(dx, dy, 0), false);
    }else{
      t.addSegment(foot, true);
    }
    t.setGroundRatio(k);
    return t;
  }
  
  PVector getSpeed(float phase, PVector point){
    return getSpeed(phase).add(ROBOT.frame.ref.copy().sub(point).cross(new PVector(0,0,getRotation(phase))));  // Speed of the point = Speed(CG) + point->cg ^ Rotation
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

class Walk extends Move {
  
  Trajectory[] t = new Trajectory[4];
  Oscillation osc;
    
  Walk(){ this(0.0, 0.0); }
  
  Walk(float rotation){ this(rotation, 0.0); }
  
  Walk(float rotation, float dr){
    put(new Parameter("speed", 20.0, 0.0, 100.0));
    put(new Parameter("shift_x", -2.0, -20.0, 20.0));
    put(new Parameter("shift_y", 0.0, -20.0, 20.0));
    put(new Parameter("height", 85.0, 10.0, 100.0)); 
    put(new Parameter("dx", 40.0, 0.0, 50.0));
    put(new Parameter("dy", 25.0, 0.0, 50.0));
    put(new Parameter("dz", 10.0, 0.0, 50.0));
    put(new Parameter("dr", dr, 0.0, 60.0));
    put(new Parameter("k_ground", 0.8, 0.0, 1.0));
    put(new Parameter("rotation", rotation, 0.0, 10.0));
    put(new Parameter("pitch", 0.0, -20.0, 20.0)); 
    put(new Parameter("roll", 0.0, -20.0, 20.0)); 
    put(new Parameter("phase1", 0.25, 0, 1.0));
    put(new Parameter("phase2", 0.00, 0, 1.0));
    put(new Parameter("phase3", 0.75, 0, 1.0));
    put(new Parameter("phase4", 0.50, 0, 1.0));
    put(new Parameter("phase_osc", 0.20, 0, 1.0));
    put(new Parameter("ampl_osc", 10.0, 0, 20.0));
  }
  
  String getName(){
    return getRotation(phase0)==0.0 ? (get("speed")==0? "Standing" : (get("dr")==0? "Straight " : "Crab ")+"Walk") : "Turn "+nf(degrees(getRotation(phase0)),1,2)+"°/s";
  }
  
  void update(boolean quick){
    osc = new Oscillation(new PVector(0,0,0), new PVector(0,0,0)); // No oscillation to define the nominal trajectories of the legs (for turns)
    for(int i=0; i<4; i++){      // for each leg
      PVector foot_ref = ROBOT.legs[i].foot_ref.copy().add(get("shift_x"), get("shift_y") + get("dy") * ROBOT.legs[i].right());
      PVector v = getSpeed(phase0, foot_ref);  // Local speed of the leg : Problem: this includes the oscillations...
      float dx = get("dx") * (getRotation(phase0)==0.0 ? 1.0 : v.x / (get("speed")));
      float dr = getRotation(phase0)==0.0 ? radians(get("dr")) : atan(v.y/v.x); // warning: some particular cases where rotation!=0 and v.x = 0
      //println(dx+" "+dy+" "+degrees(dr)+" "+get("dz")+" "+k);
      t[i] = makeStep(foot_ref, dx, dr, get("dz"), get("k_ground"));  // Left forward //<>//
    }
    osc = new Oscillation(new PVector(0,-get("ampl_osc"),0), new PVector(0,get("ampl_osc"),0), get("phase_osc"), 1.0);  // Now we add oscillations
    super.update(quick);
  }
  
  float getPeriod(float p){  // Period of the movement
    if(get("dx")==0.0) return 1.0; // Case of degenerate trajectory
    return get("dx") / (0.5 * get("k_ground") * get("speed"));
  }
  
  PVector getFootPosition(int i, float phase){
    float ph = (phase + get("phase"+(i+1))) % 1.0;
    return t[i].point(ph).add(osc.pointLin(phase));
  }
  
  PVector getSpeed(float p){ return new PVector(get("speed"), get("speed")*tan(radians(get("dr"))), 0.0).add(osc.speed(p).mult(1.0/getPeriod(p))); }
  
  float getRotation(float p){ return radians(get("rotation")); }
  
  float getHeight(float p){ return get("height"); }
  
  float getPitch(float p){ return radians(get("pitch")); }
  
  float getRoll(float p){ return radians(get("roll")); }
  
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

class LinearTransient extends Move {
    
  PVector[][][] alltrajectories;
  
  Move move1, move2;
  Move move;
  float phase_ = -1.0;
  
  LinearTransient(Move move1, Move move2){
    this.move1 = move1;
    this.move2 = move2;
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
    alltrajectories = new PVector[(int) ceil(duration)][][];
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
      move.update(true); // Recompute move's trajectory
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
  
  float getHeight(float phase){
    setParameters(phase);
    return move.getHeight(phase);
  }
  
  float getPitch(float phase){
    setParameters(phase);
    return move.getPitch(phase);
  }
  
  float getRoll(float phase){
    setParameters(phase);
    return move.getRoll(phase);
  }
  
  PVector getFootPosition(int i, float phase){
    setParameters(phase);
    return move.getFootPosition(i, phase);
  }
    
}


class PilotedMove extends LinearTransient {
  
  PilotedMove(){
    super(new Walk(), new Walk());
    parameters.putAll(move1.parameters);
  }
  
  String getName(){
    return "Piloted move";
  }
  
  void setParameters(float phase){
    super.setParameters(min(phase, phase0 + duration - 0.001));
  }
  
  void set(String name, float value){
    for(Parameter p:parameters.values())
      move1.parameters.get(p.name).value = move.get(p.name);
    move2.parameters.get(name).value = value;
    phase0 = TIME.phase;
    update();
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