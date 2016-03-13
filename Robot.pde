
// Simulation parameters
int FPS = 50;    // Frames per second
float SCALE = 2.5;  // pixels / mm
float TIME = 0.0;  // Time (in seconds)
float TIME_1 = 0.0;  // Time - 1 (in seconds)
float DT = 1.0 / FPS;
float PHASE = 0.0;  // General phase, which controls the progress of moves
boolean PAUSE = false;  // Whether the simulation is paused or not
boolean RESET = false;  // Flag to reset the simulation

// Objects in the simulation
Ground ground;
Frame frame;
Leg[] legs = new Leg[4];

// Current move of the robot
MovePlanner planner;
Move MOVE;


void setup() {
  frameRate(FPS);
  size(1280, 850);
  surface.setResizable(true);
  
  createObjects();
  createViews();    // Note: views must be created AFTER objects (because they depend on their dimensions)
  setupCharts();
  //setupArduino();
    
  // Example moves
  Stand stand = new Stand();     // Standing
  Walk walk = new Walk();        // Straight walk
  Walk crab = new Walk(0.0, 20); // 20° Crab walk 
  Walk turn = new Walk(5);       // Turn at 5°/s
  //turn.set("ampl_osc", 0.0);
  
  planner = new MovePlanner();
  
  // Uncomment for a constant move
  //planner.addMove(walk);
  
  // Uncomment for a move sequence
  planner.addMove(stand, 3);
  planner.addMove(new LinearTransient(stand, turn), 3.0);
  planner.addMove(turn);
     
}


void draw() {  
  MOVE = planner.getMove(PHASE);    // Sets the current move
      
  for(int i=0; i<4; i++){  // For each leg
    legs[i].foot = MOVE.getFootPosition(i, PHASE);  // Trajectory planning
    legs[i].footTrajectory = MOVE.trajectories[i]; 
    legs[i].resolve();  // Inverse kinematics & CG
  }
  frame.resolve();  // Compute CG and stability triangles
  ground.resolve(MOVE, PHASE, DT);  // Compute ground position variation
  
  drawViews(); // Draw the views
  
  updateCharts(); // Update the charts
    
  drawArduino(); // Arduino interface
  
  updateTime(); // Update the time, and display it
}


void updateTime(){
  fill(0);
  text(nf(TIME,1,2)+" sec", 3, 15);
  text(MOVE.getName(), 80, 15);
  if(!PAUSE)
    TIME += 1.0 / FPS; 
  DT = TIME-TIME_1;
  float period = MOVE.getPeriod(PHASE);
  if(Float.isFinite(period)){
    if(period==0)
      println("Warning: period==0 must be prevented!");
    PHASE += DT / period;   // Increment of phase: dt / T
  }
  TIME_1 = TIME;
  if(RESET){
    RESET = false;
    PHASE = 0.0;
    TIME = 0.0;
    TIME_1 = 0.0;
  }
}

void createObjects(){
  frame = new Frame();
  ground = new Ground();
  legs[0] = new Leg(new PVector(0, 0, frame.getHeight()), false, false);
  legs[1] = new Leg(new PVector(frame.getLength(), 0, frame.getHeight()), false, true);
  legs[2] = new Leg(new PVector(0, frame.getWidth(), frame.getHeight()), true, false);
  legs[3] = new Leg(new PVector(frame.getLength(), frame.getWidth(), frame.getHeight()), true, true);
}

void computeCalibrationPoints(){
  for(int i = 0; i<3; i++){
    for(int j = 0; j<3; j++){
      PVector pt = new PVector(SCALE*(-20.0+i*20.0), SCALE*(-20.0+j*20.0), 0.0);
      for(int k=0; k<4; k++){
        Leg leg = legs[k];
        PVector foot = leg.slot.copy();
        if(leg.right){
          foot.add(0, leg.shoulderWidth, -frame.getHeight()).add(pt);
        }else{
          foot.add(0, -leg.shoulderWidth, -frame.getHeight()).add(pt);
        }
        leg.foot = foot;
        leg.resolve();
        println(degrees(leg.theta)+" "+degrees(leg.phi)+" "+degrees(leg.psi));
      }
    }  
  } 
}