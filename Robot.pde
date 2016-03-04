
// Simulation parameters
int FPS = 50;    // Frames per second
float SCALE = 3.0;  // pixels / mm
float time = 0.0;  // Time (in seconds)
float time_1 = 0.0;  // Time - 1 (in seconds)
float dt = 1.0 / FPS;
float phase = 0.0;  // General phase
boolean pause = false;  // Whether the simulation is paused or not
boolean reset = false;  // Flag to reset the simulation

// Objects in the simulation
Ground ground;
Frame frame;
Leg[] legs = new Leg[4];

// Current move of the robot
MovePlanner planner;
Move move;


void setup() {
  frameRate(FPS);
  size(1024, 720);
  surface.setResizable(true);
  smooth();
  
  createObjects();
  createViews();    // Note: views must be created AFTER objects (because they depend on their dimensions)
  setupCharts();
  setupArduino();
  
  // Example moves
  Stand stand = new Stand();     // Standing
  Walk walk = new Walk();        // Straight walk
  Walk crab = new Walk(0.0, 20); // 20° Crab walk 
  Walk turn = new Walk(5);       // Turn at 5°/s
  
  planner = new MovePlanner();
  
  // Uncomment for a constant move
  planner.constantMove = true;
  planner.addMove(walk);
  
  // Uncomment for a move sequence
  //planner.constantMove = false;
  //planner.addMove(stand, 5);
  //planner.addMove(new LinearTransient(stand, turn, 0.0, 0.0, 11.0));
  //planner.addMove(turn);
     
}


void draw() {  
  background(240);   // background color
  
  planner.setMove();    // Sets the current move
      
  for(int i=0; i<4; i++){  // For each leg
    legs[i].foot = move.getFootPosition(i, phase);  // Trajectory planning
    legs[i].footTrajectory = move.trajectories[i]; 
    legs[i].resolve();  // Inverse kinematics
  }
  frame.computeStabilityTriangles();  // Static stability
  
  drawViews(); // Draw the views
  
  updateCharts(); // Update the charts
    
  drawArduino(); // Arduino interface
  
  updateTime(); // Update the time, and display it
}


void updateTime(){
  fill(0);
  text(nf(time,1,2)+" sec", 3, 15);
  text(move.name, 80, 15);
  if(!pause)
    time += 1.0 / FPS; 
  dt = time-time_1;
  if(Float.isFinite(move.getPeriod())){
    if(move.getPeriod()==0)
      println("Warning: period==0 must be prevented!");
    phase += dt / move.getPeriod();   // Increment of phase: dt / T
  }
  time_1 = time;
  if(reset){
    planner.reset();
    reset = false;
    time = 0.0;
    time_1 = 0.0;
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