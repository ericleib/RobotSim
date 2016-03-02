
// Simulation parameters
int FPS = 50;    // Frames per second
float SCALE = 3.0;  // pixels / mm
float time = 0.0;  // Time (in seconds)
float time_1 = 0.0;  // Time - 1 (in seconds)
float phase = 0.0;  // General phase
boolean pause = false;  // Whether the simulation is paused or not
boolean reset = false;  // Flag to reset the simulation

// Objects in the simulation
Ground ground;
Frame frame;
Leg[] legs = new Leg[4];

// Current move of the robot
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
  
  //move = new Walk(0.0, 20);    // 20° Crab walk 
  //move = new Walk(0.0, 0);     // Straight walk
  move = new Walk(SCALE*1000);           // Turn
  // start = new Start(20.0);             // Start move from 0.0 to speed
  
  createSliders();
}


void draw() {  
  background(240);   // background color
      
  for(int i=0; i<4; i++){
    legs[i].foot = move.getFootPosition(i, phase);
    legs[i].footTrajectory = move.trajectories[i];
    legs[i].resolve();
  }
  frame.computeStabilityTriangles();
  
  drawViews();
  
  updateCharts();
    
  drawArduino();  
  
  updateTime(); 
}


void updateTime(){
  fill(0);
  text(nf(time,1,2)+" sec", 3, 15);
  text(move.name, 80, 15);
  if(!pause)
    time += 1.0 / FPS; 
  phase += (time-time_1) / move.getPeriod();   // Increment of phase: dt / T
  time_1 = time;
  if(reset){
    reset = false;
    time = 0.0;
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