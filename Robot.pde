
// Global variables
MyRobot ROBOT;
MovePlanner PLANNER;
Time TIME;
Views UI;

// Setup method (called once)
void setup() {
  size(1280, 850);
  surface.setResizable(true);
    
  TIME = new Time(50);  
  
  ROBOT = new MyRobot();
  
  UI = new Views(3.0);
  
  initPlanner();
  
  //setupArduino();
}

// Drawing method (called every step)
void draw() {  
  Move move = PLANNER.getMove(TIME.phase); // Sets the current move
  
  ROBOT.apply(move, TIME.phase);
  
  UI.draw(move.getName()); // Draw the views
        
  TIME.update(move.getPeriod(TIME.phase)); // Update the time
}



void initPlanner(){
   
  PLANNER = new MovePlanner();
  
  // Uncomment for a constant move
  //PLANNER.addMove(new Walk());
  
  // Test: piloted walk
  PLANNER.addMove(new PilotedMove(), 2.5);
  
  // Uncomment for a move sequence
  /*
  Walk stand = new Stand();
  PLANNER.addMove(stand, 3.0);
  
  Walk walk = new Walk();
  walk.set("dx", 30);
  walk.set("speed", 40);
  PLANNER.addMoveWithTransient(walk, 2.5,5.0);
  
  Walk walk_crab = new Walk( 0.0, 25.0);
  walk_crab.set("dx", 30);
  walk_crab.set("speed", 50);
  PLANNER.addMoveWithTransient(walk_crab, 2.0, 5.0);
  
  Walk turn = new Walk(-8.0, 0.0);
  turn.set("dx", 30);
  turn.set("speed", 50);
  PLANNER.addMoveWithTransient(turn, 3.0, 10.0);
  
  Walk turn2 = new Walk(-10.0, 0.0);
  turn2.set("speed", 5.0);
  turn2.set("dx", 5.0);
  turn2.set("dy", 40.0);
  PLANNER.addMoveWithTransient(turn2, 3.0, 10.0);
  
  Walk walk2 = new Walk();
  walk2.set("dx", 30);
  walk2.set("speed", 40);
  PLANNER.addMoveWithTransient(walk2, 3.0, 5.0);
  
  Walk stand2 = new Stand();
  PLANNER.addMoveWithTransient(stand2, 5.0, 0.0);
  */
}