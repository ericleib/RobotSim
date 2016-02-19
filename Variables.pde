
// Simulation parameters
float FPS = 50;    // Frames per second
float time = 0.0;  // Time (in seconds)
boolean pause = false;  // Whether the simulation is paused or not

// Robot longitudinal speed
float speed = 20.0; // mm/s
float dx = 20.0; //mm
//float period = 3.0; // s

// Phase of each leg
float[] phase = new float[4];

// Slot coordinates (support for the shoulder)
PVector[] slot = new PVector[4];

// Shoulder coordinates
PVector[] shoulder = new PVector[4];

// Foot coordinates
PVector[] foot = new PVector[4];

// Knee coordinates
PVector[] knee = new PVector[4];

// Leg angles
float[] theta = new float[4]; // Shoulder angle
float[] phi = new float[4];   // Upper leg angle
float[] psi = new float[4];   // Lower leg angle
float[] thetad = new float[4]; // Shoulder angle
float[] phid = new float[4];   // Upper leg angle
float[] psid = new float[4];   // Lower leg angle