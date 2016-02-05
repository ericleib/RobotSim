
// Variables changing over time

// Robot longitudinal speed
float speed = 20.0; // mm/s
float period = 3.0; // s

// Phase of each leg
float[] phase = new float[4];

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