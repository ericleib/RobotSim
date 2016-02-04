
// Variables changing over time

// Robot longitudinal speed
float speed = 20.0; // mm/s

// Foot variables
float[] phase = new float[4];
float[] footx = new float[4];
float[] footy = new float[4];
float[] footz = new float[4];

// Leg angles
float[] theta = new float[4]; // Shoulder angle
float[] phi = new float[4];   // Upper leg angle
float[] psi = new float[4];   // Lower leg angle