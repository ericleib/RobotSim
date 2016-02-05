
// Variables changing over time

// Robot longitudinal speed
float speed = 20.0; // mm/s
float period = 3.0; // s

// Phase of each leg
float[] phase = new float[4];

// Foot coordinates
float[][] foot = new float[4][];

// Knee coordinates
float[][] knee = new float[4][];

// Leg angles
float[] theta = new float[4]; // Shoulder angle
float[] phi = new float[4];   // Upper leg angle
float[] psi = new float[4];   // Lower leg angle