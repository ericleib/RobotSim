
// Variables changing over time

// Robot longitudinal speed
float speed = 20.0; // mm/s

// Leg angles
float[] phase = new float[4];
float[] footx = new float[4];
float[] footy = new float[4];
float[] footz = new float[4];

float theta1 = 0.0,  theta2 = 0.0,  theta3 = 0.0, theta4 = 0.0; // Shoulder angle
float phi1 = 0.0,  phi2 = 0.0,  phi3 = 0.0, phi4 = 0.0;         // Upper leg angle
float psi1 = 0.0,  psi2 = 0.0,  psi3 = 0.0, psi4 = 0.0;         // Lower leg angle