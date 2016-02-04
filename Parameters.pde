
float SCALE = 3.0;  // pixels / mm

// Geometry (dimensions in mm)

float getFrameThickness(){  return SCALE*5.0; }

float getFrameWidth(){  return SCALE*40; }

float getFrameLength(){  return SCALE*100; }

float getFrameHeight(){ return getUpperLegLength() * 1.4; }

float getUpperLegLength(){ return SCALE*50.0; }

float getLowerLegLength(){ return SCALE*50.0; }

float getFrameCGx(){ return getFrameLength() * 0.5; }

float getFrameCGy(){ return getFrameWidth() * 0.5; }


// Kinematics

float getTargetSpeed(){ return SCALE*speed; }

float getTargetFootHeight(){ return SCALE*10.0; }

float getMovementPeriod(){ return 1.8; } // s

float getMovementThreshold(){ return 0.9; }

float getPhaseLeg(int i){ return new float[]{0.2, 0.00, 0.7, 0.50}[i]; } // % of period

float get_x(float phase){   // Returns x coordinate of foot wrt body shoulder (phase given as percentage of period)
  float threshold = getMovementThreshold(); // Threshold at which the foot goes up
  float dx = threshold * getTargetSpeed() * getMovementPeriod();  // Movement of robot in one period: corresponds to full movement of foot
  if(phase < threshold){
    return dx/2 - dx * (phase/threshold); // The foot is going backward and slow wrt to the robot    
  } else {
    return -dx/2 * cos( PI * (phase - threshold) / (1.0 - threshold)); // The foot is going forward and fast wrt to the robot  
  }
}

float get_y(float phase){    // Returns y coordinate of foot wrt body shoulder (phase given as percentage of period)
  return 0.0;  
}

float get_z(float phase){    // Returns z coordinate of foot wrt body shoulder (phase given as percentage of period)
  float threshold = getMovementThreshold(); // Threshold at which the foot goes up
  float dx = threshold * getTargetSpeed() * getMovementPeriod();  // Movement of robot in one period: corresponds to full movement of foot
  if(phase < threshold){
    return 0; // The foot is going backward and slow wrt to the robot    
  } else {
    return dx/2 * sin( PI * (phase - threshold) / (1.0 - threshold)); // The foot is going forward and fast wrt to the robot  
  }
}

// Style

void setFrameColor(){ stroke(50, 117, 200);}

void setGroundColor(){ stroke(176, 124, 82);}

void setRed(){ stroke(210, 38, 38);}

void setGreen(){ stroke(9, 169, 61); }