
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

float getMovementPeriod(){ return period; } // s

float getMovementThreshold(){ return 0.80; }

float getPhaseLeg(int i){ return new float[]{0.30, 0.00, 0.80, 0.50}[i]; } // % of period

// Circular kinematics
/*
Trajectory getFootTrajectory(){
  float threshold = getMovementThreshold(); // Threshold at which the foot goes up
  float dx = threshold * getTargetSpeed() * getMovementPeriod();  // Movement of robot in one period: corresponds to full movement of foot
  
  Trajectory t = new Trajectory();
  t.phases.add(0.0);
  t.phases.add(threshold);
  t.phases.add(1.0);
  t.segments_x.add(new Lin(dx/2, -dx/2));
  t.segments_x.add(new Cos(0.0, dx/2, PI, 0));
  t.segments_y.add(new Constant(0.0));
  t.segments_y.add(new Constant(0.0));
  t.segments_z.add(new Constant(0.0));
  t.segments_z.add(new Cos(0.0, dx/2, HALF_PI, -HALF_PI));
  return t;
}
*/

// Composed kinematics
Trajectory getFootTrajectory(){
  float threshold = getMovementThreshold(); // Threshold at which the foot goes up
  float airborne = 1.0 - threshold;
  float dx = threshold * getTargetSpeed() * getMovementPeriod();  // Movement of robot in one period: corresponds to full movement of foot
  float h = getTargetFootHeight();
  float t_arc = airborne * HALF_PI * h / (dx + (PI-1.0) * h);
  
  Trajectory t = new Trajectory();
  t.addSegment(threshold,       new Lin(dx/2, -dx/2),                  new Constant(0.0), new Constant(0.0));
  t.addSegment(threshold+t_arc, new Cos(-dx/2+h, -h, 0, -HALF_PI),     new Constant(0.0), new Cos(0.0, h, -HALF_PI, 0));
  t.addSegment(1.0-t_arc,       new Lin(-dx/2+h, dx/2),                new Constant(0.0), new Constant(h));
  t.addSegment(1.0,             new Cos(dx/2, h/2, HALF_PI, -HALF_PI), new Constant(0.0), new Cos(h/2, h/2, 0, PI));
    
  return t;
}

// Style

void setFrameColor(){ stroke(50, 117, 200);}

void setGroundColor(){ stroke(176, 124, 82);}

void setRed(){ stroke(210, 38, 38);}

void setGreen(){ stroke(9, 169, 61); }