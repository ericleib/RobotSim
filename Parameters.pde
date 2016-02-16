float SCALE = 3.0;  // pixels / mm

// Geometry (dimensions in mm)

float getFrameThickness(){  return SCALE*5.0; }

float getFrameWidth(){  return SCALE*40.0; }

float getFrameLength(){  return SCALE*100.0; }

float getFrameHeight(){ return getUpperLegLength() * 1.40; }

float getUpperLegLength(){ return SCALE*50.0; }

float getLowerLegLength(){ return SCALE*50.0; }

float getShoulderWidth(){ return SCALE*10.0; }

float getFrameCGx(){ return getFrameLength() * 0.50; }

float getFrameCGy(){ return getFrameWidth() * 0.50; }


// Kinematics

float getSpeed(){ return SCALE*speed; }

float getFootHeight(){ return SCALE*10.0; }

float getFootDy(){ return -SCALE*15.0; }

float getFootDr(){ return 0.0 * PI / 180; }

float getMovementPeriod(){ return period; } // s

float getGroundRatio(){ return 0.80; }

float getPhaseLeg(int i){ return new float[]{0.25, 0.00, 0.75, 0.50}[i]; } // % of period

// kinematics
Trajectory getFootTrajectory(){
  float k = getGroundRatio(); // Threshold at which the foot goes up
  float dx = 0.5 * k * getSpeed() * getMovementPeriod();  // Half longitudinal movement
  float h = getFootHeight();
  float dy = getFootDy();
  float dr = getFootDr();
  
  Trajectory t = new Trajectory();
  t.addSegment(new PVector(dx, dy + dx * tan(dr), 0.0), new PVector(0.0, dy, 0.0), new PVector(0.0, dy, 0.0), new PVector(-dx, dy - dx * tan(dr), 0.0));
  t.addSegment(new PVector(-dx, dy - dx * tan(dr), 0.0), new PVector(-dx, dy - dx * tan(dr), h), new PVector(-h, dy -h * tan(dr), h), new PVector(0, dy, h));
  t.addSegment(new PVector(0, dy, h), new PVector(h, dy + h * tan(dr), h), new PVector(h, dy + h * tan(dr), h), new PVector(dx, dy + dx * tan(dr), h));
  t.addSegment(new PVector(dx, dy + dx * tan(dr), h), new PVector(dx+h, dy + (dx+h) * tan(dr), h), new PVector(dx+h, dy + (dx+h) * tan(dr), 0), new PVector(dx, dy + dx * tan(dr), 0));
  t.setGroundRatio(getGroundRatio());
  return t;
}

// Style

void setFrameColor(){ stroke(50, 117, 200);}

void setGroundColor(){ stroke(176, 124, 82);}

void setRed(){ stroke(210, 38, 38);}

void setGreen(){ stroke(9, 169, 61); }