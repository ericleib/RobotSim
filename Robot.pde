
void setup() {
  frameRate(FPS);
  size(1024, 720);
  smooth();
  setupCharts();
  setupArduino();
}


void draw() {  
  background(240);   // background color
  strokeWeight(3);
  noFill();
  
  // Geometrical parameters of the robot
  float w = getFrameWidth();
  float l = getFrameLength();
  float t = getFrameThickness();
  float h = getFrameHeight();
  
  // Where to draw the different views
  float offset_x = 150, offset_y = 150;
  float offset_y_side = offset_y * 2 + w;
  float offset_x_front = offset_x + l + SCALE * 80.0;
  float ground_x = offset_x - SCALE * 30.0;
  float ground_x2 = offset_x + l + SCALE * 30.0;
  float ground_y = offset_y_side + t * 0.5 + h;
    
  // Coordinates of the slots of each leg
  slot[0] = new PVector(offset_x, offset_y, h);
  slot[1] = new PVector(offset_x + l, offset_y, h);
  slot[2] = new PVector(offset_x, offset_y + w, h);
  slot[3] = new PVector(offset_x + l, offset_y + w, h);
  
  
  // Draw the frame
  setFrameColor();
  // top view
  float margin = SCALE * 5.0;
  rect(offset_x, offset_y + margin, l, w-2*margin);
  line(offset_x, offset_y, offset_x, offset_y + w);
  line(offset_x + l, offset_y, offset_x + l, offset_y + w);
  // side view
  rect(offset_x, offset_y_side, l, t);
  // front view
  rect(offset_x_front+margin, offset_y_side, w-2*margin, t);
  line(offset_x_front, offset_y_side+0.5*t, offset_x_front + w, offset_y_side+0.5*t);
    
  // Draw the ground
  setGroundColor();
  line(ground_x, ground_y, ground_x2, ground_y);
  for(float slash_x = 15.0 + (-getSpeed() * time) % 15.0; slash_x <= ground_x2 - ground_x; slash_x += 15.0)
    line(ground_x+slash_x, ground_y, ground_x+slash_x - 15.0, ground_y + 15.0);
  
  
  // Draw the feet trajectories  
  Trajectory traj = getFootTrajectory();
  float npoints = FPS * getMovementPeriod();
  for(int i=0; i<npoints; i++){
    PVector pt = traj.point(i/npoints);
    for(int j=0; j<4; j++){
      if(j==2) pt.y = -pt.y;
      drawFoot(pt.copy().add(slot[j]).add(0,0,-h), offset_x_front-offset_x, ground_y, true);
    }
  }
    
    
  // Draw the feet
  float ph = (time % getMovementPeriod()) / getMovementPeriod();
  for(int i=0; i<4; i++){
    phase[i] = (ph + getPhaseLeg(i)) % 1.0;
    foot[i] = traj.point(phase[i]);
    if(i==2 || i==3) foot[i].y = -foot[i].y;
    foot[i].add(slot[i]).add(0,0,-h);
    drawFoot(foot[i], offset_x_front-offset_x, ground_y, false);
  }
  
  
  // Draw the triangles
  PVector cg = new PVector(offset_x + getFrameCGx(), offset_y + getFrameCGy());
  List<PVector[]> stable = new ArrayList(), unstable = new ArrayList();
  for(int i=0; i<4; i++){
    PVector[] points = new PVector[3];
    for(int j=0; j<3; j++){
      int k = j<i? j : j+1;
      points[j] = foot[k];
    }
    if(points[0].z==0.0 && points[1].z==0.0 && points[2].z==0.0){
      if(pointInTriangle(points, cg))
        stable.add(points);
      else
        unstable.add(points);
    }
  }
  strokeWeight(1.5);
  // draw unstable triangles (red)
  setRed();
  for(PVector[] points : unstable)
    triangle(points[0].x, points[0].y, points[1].x, points[1].y, points[2].x, points[2].y);
  // draw stable triangles (green)
  setGreen();
  for(PVector[] points : stable)
    triangle(points[0].x, points[0].y, points[1].x, points[1].y, points[2].x, points[2].y);
  strokeWeight(3);
  
  // Draw the CG
  if(stable.size()>0)
    setGreen();
  else
    setRed();
  drawCG(cg);  // top view
  drawCG(new PVector(cg.x, offset_y_side + getFrameThickness() * 0.5));  // side view


  // Draw legs
  stroke(100);
  for(int i=0; i<4; i++){
    computeShoulder(i);
    computeKnee(i);
    drawLeg(i, offset_x_front-offset_x, ground_y);
    computeAngles(i);
  }
  
  
  // Update charts
  updateCharts();
  
  
  drawArduino();
  
  
  drawTime();
}


// Drawing functions

void drawFoot(PVector foot, float offset_x, float gndy, boolean point){
  if(foot.z==0.0) setGreen(); else setRed();
  if(point){
    point(foot.x, foot.y + foot.z);
    point(foot.x, gndy - foot.z);
    point(offset_x+foot.y, gndy - foot.z);
  }else{
    ellipse(foot.x, foot.y + foot.z, 6.0, 6.0);
    ellipse(foot.x, gndy - foot.z, 6.0, 6.0);
    ellipse(offset_x+foot.y, gndy - foot.z, 6.0, 6.0);
  }
}

void drawLeg(int i, float offset_x, float offset_z){
  // top view
  ellipse(shoulder[i].x, shoulder[i].y, 6.0, 6.0);
  ellipse(knee[i].x, knee[i].y, 6.0, 6.0);
  ellipse(slot[i].x, slot[i].y, 6.0, 6.0);
  line(foot[i].x, foot[i].y, knee[i].x, knee[i].y);
  line(knee[i].x, knee[i].y, shoulder[i].x, shoulder[i].y);
  line(slot[i].x, slot[i].y, shoulder[i].x, shoulder[i].y);
  
  // side view
  ellipse(shoulder[i].x, offset_z-shoulder[i].z, 6.0, 6.0);
  ellipse(knee[i].x, offset_z-knee[i].z, 6.0, 6.0);
  ellipse(slot[i].x, offset_z-slot[i].z, 6.0, 6.0);
  line(foot[i].x, offset_z-foot[i].z, knee[i].x, offset_z-knee[i].z);
  line(knee[i].x, offset_z-knee[i].z, shoulder[i].x, offset_z-shoulder[i].z);
  line(slot[i].x, offset_z-slot[i].z, shoulder[i].x, offset_z-shoulder[i].z);
  
  // front view
  ellipse(offset_x+shoulder[i].y, offset_z-shoulder[i].z, 6.0, 6.0);
  ellipse(offset_x+knee[i].y, offset_z-knee[i].z, 6.0, 6.0);
  ellipse(offset_x+slot[i].y, offset_z-slot[i].z, 6.0, 6.0);
  line(offset_x+foot[i].y, offset_z-foot[i].z, offset_x+knee[i].y, offset_z-knee[i].z);
  line(offset_x+knee[i].y, offset_z-knee[i].z, offset_x+shoulder[i].y, offset_z-shoulder[i].z);
  line(offset_x+slot[i].y, offset_z-slot[i].z, offset_x+shoulder[i].y, offset_z-shoulder[i].z);
}

void drawCG(PVector cg){
  strokeWeight(1);
  ellipse(cg.x, cg.y, 10, 10);
  fill(g.strokeColor);
  arc(cg.x, cg.y, 10, 10, 0, HALF_PI);
  arc(cg.x, cg.y, 10, 10, PI, PI + HALF_PI);   
  noFill();
  strokeWeight(3);
}

void drawTime(){
  fill(0);
  text(nf(time,1,2)+" sec", 1, 15);
  if(!pause)
    time += 1.0 / FPS; 
}


// Calculation functions

void computeShoulder(int i){
  PVector slot_foot = foot[i].copy().sub(slot[i]);  // Vector from slot to foot
  slot_foot.x = 0.0;                                // Project on plan (y,z)
  float d1 = getShoulderWidth();
  float a1 = acos(d1 / slot_foot.mag()); // always positive 
  float a2 = atan((slot[i].z - foot[i].z) / abs(foot[i].y-slot[i].y)); // always positive
  shoulder[i] = slot[i].copy().add(0, d1*cos(a1-a2)*(i>=2? 1 : -1), d1*sin(a1-a2));
}

void computeKnee(int i){
  float r1 = getLowerLegLength();
  float r2 = getUpperLegLength();
  float d = foot[i].dist(shoulder[i]);
  float a = (r1*r1 - r2*r2 + d*d) / (2 * d);
  float h = sqrt(r1*r1 - a*a);
  PVector pt = foot[i].copy().add(shoulder[i].copy().sub(foot[i]).mult(a/d));
  PVector normal = shoulder[i].copy().sub(slot[i]).normalize().mult((i>=2? 1 : -1));
  knee[i] = pt.add(shoulder[i].copy().sub(foot[i]).cross(normal).mult(h/d));
}

void computeAngles(int i){
  float theta_ = theta[i];
  float phi_ = phi[i];
  float psi_ = psi[i];
  theta[i] = degrees( atan((shoulder[i].z-slot[i].z) / (shoulder[i].y-slot[i].y) ));
  phi[i] = degrees( PVector.angleBetween(new PVector(-1,0,0), knee[i].copy().sub(shoulder[i])));
  psi[i] = degrees( PVector.angleBetween(knee[i].copy().sub(shoulder[i]), foot[i].copy().sub(knee[i])));
  thetad[i] = (theta[i] - theta_)*FPS;
  phid[i] = (phi[i] - phi_)*FPS;
  psid[i] = (psi[i] - psi_)*FPS;
}

// Checks if a point is inside a triangle
boolean pointInTriangle(PVector[] points, PVector pt){
  float area = 0.5*(-points[1].y*points[2].x + points[0].y*(-points[1].x + points[2].x) + points[0].x*(points[1].y - points[2].y) + points[1].x*points[2].y);
  float s = 1/(2*area)*(points[0].y*points[2].x - points[0].x*points[2].y + (points[2].y - points[0].y)*pt.x + (points[0].x - points[2].x)*pt.y);
  float t = 1/(2*area)*(points[0].x*points[1].y - points[0].y*points[1].x + (points[0].y - points[1].y)*pt.x + (points[1].x - points[0].x)*pt.y);
  return 0 <= s && s<= 1 && 0 <= t && t <= 1 && s + t <= 1;
}