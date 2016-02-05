import controlP5.*;    // import controlP5 library
import java.util.*;

// Simulation parameters
float FPS = 50;
float time = 0.0;
boolean pause = false;

ControlP5 cp5;   // controlP5 object
Chart phiChart, psiChart, phidChart, psidChart;

void setup() {
  frameRate(FPS);
  size(1024, 720);
  smooth();
  cp5 = new ControlP5(this);
  Controller speedSlider = cp5.addSlider("speed",0.0,100.0,20.0,150,5,300,15);
  speedSlider.getCaptionLabel().setColor(0);
  Controller periodSlider = cp5.addSlider("period",0.0,10,3.0,150,22,300,15);
  periodSlider.getCaptionLabel().setColor(0);
  cp5.addButton("pause").setPosition(500,5).setSize(35,15);
  cp5.addButton("next").setPosition(538,5).setSize(35,15);
  phiChart = makeChart("phi", 0, 0, 180);
  psiChart = makeChart("psi", 1, 0, 180);
  phidChart = makeChart("phid", 2, -60, 60);
  psidChart = makeChart("psid", 3, -60, 60);
}


void draw() {  
  background(240);   // background color
  strokeWeight(3);
  noFill();
  
  float w = getFrameWidth();
  float l = getFrameLength();
  float t = getFrameThickness();
  float h = getFrameHeight();
  
  float offset_x = 150, offset_y = 150;
  float offset_y_side = offset_y * 2 + w;
  float ground_x = offset_x - SCALE * 30.0;
  float ground_x2 = offset_x + l + SCALE * 30.0;
  float ground_y = offset_y_side + t * 0.5 + h;
  
  shoulder[0] = new PVector(offset_x, offset_y, h);
  shoulder[1] = new PVector(offset_x + l, offset_y, h);
  shoulder[2] = new PVector(offset_x, offset_y + w, h);
  shoulder[3] = new PVector(offset_x + l, offset_y + w, h);
  
  // Draw the frame
  setFrameColor();
  // top view
  float margin = SCALE * 5.0;
  rect(offset_x, offset_y +  margin, l, w-2*margin);
  line(offset_x, offset_y, offset_x, offset_y + w);
  line(offset_x + l, offset_y, offset_x + l, offset_y + w);
  // side view
  rect(offset_x, offset_y_side, getFrameLength(), getFrameThickness());
  
    
  // Draw the ground
  setGroundColor();
  line(ground_x, ground_y, ground_x2, ground_y);
  for(float slash_x = 15.0 + (-getTargetSpeed() * time) % 15.0; slash_x <= ground_x2 - ground_x; slash_x += 15.0)
    line(ground_x+slash_x, ground_y, ground_x+slash_x - 15.0, ground_y + 15.0);
  
  
  // Draw the feet trajectories  
  Trajectory traj = getFootTrajectory();
  float npoints = FPS * getMovementPeriod();
  for(int i=0; i<npoints; i++){
    PVector pt = traj.interpolate(i/npoints);
    for(int j=0; j<4; j++){
      if(j==2 || j==3) pt.y = -pt.y;
      drawFoot(PVector.add(pt,shoulder[j]).add(0,0,-h), ground_y, true);
    }
  }
    
    
  // Draw the feet
  float ph = (time % getMovementPeriod()) / getMovementPeriod();
  for(int i=0; i<4; i++){
    phase[i] = (ph + getPhaseLeg(i)) % 1.0;
    foot[i] = traj.interpolate(phase[i]);
    if(i==2 || i==3) foot[i].y = -foot[i].y;
    foot[i].add(shoulder[i]).add(0,0,-h);
    drawFoot(foot[i], ground_y, false);
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
  // draw unstable triangles (red)
  setRed();
  for(PVector[] points : unstable)
    triangle(points[0].x, points[0].y, points[1].x, points[1].y, points[2].x, points[2].y);
  // draw stable triangles (green)
  setGreen();
  for(PVector[] points : stable)
    triangle(points[0].x, points[0].y, points[1].x, points[1].y, points[2].x, points[2].y);
  
  
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
    knee[i] = computeKnee(foot[i], shoulder[i]);
    drawLeg(foot[i], knee[i], shoulder[i], ground_y);
  }
  
  
  // Compute angles
  for(int i=0; i<4; i++){
    float theta_ = theta[i];
    float phi_ = phi[i];
    float psi_ = psi[i];
    theta[i] = 180.0/PI * atan((foot[i].y-shoulder[i].y) / h);
    phi[i] = 180.0/PI * PVector.angleBetween(new PVector(1,0,0), PVector.sub(knee[i], shoulder[i]));
    psi[i] = 180.0/PI * PVector.angleBetween(PVector.sub(knee[i], shoulder[i]), PVector.sub(foot[i], knee[i]));
    thetad[i] = (theta[i] - theta_)*FPS;
    phid[i] = (phi[i] - phi_)*FPS;
    psid[i] = (psi[i] - psi_)*FPS;
  }
  phiChart.push("phi", phi[0]);
  psiChart.push("psi", psi[0]);
  phidChart.push("phid", phid[0]);
  psidChart.push("psid", psid[0]);
  
  
  drawTime();
}

void drawFoot(PVector foot, float gndy, boolean point){
  if(foot.z==0.0) setGreen(); else setRed();
  if(point){
    point(foot.x, foot.y + foot.z);
    point(foot.x, gndy - foot.z);
  }else{
    ellipse(foot.x, foot.y + foot.z, 6.0, 6.0);
    ellipse(foot.x, gndy - foot.z, 6.0, 6.0);
  }
}

void drawLeg(PVector foot, PVector knee, PVector shoulder, float offset_z){
  ellipse(knee.x, offset_z-knee.z, 6.0, 6.0);
  ellipse(shoulder.x, offset_z-shoulder.z, 6.0, 6.0);
  line(foot.x, offset_z-foot.z, knee.x, offset_z-knee.z);
  line(knee.x, offset_z-knee.z, shoulder.x, offset_z-shoulder.z);
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


PVector computeKnee(PVector foot, PVector shoulder){
  float r1 = getLowerLegLength();
  float r2 = getUpperLegLength();
  float d = foot.dist(shoulder);
  float a = (r1*r1 - r2*r2 + d*d) / (2 * d);
  float h = sqrt(r1*r1 - a*a);
  PVector pt = PVector.add(foot, PVector.sub(shoulder,foot).mult(a/d));
  PVector knee = pt.add(PVector.sub(shoulder,foot).cross(new PVector(0,1,0)).mult(h/d));
  return knee;
}

boolean pointInTriangle(PVector[] points, PVector pt){
  float area = 0.5*(-points[1].y*points[2].x + points[0].y*(-points[1].x + points[2].x) + points[0].x*(points[1].y - points[2].y) + points[1].x*points[2].y);
  float s = 1/(2*area)*(points[0].y*points[2].x - points[0].x*points[2].y + (points[2].y - points[0].y)*pt.x + (points[0].x - points[2].x)*pt.y);
  float t = 1/(2*area)*(points[0].x*points[1].y - points[0].y*points[1].x + (points[0].y - points[1].y)*pt.x + (points[1].x - points[0].x)*pt.y);
  return 0 <= s && s<= 1 && 0 <= t && t <= 1 && s + t <= 1;
}

Chart makeChart(String name, int n, float min, float max){
  return cp5.addChart(name)
               .setPosition(620, 5 + n * 170)
               .setSize(400, 150)
               .setRange(min, max)
               .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
               .setStrokeWeight(1.5)
               .setColorCaptionLabel(color(40))
               .addDataSet(name)
               ;
}

void controlEvent(ControlEvent e) {
  if(e.getName()=="pause") pause = !pause;
  if(e.getName()=="next")
    if(!pause)
      pause = true;
    else
      time += 1.0 / FPS;
}