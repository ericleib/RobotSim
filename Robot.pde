import controlP5.*;    // import controlP5 library
import java.util.*;
ControlP5 controlP5;   // controlP5 object

// Simulation parameters
float FPS = 50;
float time = 0.0;

boolean pause = false;

void setup() {
  frameRate(FPS);
  size(640, 720);
  smooth();
  controlP5 = new ControlP5(this);
  Controller speedSlider = controlP5.addSlider("speed",0.0,100.0,20.0,150,5,300,15);
  speedSlider.getCaptionLabel().setColor(0);
  Controller periodSlider = controlP5.addSlider("period",0.0,10,3.0,150,22,300,15);
  periodSlider.getCaptionLabel().setColor(0);
  controlP5.addButton("pause").setPosition(500,5).setSize(35,15);
  controlP5.addButton("next").setPosition(538,5).setSize(35,15);
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
    float[] pt = traj.interpolate(i/npoints);
    drawFoot(offset_x + pt[0],     offset_y - pt[1],     pt[2], ground_y, 1, true);
    drawFoot(offset_x + pt[0] + l, offset_y - pt[1],     pt[2], ground_y, 1, true);
    drawFoot(offset_x + pt[0],     offset_y + w + pt[1], pt[2], ground_y, -1, true);
    drawFoot(offset_x + pt[0] + l, offset_y + w + pt[1], pt[2], ground_y, -1, true);
  }
    
    
  // Draw the feet
  float ph = (time % getMovementPeriod()) / getMovementPeriod();
  for(int i=0; i<4; i++){
    phase[i] = (ph + getPhaseLeg(i)) % 1.0;
    foot[i] = traj.interpolate(phase[i]);
    foot[i][0] += offset_x;
    if(i==1 || i==3) foot[i][0] += l;
    if(i==0 || i==1){
      foot[i][1] = offset_y - foot[i][1];
      drawFoot(foot[i][0], foot[i][1], foot[i][2], ground_y, 1, false);
    }else{
      foot[i][1] = offset_y + w + foot[i][1];
      drawFoot(foot[i][0], foot[i][1], foot[i][2], ground_y, -1, false);
    }
  }
  
  
  // Draw the triangles
  float cgx = offset_x + getFrameCGx();
  float cgy = offset_y + getFrameCGy();
  List<float[][]> stable = new ArrayList(), unstable = new ArrayList();
  for(int i=0; i<4; i++){
    float[][] points = new float[3][3];
    for(int j=0; j<3; j++){
      int k = j<i? j : j+1;
      points[j] = foot[k];
    }
    if(points[0][2]==0.0 && points[1][2]==0.0 && points[2][2]==0.0){
      if(pointInTriangle(points, cgx, cgy))
        stable.add(points);
      else
        unstable.add(points);
    }
  }
  // draw unstable triangles (red)
  setRed();
  for(float[][] points : unstable)
    triangle(points[0][0], points[0][1], points[1][0], points[1][1], points[2][0], points[2][1]);
  // draw stable triangles (green)
  setGreen();
  for(float[][] points : stable)
    triangle(points[0][0], points[0][1], points[1][0], points[1][1], points[2][0], points[2][1]);
  
  
  // Draw the CG
  if(stable.size()>0)
    setGreen();
  else
    setRed();
  drawCG(cgx, cgy);  // top view
  drawCG(cgx, offset_y_side + getFrameThickness() * 0.5);  // side view


  // Draw legs
  stroke(100);
  drawLeg(foot[0], offset_x,   getFrameHeight(), ground_y);
  drawLeg(foot[1], offset_x+l, getFrameHeight(), ground_y);
  drawLeg(foot[2], offset_x,   getFrameHeight(), ground_y);
  drawLeg(foot[3], offset_x+l, getFrameHeight(), ground_y);
  
  
  drawTime();
}

void drawFoot(float x, float y, float z, float gndy, float side, boolean point){
  if(z==0.0)
    setGreen();
  else
    setRed();
  if(point){
    point(x, y - side * z);
    point(x, gndy - z);
  }else{
    ellipse(x, y - side * z, 6.0, 6.0);
    ellipse(x, gndy - z, 6.0, 6.0);
  }
}

void drawLeg(float[] pt, float x2, float z2, float offset_z){
  float r1 = getLowerLegLength();
  float r2 = getUpperLegLength();
  float d = sqrt((pt[0]-x2)*(pt[0]-x2) + (z2-pt[2])*(z2-pt[2]));
  float a = (r1*r1 - r2*r2 + d*d) / (2 * d);
  float h = sqrt(r1*r1 - a*a);
  float x3 = pt[0] + a * (x2 - pt[0]) / d;
  float z3 = pt[2] + a * (z2 - pt[2]) / d;
  float x4 = x3 - h * (z2 - pt[2]) / d;
  float z4 = z3 + h * (x2 - pt[0]) / d;
  ellipse(x4, offset_z-z4, 6.0, 6.0);
  ellipse(x2, offset_z-z2, 6.0, 6.0);
  line(pt[0], offset_z-pt[2], x4, offset_z-z4);
  line(x4, offset_z-z4, x2, offset_z-z2);
}

void drawCG(float x, float y){
  strokeWeight(1);
  ellipse(x, y, 10, 10);
  fill(g.strokeColor);
  arc(x, y, 10, 10, 0, HALF_PI);
  arc(x, y, 10, 10, PI, PI + HALF_PI);   
  noFill();
  strokeWeight(3);
}

void drawTime(){
  fill(0);
  text(nf(time,1,2)+" sec", 1, 15);
  if(!pause)
    time += 1.0 / FPS; 
}


boolean pointInTriangle(float[][] points, float x, float y){
  float area = 0.5*(-points[1][1]*points[2][0] + points[0][1]*(-points[1][0] + points[2][0]) + points[0][0]*(points[1][1] - points[2][1]) + points[1][0]*points[2][1]);
  float s = 1/(2*area)*(points[0][1]*points[2][0] - points[0][0]*points[2][1] + (points[2][1] - points[0][1])*x + (points[0][0] - points[2][0])*y);
  float t = 1/(2*area)*(points[0][0]*points[1][1] - points[0][1]*points[1][0] + (points[0][1] - points[1][1])*x + (points[1][0] - points[0][0])*y);
  return 0 <= s && s<= 1 && 0 <= t && t <= 1 && s + t <= 1;
}


public void controlEvent(ControlEvent e) {
  if(e.getName()=="pause") pause = !pause;
  if(e.getName()=="next")
    if(!pause)
      pause = true;
    else
      time += 1.0 / FPS;
}