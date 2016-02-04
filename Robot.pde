import controlP5.*;    // import controlP5 library
ControlP5 controlP5;   // controlP5 object

// Simulation parameters
float FPS = 50;
float time = 0.0;

void setup() {
  frameRate(FPS);
  size(640, 720);
  smooth();
  controlP5 = new ControlP5(this);
  Controller mySlider = controlP5.addSlider("speed",0.0,100.0,20.0,150,5,300,15);
  mySlider.getCaptionLabel().setColor(0);
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
  for(int i=0; i<100; i++){
    float x = offset_x + get_x(i/100.0);
    float y = get_y(i/100.0);
    float z = get_z(i/100.0);
    drawFoot(x,     offset_y - y,     z, ground_y, 1, true);
    drawFoot(x + l, offset_y - y,     z, ground_y, 1, true);
    drawFoot(x,     offset_y + w + y, z, ground_y, -1, true);
    drawFoot(x + l, offset_y + w + y, z, ground_y, -1, true);
  }
    
    
  // Draw the feet
  float ph = (time % getMovementPeriod()) / getMovementPeriod();
  for(int i=0; i<4; i++){
    phase[i] = (ph + getPhaseLeg(i)) % 1.0;
    footx[i] = offset_x + get_x(phase[i]);
    footy[i] = get_y(phase[i]);
    footz[i] = get_z(phase[i]);
    if(i==1 || i==3) footx[i] += l;
    if(i==0 || i==1){
      footy[i] = offset_y - footy[i];
      drawFoot(footx[i], footy[i], footz[i], ground_y, 1, false);
    }else{
      footy[i] = offset_y + w + footy[i];
      drawFoot(footx[i], footy[i], footz[i], ground_y, -1, false);
    }
  }
  
  
  
  // Draw the triangles
  boolean stable = false;
  float cgx = offset_x + getFrameCGx();
  float cgy = offset_y + getFrameCGy();
  for(int i=0; i<4; i++){
    float[][] points = new float[3][3];
    for(int j=0; j<3; j++){
      int k = j<i? j : j+1;
      points[j][0] = footx[k];
      points[j][1] = footy[k];
      points[j][2] = footz[k];
    }
    if(points[0][2]==0.0 && points[1][2]==0.0 && points[2][2]==0.0){
      if(pointInTriangle(points, cgx, cgy)){
        stable = true;
        setGreen();
      }else{
        setRed();
      }
      line(points[0][0], points[0][1], points[1][0], points[1][1]);
      line(points[1][0], points[1][1], points[2][0], points[2][1]);
      line(points[2][0], points[2][1], points[0][0], points[0][1]);
    }
  }
  

  // Draw the CG
  if(stable)
    setGreen();
  else
    setRed();
  drawCG(cgx, cgy);  // top view
  drawCG(cgx, offset_y_side + getFrameThickness() * 0.5);  // side view


  // Draw legs
  stroke(100);
  drawLeg(footx[0], footz[0], offset_x,   getFrameHeight(), ground_y);
  drawLeg(footx[1], footz[1], offset_x+l, getFrameHeight(), ground_y);
  drawLeg(footx[2], footz[2], offset_x,   getFrameHeight(), ground_y);
  drawLeg(footx[3], footz[3], offset_x+l, getFrameHeight(), ground_y);
  
  
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

void drawLeg(float x1, float z1, float x2, float z2, float offset_z){
  float r1 = getLowerLegLength();
  float r2 = getUpperLegLength();
  float d = sqrt((x1-x2)*(x1-x2) + (z2-z1)*(z2-z1));
  float a = (r1*r1 - r2*r2 + d*d) / (2 * d);
  float h = sqrt(r1*r1 - a*a);
  float x3 = x1 + a * (x2 - x1) / d;
  float z3 = z1 + a * (z2 - z1) / d;
  float x4 = x3 - h * (z2 - z1) / d;
  float z4 = z3 + h * (x2 - x1) / d;
  ellipse(x4, offset_z-z4, 6.0, 6.0);
  ellipse(x2, offset_z-z2, 6.0, 6.0);
  line(x1, offset_z-z1, x4, offset_z-z4);
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
  time += 1.0 / FPS; 
}

boolean pointInTriangle(float[][] points, float x, float y){
  float area = 0.5*(-points[1][1]*points[2][0] + points[0][1]*(-points[1][0] + points[2][0]) + points[0][0]*(points[1][1] - points[2][1]) + points[1][0]*points[2][1]);
  float s = 1/(2*area)*(points[0][1]*points[2][0] - points[0][0]*points[2][1] + (points[2][1] - points[0][1])*x + (points[0][0] - points[2][0])*y);
  float t = 1/(2*area)*(points[0][0]*points[1][1] - points[0][1]*points[1][0] + (points[0][1] - points[1][1])*x + (points[1][0] - points[0][0])*y);
  return 0 <= s && s<= 1 && 0 <= t && t <= 1 && s + t <= 1;
}