
class Frame extends Drawable {
  
  // Stable and unstable triangles
  List<PVector[]> stable = new ArrayList(), unstable = new ArrayList();
  
    
  float getThickness(){  return SCALE*5.0; }
  
  float getWidth(){  return SCALE*40.0; }
  
  float getLength(){  return SCALE*100.0; }
  
  float getHeight(){ return SCALE*50 * 1.40; }
  
  PVector getCG(){ return new PVector(getLength() * 0.50, getWidth() * 0.50, getHeight()); }
  
  // Calculate the triangles formed by the legs, and check if the CG is inside
  void computeStabilityTriangles(){
    stable.clear(); unstable.clear();
    for(int i=0; i<4; i++){
      PVector[] points = new PVector[3];
      for(int j=0; j<3; j++){
        int k = j<i? j : j+1;
        points[j] = legs[k].foot;
      }
      if(points[0].z==0.0 && points[1].z==0.0 && points[2].z==0.0){
        if(pointInTriangle(points, getCG()))
          stable.add(points);
        else
          unstable.add(points);
      }
    }
  }

  // Checks if a point is inside a triangle
  boolean pointInTriangle(PVector[] points, PVector pt){
    float area = 0.5*(-points[1].y*points[2].x + points[0].y*(-points[1].x + points[2].x) + points[0].x*(points[1].y - points[2].y) + points[1].x*points[2].y);
    float s = 1/(2*area)*(points[0].y*points[2].x - points[0].x*points[2].y + (points[2].y - points[0].y)*pt.x + (points[0].x - points[2].x)*pt.y);
    float t = 1/(2*area)*(points[0].x*points[1].y - points[0].y*points[1].x + (points[0].y - points[1].y)*pt.x + (points[1].x - points[0].x)*pt.y);
    return 0 <= s && s<= 1 && 0 <= t && t <= 1 && s + t <= 1;
  }
  
  void draw(View v){
    v.stroke(50, 117, 200);
    
    v.fill(255,255,255,128);    
    float margin = SCALE * 5.0;
    PVector corner1 = new PVector(0.0, margin, getHeight()+0.5*getThickness());
    PVector corner2 = new PVector(getLength(), getWidth()-margin, getHeight()-0.5*getThickness());
    PVector l1 = new PVector(0.0, 0.0, getHeight());
    PVector l2 = new PVector(0.0, getWidth(), getHeight());
    v.rect(corner1, corner2);
    v.line(l1, l2);
    v.line(l1.add(getLength(),0,0), l2.add(getLength(),0,0));    
    v.noFill();
    
    if(v==TOP){
      
      v.strokeWeight(1.5);
      // draw unstable triangles (red)
      v.setRed();
      for(PVector[] points : unstable)
        v.triangle(points[0], points[1], points[2]);
      // draw stable triangles (green)
      v.setGreen();
      for(PVector[] points : stable)
        v.triangle(points[0], points[1], points[2]);
      v.strokeWeight(3);
      
    }
    
    if(stable.size()>0)
      v.setGreen();
    else
      v.setRed();
    v.cg(getCG());
  }
  
}