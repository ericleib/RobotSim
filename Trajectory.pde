import java.util.*;

class Trajectory {
 
  List<Bezier> beziers = new ArrayList();
  float length_grd, length_air, length;
  
  void addSegment(PVector p1, PVector cp1, PVector cp2, PVector p2){
    Bezier b = new Bezier(p1, cp1, cp2, p2);
    beziers.add(b);
    length += b.length;
  }
  
  void setGroundRatio(float k){
    length_grd = 0.0;
    length_air = 0.0;
    for(Bezier b : beziers){  // Calculate length of ground and air segments
      if(b.ground) length_grd += b.length;
      else length_air += b.length;
    }                          // Set the phase for each segment, in function of the ratio (speed is assumed constant on respectively air and ground)
    for(Bezier b : beziers)
      b.phase = b.length * (b.ground? k / length_grd : (1 - k) / length_air);
    length = length_grd + length_air;
  }
  
  PVector point(float phase){
    float p = 0.0;
    for( Bezier b : beziers ){
      p += b.phase;
      if(phase <= p)
        return b.pointLin(1-(p-phase)/ b.phase);
    }
    return null;
  }
  
  String toString(){
    String s = "";
    for(Bezier b : beziers)
      s += b.ground + " " + b.phase + " " + b.length+"; ";
    return s;
  }
}



class Bezier {
  
  final int STEPS = 100;   // Steps to discretize the curve
  
  PVector p1, v1, v2, v3;
  float[] lengths = new float[STEPS+1];
  float length, phase;
  boolean ground;
  
  Bezier(PVector p1, PVector cp1, PVector cp2, PVector p2){
    this.p1 = p1;
    v1 = p1.copy().mult(-1).add(cp1.copy().mult(3)).add(cp2.copy().mult(-3)).add(p2);
    v2 = p1.copy().mult(3).add(cp1.copy().mult(-6)).add(cp2.copy().mult(3));
    v3 = p1.copy().mult(-3).add(cp1.copy().mult(3));
    length = length();
    ground = p1.z == 0.0 && p2.z == 0.0 && cp1.z == 0.0 && cp2.z == 0.0;
  }
  
  float length(){ 
    for(int i=0; i<STEPS; i++)
      lengths[i+1] = lengths[i] + dpoint(((float)i)/STEPS).mag() / STEPS;  // Discretize the trajectory
    return lengths[STEPS];
  }
  
  PVector point(float t){
    return v1.copy().mult(t*t*t).add(v2.copy().mult(t*t)).add(v3.copy().mult(t)).add(p1);
  }
  
  PVector dpoint(float t){
    return v1.copy().mult(3*t*t).add(v2.copy().mult(2*t)).add(v3);
  }
  
  PVector pointLin(float t){
    float l = t * length;   // length that must be walked on the bezier
    int i = Arrays.binarySearch(lengths, l);
    //println(lengths[STEPS-1]+" "+l+" "+lengths[STEPS]+" "+i);
    if( i >= 0 )
      return point(((float)i) / STEPS);
    else{
      i = - i - 2; // insertion point
      return point( (i + (l-lengths[i])/(lengths[i+1]-lengths[i])) / STEPS );  // Interpolate in the bezier
    }   
  }
  
  void draw(){
    for(int i=0; i<=STEPS; i++){
      PVector p = point(((float)i)/STEPS);
      ellipse(p.x, p.y, 3, 3);
    }
  }
  
  void drawLin(){
    for(int i=0; i<=STEPS; i++){
      PVector p = pointLin(((float)i)/STEPS);
      ellipse(p.x, p.y, 3, 3);
    }
  }
}