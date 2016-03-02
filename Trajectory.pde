import java.util.*;

class Trajectory {
 
  List<Segment> segments = new ArrayList();
  float length_grd, length;
  
  void addSegment(PVector p1, PVector cp1, PVector cp2, PVector p2){
    addSegment(new Bezier(p1, cp1, cp2, p2));
  }
  
  void addSegment(PVector p1, PVector cp, PVector p2){
    addSegment(p1, cp, cp, p2);
  }
  
  void addSegment(PVector p1, PVector p2){
    addSegment(new Line(p1,p2));
  }
  
  void addSegment(Segment s){
    segments.add(s);
    length += s.length;
    if(s.ground) length_grd += s.length;
  }
  
  void setGroundRatio(float k){
    for(Segment s : segments)  // Set the phase for each segment, in function of the ratio (speed is assumed constant on respectively air and ground)
      s.phase = s.length * (s.ground? k / length_grd : (1 - k) / (length - length_grd));
  }
  
  PVector point(float phase){
    float p = 0.0;
    for(Segment s : segments){
      p += s.phase;
      if(phase <= p)
        return s.pointLin(1-(p-phase)/ s.phase);
    }
    return null;
  }
  
  String toString(){
    String s = "";
    for(Segment seg : segments)
      s += seg.ground + " " + seg.phase + " " + seg.length+"; ";
    return s;
  }
}

abstract class Segment {
  
  float length, phase;
  boolean ground;
  
  abstract PVector pointLin(float phase);
}

class Line extends Segment {
  
  PVector v1, v2;
  
  Line(PVector v1, PVector v2){
    this.v1 = v1;
    this.v2 = v2;
    length = v1.dist(v2);
    ground = v1.z == 0.0 && v2.z == 0.0;
  }
  
  PVector pointLin(float t){
    return v1.copy().lerp(v2,t);
  }
}

class Bezier extends Segment {
  
  final int STEPS = 100;   // Steps to discretize the curve
  
  PVector p1, v1, v2, v3;
  float[] lengths = new float[STEPS+1];
  
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
}