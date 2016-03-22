import java.util.*;

PVector EX = new PVector(1,0,0);
PVector EY = new PVector(0,1,0);
PVector EZ = new PVector(0,0,1);

class Trajectory {
 
  List<Segment> segments = new ArrayList();
  float length_grd, length;
  
  void addSegment(PVector p1, PVector cp1, PVector cp2, PVector p2){
    addSegment(new Bezier(p1, cp1, cp2, p2));
  }
  
  void addSegment(PVector p1, PVector cp, PVector p2){
    addSegment(new Bezier2(p1, cp, p2));
  }
  
  void addSegment(PVector p1, PVector p2){
    addSegment(new Line(p1,p2));
  }
  
  void addSegment(PVector p1){
    addSegment(new Point(p1));
  }
  
  void addSegment(Segment s){
    segments.add(s);
    length += s.length;
    if(s.ground) length_grd += s.length;
  }
  
  void setGroundRatio(float k){
    if(length==0){
      segments.get(0).phase = 1.0;    // Case of degenerate trajectory
    }else if(length_grd==0){
      for(Segment s : segments)  // Set the phase for each segment, in function of the ratio (speed is assumed constant on respectively air and ground)
        s.phase = s.ground? k : (1 - k) * s.length / length;
    }else{
      for(Segment s : segments)  // Set the phase for each segment, in function of the ratio (speed is assumed constant on respectively air and ground)
        s.phase = s.length * (s.ground? k / length_grd : (1 - k) / (length - length_grd));
    }
  }
  
  PVector point(float phase){
    float p = 0.0;
    for(Segment s : segments){
      p += s.phase;
      if(phase <= p)
        return s.pointLin(1-(p-phase)/ s.phase);
    }
    return new PVector(0,0,0);
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

class Point extends Segment {
  PVector p;
  
  Point(PVector p){
    this.p = p;
    length = 0.0;
    ground = p.z==0;
  }
  
  PVector pointLin(float t){
    return p.copy();
  }
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

class Bezier2 extends Segment {
  
  final int STEPS = 10;   // Steps to discretize the curve
  PVector p1, v1, v2, cp;
  float[] lengths = new float[STEPS+1];
  
  Bezier2(PVector p1, PVector cp, PVector p2){
    this.p1 = p1;
    this.cp = cp;
    v1 = p1.copy().mult(2).add(cp.copy().mult(-4)).add(p2.copy().mult(2));
    v2 = p1.copy().mult(-2).add(cp.copy().mult(2));
    length = length();
    ground = p1.z == 0.0 && p2.z == 0.0 && cp.z == 0.0;
  }
  
  float length(){ 
    for(int i=0; i<STEPS; i++)
      lengths[i+1] = lengths[i] + dpoint(((float)i)/STEPS).mag() / STEPS;  // Discretize the trajectory
    return lengths[STEPS];
  }
  
  
  PVector point(float t){
    return v1.copy().mult(0.5*t*t).add(v2.copy().mult(t)).add(p1);
  }
  
  PVector dpoint(float t){
    return v1.copy().mult(t).add(v2);
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

class Bezier extends Bezier2 {
      
  Bezier(PVector p1, PVector cp1, PVector cp2, PVector p2){
    super(p1, cp1, p2);
    v1 = p1.copy().mult(-1).add(cp1.copy().mult(3)).add(cp2.copy().mult(-3)).add(p2);
    v2 = p1.copy().mult(3).add(cp1.copy().mult(-6)).add(cp2.copy().mult(3));
    length = length();
    ground = p1.z == 0.0 && p2.z == 0.0 && cp1.z == 0.0 && cp2.z == 0.0;
  }
    
  PVector point(float t){
    PVector v3 = p1.copy().mult(-3).add(cp.copy().mult(3));
    return v1.copy().mult(t*t*t).add(v2.copy().mult(t*t)).add(v3.copy().mult(t)).add(p1);
  }
  
  PVector dpoint(float t){
    PVector v3 = p1.copy().mult(-3).add(cp.copy().mult(3));
    return v1.copy().mult(3*t*t).add(v2.copy().mult(2*t)).add(v3);
  }
}

class Oscillation extends Segment {
  
  PVector p1, p2, v;
  float dphase, freq;
  
  Oscillation(PVector p1, PVector p2, float dphase, float freq){
    this.p1 = p1;
    this.p2 = p2;
    this.v = p2.copy().sub(p1);
    this.dphase = dphase;
    this.freq = freq;
  }
  
  Oscillation(PVector p1, PVector p2, float phase){
    this(p1, p2, phase, 1.0);
  }
  
  Oscillation(PVector p1, PVector p2){
    this(p1, p2, 0.0);
  }
  
  PVector pointLin(float phase){
    float t = 0.5 - 0.5 * cos((dphase + freq*phase) * 2 * PI);
    return p1.copy().add(v.copy().mult(t));
  }
  
  PVector speed(float phase){
    return v.copy().mult(-PI * freq * sin((dphase + freq*phase) * 2 * PI));
  }
}

class Rotation {
 
  float q0, q1, q2, q3;
  
  Rotation(float q0, float q1, float q2, float q3){
    set(q0, q1, q2, q3);
  }
  
  Rotation(PVector axis, float angle){
    float coeff = sin(-0.5 * angle) / axis.mag();
    set(cos(-0.5 * angle), coeff * axis.x, coeff * axis.y, coeff * axis.z);
  }
  
  void set(float q0, float q1, float q2, float q3){   
    this.q0 = q0; 
    this.q1 = q1;
    this.q2 = q2;
    this.q3 = q3; 
  }
  
  PVector rotate(PVector v){
    float s = q1 * v.x + q2 * v.y + q3 * v.z;
    return new PVector(2 * (q0 * (v.x * q0 - (q2 * v.z - q3 * v.y)) + s * q1) - v.x,
                        2 * (q0 * (v.y * q0 - (q3 * v.x - q1 * v.z)) + s * q2) - v.y,
                        2 * (q0 * (v.z * q0 - (q1 * v.y - q2 * v.x)) + s * q3) - v.z);
  }
  
  Rotation compose(Rotation r){
    return new Rotation(r.q0 * q0 - (r.q1 * q1 + r.q2 * q2 + r.q3 * q3),
                        r.q1 * q0 + r.q0 * q1 + (r.q2 * q3 - r.q3 * q2),
                        r.q2 * q0 + r.q0 * q2 + (r.q3 * q1 - r.q1 * q3),
                        r.q3 * q0 + r.q0 * q3 + (r.q1 * q2 - r.q2 * q1));
  }
  
}