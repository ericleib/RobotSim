
class Trajectory {
  
  List<Float> phases = new ArrayList();
  List<Segment> segments_x = new ArrayList();
  List<Segment> segments_y = new ArrayList();
  List<Segment> segments_z = new ArrayList();
  
  Trajectory(){
    phases.add(0.0); 
  }
  
  void addSegment(float stop, Segment x, Segment y, Segment z){
    phases.add(stop);
    segments_x.add(x);
    segments_y.add(y);
    segments_z.add(z);
  }
  
  float[] interpolate(float phase){
    float p1 = phases.get(0), p2 = phases.get(1);
    int i=1;
    for(; i<phases.size(); i++){
      p2 = phases.get(i);
      if(phase >= p1 && phase <=p2)
        break;
      p1 = p2;
    }
    float ph = (phase - p1) / (p2 - p1);
    return new float[]{
      segments_x.get(i-1).interpolate(ph),
      segments_y.get(i-1).interpolate(ph),
      segments_z.get(i-1).interpolate(ph)
    };
  }
  
}

abstract class Segment {
    
  float x1, x2;
  
  Segment(float x1, float x2){
    this.x1 = x1;
    this.x2 = x2;
  }
  
  abstract float interpolate(float phase);
  
}

class Lin extends Segment {
    
  Lin(float x1, float x2){
    super(x1,x2);
  }
  
  float interpolate(float phase){
    return x1 + phase * (x2 - x1);
  }
  
}

class Cos extends Segment {
  
  float p1, p2, cosp1, cosp2;
  
  Cos(float x1, float x2, float p1, float p2){
    super(x1, x2);
    this.p1 = p1;
    this.p2 = p2;
    this.cosp1 = cos(p1);
    this.cosp2 = cos(p2);
  }
  
  float interpolate(float phase){
      return x1 + x2 * cos(p1 + (p2-p1) * phase);
  }
}

class Constant extends Lin {
 
  Constant(float c){
    super(c, c); 
  }
  
}