class Leg extends Drawable {
  
  
  boolean right, forward;
  PVector slot, shoulder, knee, foot;
  
  float theta, phi, psi, thetad, phid, psid;
  
  PVector[] footTrajectory;
  
  
  Leg(PVector slot, boolean right, boolean forward){
    super();
    this.slot  = slot;
    this.right = right;
    this.forward = forward;
  }
  
  
  float getUpperLegLength(){ return SCALE*50.0; }
  
  float getLowerLegLength(){ return SCALE*50.0; }
  
  float getShoulderWidth(){ return SCALE*10.0; }
  
  
  // Calculation functions
  
  void resolve(){
    computeShoulder();
    computeKnee();
    computeAngles(); 
  }
  
  void computeShoulder(){
    PVector slot_foot = slot_foot();  // Vector from slot to foot
    slot_foot.x = 0.0;                // Project on plan (y,z)
    float d1 = getShoulderWidth();
    float a1 = acos(d1 / slot_foot.mag()); // always positive 
    float a2 = atan((slot.z - foot.z) / abs(foot.y-slot.y)); // always positive
    shoulder = slot.copy().add(0, d1*cos(a1-a2)*(right? 1 : -1), d1*sin(a1-a2));
  }
  
  void computeKnee(){  // Computes the intersection of two circles
    float r1 = getLowerLegLength();
    float r2 = getUpperLegLength();
    float d = foot.dist(shoulder);
    float a = (r1*r1 - r2*r2 + d*d) / (2 * d);
    float h = sqrt(r1*r1 - a*a);
    PVector pt = foot.copy().add(foot_shoulder().mult(a/d));
    PVector normal = slot_shoulder().normalize().mult((right? 1 : -1));
    knee = pt.add(foot_shoulder().cross(normal).mult(h/d));
  }
  
  void computeAngles(){  
    float theta_ = theta;
    float phi_ = phi;
    float psi_ = psi;
    theta = degrees( atan((shoulder.z-slot.z) / (shoulder.y-slot.y) ));
    phi = degrees( PVector.angleBetween(new PVector(-1,0,0), shoulder_knee()));
    psi = degrees( PVector.angleBetween(shoulder_knee(), knee_foot()));
    thetad = (theta - theta_)*FPS;
    phid = (phi - phi_)*FPS;
    psid = (psi - psi_)*FPS;
  }

  void draw(View v){
    for(int i=0; i<footTrajectory.length; i++){
      PVector pt = footTrajectory[i];
      if(pt.z==0.0) v.setGreen(); else v.setRed();
      v.point(pt);
    }
          
    if(foot.z==0.0) v.setGreen(); else v.setRed();
    v.ellipse(foot, 6);
    v.stroke(100);
    v.ellipse(shoulder, 6);
    v.ellipse(knee, 6);
    v.ellipse(slot, 6);
    v.line(foot, knee);
    v.line(knee, shoulder);
    v.line(slot, shoulder);
  }
  
  // Shortcuts functions
    
  PVector slot_foot(){ return foot.copy().sub(slot); }
  
  PVector slot_shoulder(){ return shoulder.copy().sub(slot); }
  
  PVector foot_shoulder(){ return shoulder.copy().sub(foot); }
  
  PVector shoulder_knee(){ return knee.copy().sub(shoulder); }
  
  PVector knee_foot(){ return foot.copy().sub(knee); }
  
}