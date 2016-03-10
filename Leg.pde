class Leg extends Drawable {
  
  
  boolean right, forward;  // To differentiate the different legs
  
  PVector slot, shoulder, knee, foot;  // Coordinates of each articulation
  
  float theta, phi, psi;  // Angles of each articulation (in degrees)
  
  PVector[] footTrajectory;  // Table containing the trajectory of the foot over one period
  
  float shoulderWidth = SCALE * 25.0;
  float upperLegLength = SCALE * 53.0;
  float lowerLegLength = SCALE * 53.0;
  
  float shoulderMass = 17.9;
  float upperLegMass = 17.4;
  float lowerLegMass = 8.2;
  float mass = shoulderMass + upperLegMass + lowerLegMass;
    
  PVector shoulderCG, upperLegCG, lowerLegCG, cg;      // CG of each segment in the main axis system (computed)
  
  
  Leg(PVector slot, boolean right, boolean forward){
    super();
    this.slot  = slot;
    this.right = right;
    this.forward = forward;
  }
    
  
  // Calculation functions
  
  void resolve(){
    computeShoulder();
    computeKnee();
    computeAngles();
    computeCG();
  }
  
  void computeShoulder(){
    PVector slot_foot = slot_foot();  // Vector from slot to foot
    slot_foot.x = 0.0;                // Project on plan (y,z)
    float a1 = acos(shoulderWidth / slot_foot.mag()); // always positive 
    float a2 = atan((slot.z - foot.z) / abs(foot.y-slot.y)); // always positive
    shoulder = slot.copy().add(0, shoulderWidth*cos(a1-a2)*right(), shoulderWidth*sin(a1-a2));
  }
  
  void computeKnee(){  // Computes the intersection of two circles
    float d = foot.dist(shoulder);
    float a = (lowerLegLength*lowerLegLength - upperLegLength*upperLegLength + d*d) / (2 * d);
    float h = sqrt(lowerLegLength*lowerLegLength - a*a); 
    PVector pt = foot.copy().add(foot_shoulder().mult(a/d));
    PVector normal = slot_shoulder().normalize().mult(right());
    knee = pt.add(foot_shoulder().cross(normal).mult(h/d));
  }
  
  void computeAngles(){
    theta = atan((shoulder.z-slot.z) / (shoulder.y-slot.y) );
    phi = PI - PVector.angleBetween(EX, shoulder_knee());
    psi = PVector.angleBetween(shoulder_knee(), knee_foot());
  }
  
  void computeCG(){
    Rotation rotationShoulder = new Rotation(EX, theta);
    Rotation rotationUpperLeg = rotationShoulder.compose(new Rotation(EY, HALF_PI - phi));
    Rotation rotationLowerLeg = rotationUpperLeg.compose(new Rotation(EY, HALF_PI - psi));
    PVector shoulderCG = new PVector(-SCALE*5*forward(), SCALE*5.4*right(), SCALE*0);      // CG of each segment in its own axis system (given)
    PVector upperLegCG = new PVector(SCALE*0, SCALE*5.5*right(), -SCALE*39.1);
    PVector lowerLegCG = new PVector(SCALE*32.7, SCALE*0, SCALE*0);
    this.shoulderCG = rotationShoulder.rotate(shoulderCG).add(slot);
    this.upperLegCG = rotationUpperLeg.rotate(upperLegCG).add(shoulder);
    this.lowerLegCG = rotationLowerLeg.rotate(lowerLegCG).add(knee);
    cg = this.shoulderCG.copy().mult(shoulderMass / mass);
    cg.add(this.upperLegCG.copy().mult(upperLegMass / mass));
    cg.add(this.lowerLegCG.copy().mult(lowerLegMass / mass));
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
    v.cg(shoulderCG);
    v.cg(upperLegCG);
    v.cg(lowerLegCG);
    //v.cg(cg);
  }
  
  // Shortcuts functions
    
  PVector slot_foot(){ return foot.copy().sub(slot); }
  
  PVector slot_shoulder(){ return shoulder.copy().sub(slot); }
  
  PVector foot_shoulder(){ return shoulder.copy().sub(foot); }
  
  PVector shoulder_knee(){ return knee.copy().sub(shoulder); }
  
  PVector knee_foot(){ return foot.copy().sub(knee); }
  
  float right(){ return right? 1.0 : -1.0; }
  
  float forward(){ return forward? 1.0 : -1.0; }
}