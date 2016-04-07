
// ROBOT class

class MyRobot extends Drawable {

  PVector cg;

  float mass;

  // Objects in the simulation
  Ground ground;
  Frame frame;
  Leg[] legs = new Leg[4];

  // Stable and unstable triangles
  List<PVector[]> stable = new ArrayList(), unstable = new ArrayList();

  MyRobot() {
    super();
    frame = new Frame();
    frame.setAttitude(85, 0, 0);
    ground = new Ground();
    legs[0] = new Leg(new PVector(0,0), false, false);
    legs[1] = new Leg(new PVector(frame.length_,0), false, true);
    legs[2] = new Leg(new PVector(0,frame.width_), true, false);
    legs[3] = new Leg(new PVector(frame.length_,frame.width_), true, true);
  }

  void apply(Move move, float phase){
    frame.setAttitude(move.getHeight(phase), move.getPitch(phase), move.getRoll(phase));
    for(int i=0; i<4; i++){  // For each leg
      legs[i].slot = frame.getSlotPosition(i);
      legs[i].foot = move.getFootPosition(i, phase);  // Trajectory planning
      legs[i].footTrajectory = move.trajectories[i]; 
      legs[i].resolve();  // Inverse kinematics & CG
    }
    resolve();  // Compute CG and stability triangles
    ground.resolve(move, phase, TIME.dt);  // Compute ground position variation 
  }
  
  void resolve() {
    mass = frame.mass + legs[0].mass + legs[1].mass + legs[2].mass + legs[3].mass;
    cg = frame.ref.copy().mult(frame.mass/mass);
    for (int i=0; i<4; i++)
      cg.add(legs[i].cg.copy().mult(legs[i].mass / mass));
    computeStabilityTriangles();  // Static stability
  }

  // Calculate the triangles formed by the legs, and check if the CG is inside
  void computeStabilityTriangles() {
    stable.clear(); 
    unstable.clear();
    for (int i=0; i<4; i++) {
      PVector[] points = new PVector[3];
      for (int j=0; j<3; j++) {
        int k = j<i? j : j+1;
        points[j] = legs[k].foot;
      }
      if (points[0].z==0.0 && points[1].z==0.0 && points[2].z==0.0) {
        if (pointInTriangle(points, cg))
          stable.add(points);
        else
          unstable.add(points);
      }
    }
  }

  // Checks if a point is inside a triangle
  boolean pointInTriangle(PVector[] points, PVector pt) {
    float area = 0.5*(-points[1].y*points[2].x + points[0].y*(-points[1].x + points[2].x) + points[0].x*(points[1].y - points[2].y) + points[1].x*points[2].y);
    float s = 1/(2*area)*(points[0].y*points[2].x - points[0].x*points[2].y + (points[2].y - points[0].y)*pt.x + (points[0].x - points[2].x)*pt.y);
    float t = 1/(2*area)*(points[0].x*points[1].y - points[0].y*points[1].x + (points[0].y - points[1].y)*pt.x + (points[1].x - points[0].x)*pt.y);
    return 0 <= s && s<= 1 && 0 <= t && t <= 1 && s + t <= 1;
  }

  void computeCalibrationPoints() {
    for (int i = 0; i<3; i++) {
      for (int j = 0; j<3; j++) {
        PVector pt = new PVector(-20.0+i*20.0, -20.0+j*20.0, 0.0);
        for (int k=0; k<4; k++) {
          Leg leg = legs[k];
          PVector foot = leg.slot.copy();
          if (leg.right) {
            foot.add(0, leg.shoulderWidth, -frame.ref.z).add(pt);
          } else {
            foot.add(0, -leg.shoulderWidth, -frame.ref.z).add(pt);
          }
          leg.foot = foot;
          leg.resolve();
          println(degrees(leg.theta)+" "+degrees(leg.phi)+" "+degrees(leg.psi));
        }
      }
    }
  }

  void draw(View v) {
    if (v==UI.TOP) {

      v.strokeWeight(1.5);
      // draw unstable triangles (red)
      v.setRed();
      for (PVector[] points : unstable)
        v.triangle(points[0], points[1], points[2]);
      // draw stable triangles (green)
      v.setGreen();
      for (PVector[] points : stable)
        v.triangle(points[0], points[1], points[2]);
      v.strokeWeight(3);
    }

    if (stable.size()>0)
      v.setGreen();
    else
      v.setRed();
    v.cg(cg);
  }
}


// LEG class

class Leg extends Drawable {
  
  float shoulderWidth = 25.0, upperLegLength = 53.0, lowerLegLength = 53.0;

  boolean right, forward;  // To differentiate the different legs

  PVector slot, shoulder, knee, foot, foot_ref;  // Coordinates of each articulation

  float theta, phi, psi;  // Angles of each articulation (in degrees)

  PVector[] footTrajectory;  // Table containing the trajectory of the foot over one period

  float shoulderMass = 17.9;
  float upperLegMass = 17.4;
  float lowerLegMass = 8.2;
  float mass = shoulderMass + upperLegMass + lowerLegMass;

  PVector shoulderCG, upperLegCG, lowerLegCG, cg;      // CG of each segment in the main axis system (computed)


  Leg(PVector foot_ref, boolean right, boolean forward) {
    super();
    this.foot_ref = foot_ref;
    this.right = right;
    this.forward = forward;
  }


  // Calculation functions

  void resolve() {
    computeShoulder();
    computeKnee();
    computeAngles();
    computeCG();
  }

  void computeShoulder() {
    PVector slot_foot = slot_foot();  // Vector from slot to foot
    slot_foot.add(ROBOT.frame.xf.copy().mult(-slot_foot.dot(ROBOT.frame.xf)));  //Projection on plane of rotation of shoulder
    float a1 = acos(shoulderWidth / slot_foot.mag()); // always positive : angle between slot->foot and slot->shoulder
    PVector slot_shoulder = new Rotation(ROBOT.frame.xf, a1*right()).rotate(slot_foot.setMag(shoulderWidth));  // Rotate by a1 and resize to transform slot->foot into slot->shoulder
    shoulder = slot.copy().add(slot_shoulder);
  }

  void computeKnee() {  // Computes the intersection of two circles
    float d = foot.dist(shoulder);
    float a = (pow(lowerLegLength, 2) - pow(upperLegLength, 2) + d*d) / (2 * d);
    float h = sqrt(pow(lowerLegLength, 2) - a*a); 
    PVector pt = foot.copy().add(foot_shoulder().mult(a/d));
    PVector normal = slot_shoulder().normalize().mult(right());
    knee = pt.add(foot_shoulder().cross(normal).mult(h/d));
  }

  void computeAngles() {
    theta = signedAngleBetween(ROBOT.frame.yf.copy().mult(right()), slot_shoulder(), ROBOT.frame.xf.copy());
    phi = PI - PVector.angleBetween(ROBOT.frame.xf, shoulder_knee());
    psi = PVector.angleBetween(shoulder_knee(), knee_foot());
  }

  void computeCG() {
    Rotation rotationShoulder = ROBOT.frame.attitude.compose(new Rotation(ROBOT.frame.xf, theta));
    Rotation rotationUpperLeg = rotationShoulder.compose(new Rotation(ROBOT.frame.yf, HALF_PI - phi));
    Rotation rotationLowerLeg = rotationUpperLeg.compose(new Rotation(ROBOT.frame.yf, HALF_PI - psi));
    PVector shoulderCG = new PVector(-5*forward(), 5.4*right(), 0);      // CG of each segment in its own axis system (given)
    PVector upperLegCG = new PVector(0, 5.5*right(), -39.1);
    PVector lowerLegCG = new PVector(32.7, 0, 0);
    this.shoulderCG = rotationShoulder.rotate(shoulderCG).add(slot);
    this.upperLegCG = rotationUpperLeg.rotate(upperLegCG).add(shoulder);
    this.lowerLegCG = rotationLowerLeg.rotate(lowerLegCG).add(knee);
    cg = this.shoulderCG.copy().mult(shoulderMass / mass);
    cg.add(this.upperLegCG.copy().mult(upperLegMass / mass));
    cg.add(this.lowerLegCG.copy().mult(lowerLegMass / mass));
  }



  void draw(View v) {
    for (int i=0; i<footTrajectory.length; i++) {
      PVector pt = footTrajectory[i];
      if (pt.z==0.0) v.setGreen(); 
      else v.setRed();
      v.point(pt);
    }

    if (foot.z==0.0) v.setGreen(); 
    else v.setRed();
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

  PVector slot_foot() { 
    return foot.copy().sub(slot);
  }

  PVector slot_shoulder() { 
    return shoulder.copy().sub(slot);
  }

  PVector foot_shoulder() { 
    return shoulder.copy().sub(foot);
  }

  PVector shoulder_knee() { 
    return knee.copy().sub(shoulder);
  }

  PVector knee_foot() { 
    return foot.copy().sub(knee);
  }

  float right() { 
    return right? 1.0 : -1.0;
  }

  float forward() { 
    return forward? 1.0 : -1.0;
  }
  
  float signedAngleBetween(PVector v1, PVector v2, PVector ref){
    return PVector.angleBetween(v1,v2) * Math.signum(v1.cross(v2).dot(ref));
  }
}


// FRAME class (for visualization)

class Frame extends Drawable {

  float width_ = 43.75, length_ = 127.75, thick = 5.0;
  float mass = 380;
  
  PVector ref;      // Reference point for speed
  Rotation attitude;
  PVector xf, yf, zf;
    
  void setAttitude(float h, float pitch, float roll){
    ref = new PVector(0.5*length_, 0.5*width_, h);
    attitude = new Rotation(EY, pitch).compose(new Rotation(EX, roll));
    xf = attitude.rotate(EX);
    yf = attitude.rotate(EY);
    zf = attitude.rotate(EZ);
  }
  
  PVector getSlotPosition(int i){
    switch(i){
      case 0: return getPoint(-0.5*length_, -0.5*width_, 0);
      case 1: return getPoint( 0.5*length_, -0.5*width_, 0);
      case 2: return getPoint(-0.5*length_,  0.5*width_, 0);
      default: return getPoint(0.5*length_,  0.5*width_, 0);
    }
  }
  
  void draw(View v) {
    v.stroke(50, 117, 200);
   
    float margin = 5.0;
    PVector[] corners = new PVector[8];
    for(int i=0; i<2; i++)
      for(int j=0; j<2; j++)
        for(int k=0; k<2; k++)
          corners[i+j*2+k*4] = getPoint(0.5*length_*(i==1?1:-1), (0.5*width_-margin)*(j==1?1:-1), 0.5*thick*(k==1?1:-1));

    for(int i=0; i<2; i++){
      v.line(corners[0+i*4], corners[1+i*4]);
      v.line(corners[1+i*4], corners[3+i*4]);
      v.line(corners[3+i*4], corners[2+i*4]);
      v.line(corners[2+i*4], corners[0+i*4]);
      v.line(corners[0+i*2], corners[4+i*2]);
      v.line(corners[1+i*2], corners[5+i*2]);
    }
        
    v.line(getSlotPosition(0), getSlotPosition(2));
    v.line(getSlotPosition(1), getSlotPosition(3));

    v.stroke(100);
    v.cg(ref);
  }
  
  PVector getPoint(float x, float y, float z){
    return ref.copy().add(attitude.rotate(new PVector(x,y,z)));
  }
}


// GROUND class (for visualization)

class Ground extends Drawable {

  float angle = HALF_PI*0.5;  // Angle of the grid
  PVector grid_point = new PVector(0.0, 0.0, 0.0);  // Arbitrary point of the grid

  PVector speed_grid, speed_cg;


  void resolve(Move move, float phase, float dt) {
    float rotation = move.getRotation(phase);
    speed_cg = move.getSpeed(phase);
    speed_grid = move.getSpeed(phase, grid_point);
    angle = ((angle + rotation * dt) % HALF_PI + HALF_PI) % HALF_PI;
    grid_point.add(-speed_grid.x * dt, -speed_grid.y * dt);
  }  

  void draw(View v) {

    float pitch = 8, margin = 30;
    v.stroke(176, 124, 82);

    if (v==UI.TOP) {

      v.strokeWeight(1);
      v.grid(grid_point.x, grid_point.y, -HALF_PI*0.5 + angle, pitch);
      v.strokeWeight(3);
    } else {

      PVector start = new PVector(-margin, -margin, 0);
      PVector end = new PVector(ROBOT.frame.length_+margin, ROBOT.frame.width_+margin, 0);
      v.line(start, end);
      float slash_x = pitch + v.xraw(grid_point) % pitch;
      for (start.add(slash_x, slash_x); v.x(start) <= v.x(end); start.add(pitch, pitch))
        v.line(start, start.copy().add(-pitch, -pitch, -pitch));
    }
  }
}