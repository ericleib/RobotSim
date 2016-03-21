
class Time {
  
  // Simulation parameters
  float fps, dt, time, time_1, phase;

  boolean pause = false;  // Whether the simulation is paused or not
  boolean reset = false;  // Flag to reset the simulation
  
  Time(int fps){
    this.fps = fps;
    this.dt = 1.0/fps;
    frameRate(fps);
  }
  
  void update(float period){
    fill(0);
    text(nf(time,1,2)+" sec", 3, 15);
    text(MOVE.getName(), 80, 15);
    if(!pause)
      time += 1.0 / fps; 
    dt = time-time_1;
    if(Float.isFinite(period)){
      if(period==0)
        println("Warning: period==0 must be prevented!");
      phase += dt / period;   // Increment of phase: dt / T
    }
    time_1 = time;
    if(reset){
      reset = false;
      phase = 0.0;
      time = 0.0;
      time_1 = 0.0;
    }
  }
  
  void event(ControlEvent e){
    if(e.getName()=="pause") pause = !pause;
    if(e.getName()=="next" || e.getName()=="prev")
      if(!pause)
        pause = true;
      else
        time += (e.getName()=="next"? 1.0 : -1.0) / fps;
    if(e.getName()=="reset") reset = true; 
  }

}