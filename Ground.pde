class Ground extends Drawable {
    
  float angle = HALF_PI*0.5;  // Angle of the grid
  PVector grid_point = new PVector(0.0, 0.0, 0.0);  // Point of the grid
  
  void draw(View v){
    
    float pitch = 20, margin = SCALE * 20.0;
    v.stroke(176, 124, 82);
    
    if(v==TOP){
      
      float rotation = move.getRotation(phase);
      PVector speed = move.getSpeed(phase, grid_point);
      angle = (angle + rotation * dt) % HALF_PI;
      grid_point.add(-speed.x * dt, -speed.y * dt);
      
      v.strokeWeight(1);
      v.grid(grid_point.x, grid_point.y, -HALF_PI*0.5 + angle, pitch);
      v.strokeWeight(3);
      
    }else if(v==SIDE || v==FRONT){
      
      PVector start = new PVector(-margin, -margin, 0);
      PVector end = new PVector(frame.getLength()+margin, frame.getWidth()+margin, 0);
      v.line(start, end);
      float speed = v.x(move.getSpeed(phase)) - v.offset_x;
      float slash_x = pitch + (-speed * time) % pitch;
      for(start.add(slash_x,slash_x); v.x(start) <= v.x(end); start.add(pitch,pitch))
        v.line(start, start.copy().add(-pitch, -pitch, -pitch)); 
        
    }
  }
  
}