class Ground extends Drawable {
    
  void draw(View v){
    
    float pitch = 20, margin = SCALE * 20.0;
    v.stroke(176, 124, 82);
    
    if(v==TOP){
      
      float a=0.0, x0=frame.getLength()*0.5, y0=frame.getWidth()*0.5;
      float rotation = move.getRotation(phase);
      PVector speed = move.getSpeed(phase);
      if(rotation==0){
        x0 += (-speed.x * time) % pitch;
        y0 += (-speed.y * time) % pitch;
      }else{
        a = -HALF_PI*0.5 + ((time * rotation) % HALF_PI);
        x0 += speed.y / rotation;
        y0 += speed.x / rotation;
      }
      
      v.strokeWeight(1);
      v.grid(x0, y0, a, pitch);
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