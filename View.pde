
View TOP, SIDE, FRONT;
List<Drawable> OBJECTS = new ArrayList();

void createViews(){
  TOP = new View(2, 120, 150, 100, 300 + frame.getLength(), 200 + frame.getWidth());
  SIDE = new View(2, TOP.y+TOP.h-1, TOP.offset_x, TOP.offset_y*0.4+frame.getHeight(), TOP.w, TOP.offset_y * 0.7 + frame.getHeight());
  FRONT = new View(TOP.x+TOP.w-1, SIDE.y, TOP.offset_x, SIDE.offset_y, 2*TOP.offset_x + frame.getWidth(), SIDE.h);
}

void drawViews(){
  TOP.draw();
  SIDE.draw();
  FRONT.draw(); 
}

abstract class Drawable {
  Drawable(){  OBJECTS.add(this); }  
  abstract void draw(View v);
}

class View {
  
  PGraphics pg;
  float x,y,offset_x,offset_y,w,h;
  
  View(float x, float y, float offset_x, float offset_y, float w, float h){
    this.pg = createGraphics((int)w, (int)h);
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
    this.offset_x = offset_x;
    this.offset_y = offset_y;
  }
  
  void draw(){
    pg.beginDraw();
    pg.clear(); 
    pg.noFill();
    pg.strokeWeight(1);   
    pg.rect(0,0,w-1,h-1);
    pg.strokeWeight(3);   
    for(Drawable d : OBJECTS)
      d.draw(this);
    pg.endDraw();
    image(pg, x, y);
  }
  
  
  // Drawing functions

  void arc(PVector v, float r, float a1, float a2){
    pg.arc(x(v), y(v), r, r, a1, a2);
  }
  
  void ellipse(PVector v, float r){
    pg.ellipse(x(v), y(v), r, r);
  }
  
  void line(PVector v1, PVector v2){
    pg.line(x(v1), y(v1), x(v2), y(v2));
  }
  
  void rect(PVector v1, PVector v2){
    pg.rect(x(v1), y(v1), x(v2)-x(v1), y(v2)-y(v1));
  }
  
  void triangle(PVector v1, PVector v2, PVector v3){
    pg.triangle(x(v1), y(v1), x(v2), y(v2), x(v3), y(v3));
  }
  
  void point(PVector pt){
    pg.point(x(pt), y(pt));
  }
  
  void grid(float x0, float y0, float angle, float pitch){
    float p = pitch / cos(angle);
    float tan = tan(angle);
    float x = -h + (offset_x+x0-tan*(offset_y+y0) + h) % p;
    float y = -w + (offset_y+y0+tan*(offset_x+x0) + w) % p;
    for(; x<=w+h || y<=w+h ; x+=p, y+=p){
      pg.line(x,0,x+h*tan,h);
      pg.line(0,y,w,y-w*tan);
    }
  }
  
  void cg(PVector cg){
    pg.strokeWeight(1);
    ellipse(cg, 10);
    pg.fill(pg.strokeColor);
    arc(cg, 10, 0, HALF_PI);
    arc(cg, 10, PI, PI + HALF_PI);   
    pg.noFill();
    pg.strokeWeight(3);
  }
  
  
  // Extract coordinates from vectors, according to the current view
  
  float x(PVector v){
    return offset_x + (this==TOP? v.x : this==SIDE? v.x : v.y);
  }
  
  float y(PVector v){
    return offset_y + (this==TOP? v.y : this==SIDE? -v.z : -v.z);
  }
  
  
  
  // Style
   
  void setRed(){ 
    pg.stroke(210, 38, 38);
  }
  
  void setGreen(){ 
    pg.stroke(9, 169, 61); 
  }
  
  void stroke(int a, int b, int c){
    pg.stroke(a,b,c);
  }
  
  void stroke(int i){
    pg.stroke(i);
  }
  
  void strokeWeight(float f){
    pg.strokeWeight(f);
  }
  
  void fill(int a, int b, int c, int d){
    pg.fill(a,b,c,d);
  }
  
  void fill(int a, int b, int c){
    pg.fill(a,b,c);
  }
  
  void fill(int i){
    pg.fill(i);
  }
  
  void noFill(){
    pg.noFill();
  }
}