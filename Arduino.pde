
import processing.serial.*;

import cc.arduino.*;

Arduino arduino;

static final int LATERAL = 4, CONTACT = 3, PENDULUM = 2;

void setupArduino() {
  
  // Prints out the available serial ports.
  println("Available boards on serial ports:");
  for(String s : Arduino.list())
    println("-> "+s);
  
  if(Arduino.list().length == 0)
    System.err.println("It appears you have not connected a board!");
    
  else{
    arduino = new Arduino(this, Arduino.list()[0], 57600);
    arduino.pinMode(LATERAL, Arduino.SERVO);
    arduino.pinMode(CONTACT, Arduino.SERVO);
    arduino.pinMode(PENDULUM, Arduino.SERVO);
  }
}

void drawArduino(){
  
  if(arduino!=null){
    int Ptheta = (int) interp(0, -45, 102.3, 48.7, ROBOT.legs[0].theta);
    int Pphi = (int) interp(90, 45, 69.1, 121.8, ROBOT.legs[0].phi);
    int Ppsi = (int) interp(90, 135, 102, 142.4, ROBOT.legs[0].psi);
    
    arduino.servoWrite(CONTACT, constrain(Ppsi, 0, 180));
    arduino.servoWrite(PENDULUM, constrain(Pphi, 0, 180));
    arduino.servoWrite(LATERAL, constrain(Ptheta, 0, 180));
    
  }
}

float interp(float x1, float x2, float y1, float y2, float x){
  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

Chart makeArduinoChart(ControlP5 cp5){
  return cp5.addChart("Arduino")
               .setPosition(3,45)
               .setSize(160, 60)
               .setRange(0, 1024)
               .setColorCaptionLabel(color(40))
               .setView(Chart.BAR)
               .addDataSet("arduino")
               .setData("arduino", new float[] {300,900,700}); 
}

void updateArduinoChart(Chart c){
  if(arduino != null){
    c.setData(new float[]{arduino.analogRead(1), arduino.analogRead(2), arduino.analogRead(3)});
  }
}