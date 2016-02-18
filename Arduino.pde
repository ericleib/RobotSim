
import processing.serial.*;

import cc.arduino.*;

Arduino arduino;

static final int LATERAL = 10, CONTACT = 11, PENDULUM = 12;

void setupArduino() {
  
  // Prints out the available serial ports.
  println("Available boards on serial ports:");
  for(String s : Arduino.list())
    println("-> "+s);
  
  if(Arduino.list().length == 0)
    System.err.println("It appears you have not connected a board!");
    
  else{
    // Modify this line, by changing the "0" to the index of the serial
    // port corresponding to your Arduino board (as it appears in the list
    // printed by the line above).
    arduino = new Arduino(this, Arduino.list()[0], 57600);
 /*
    arduino.pinMode(LATERAL, Arduino.PWM);
    arduino.pinMode(CONTACT, Arduino.PWM);
    arduino.pinMode(PENDULUM, Arduino.PWM);
 */   
    arduino.pinMode(LATERAL, Arduino.SERVO);
    arduino.pinMode(CONTACT, Arduino.SERVO);
    arduino.pinMode(PENDULUM, Arduino.SERVO);
  }
}

void drawArduino(){
  
  if(arduino!=null){
    //int Ptheta = (int) (-673 * theta[0] * PI/180 + 1608);
    //int Pphi = (int) (-694 * phi[0] * PI/180 + 2330);
    //int Ppsi = (int) ( 577 * psi[0] * PI/180 + 784);
    //arduino.servoWrite(LATERAL, constrain(mouseX / 2, 80, 110));
    //arduino.analogWrite(LATERAL, constrain(Ptheta, 200, 220));
    //arduino.analogWrite(CONTACT, constrain(Ppsi, 50, 200));
    //arduino.analogWrite(PENDULUM, constrain(Pphi, 0, 255));
    //arduino.analogWrite(LATERAL, constrain(Ptheta, 0, 255));
    //arduino.analogWrite(CONTACT, constrain(arduino.analogRead(1), 0, 255));
    //arduino.analogWrite(PENDULUM, constrain(arduino.analogRead(2), 0, 500));
    //arduino.analogWrite(LATERAL, constrain(arduino.analogRead(3), 0, 500));
    //arduino.analogWrite(CONTACT, constrain(arduino.analogRead(2)*180/1024, 70, 120));
    //arduino.analogWrite(PENDULUM, constrain(arduino.analogRead(3)*180/1024, 70, 120));
    
    int Ptheta = (int) interp(-90, -135, 102.3, 48.7, theta[0]);
    int Pphi = (int) interp(90, 135, 69.1, 121.8, phi[0]);
    int Ppsi = (int) interp(90, 135, 102, 142.4, psi[0]);
    
    //arduino.servoWrite(CONTACT, constrain(arduino.analogRead(1)*180/1024, 0, 180));
    arduino.servoWrite(CONTACT, constrain(Ppsi, 0, 180));
    //println("Contact: "+arduino.analogRead(1)*180.0/1024+" "+psi[0]);
    //arduino.servoWrite(PENDULUM, constrain(arduino.analogRead(2)*180/1024, 0, 180));
    arduino.servoWrite(PENDULUM, constrain(Pphi, 0, 180));
    //println("Pendulum: "+arduino.analogRead(2)*180.0/1024+" "+phi[0]);
    //arduino.servoWrite(LATERAL, constrain(arduino.analogRead(3)*180/1024, 0, 180));
    arduino.servoWrite(LATERAL, constrain(Ptheta, 0, 180));
    //println("Lateral: "+arduino.analogRead(3)*180.0/1024+" "+theta[0]);
    
  }
}

float interp(float x1, float x2, float y1, float y2, float x){
  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

void updateArduinoChart(Chart c){
  if(arduino != null){
    c.setData(new float[]{arduino.analogRead(1), arduino.analogRead(2), arduino.analogRead(3)});
  }
}