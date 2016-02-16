
import processing.serial.*;

import cc.arduino.*;

Arduino arduino;

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
 
  }
      
}

void drawArduino(){
  
}

void updateArduinoChart(Chart c){
  if(arduino != null){
    c.setData(new float[]{arduino.analogRead(1), arduino.analogRead(2), arduino.analogRead(3)});
  }
}