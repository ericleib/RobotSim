import controlP5.*;    // import controlP5 library

ControlP5 cp5;   // controlP5 object
Chart phiChart, psiChart, thetaChart, phidChart, psidChart, arduChart;

void setupCharts(){
  cp5 = new ControlP5(this);
  Controller speedSlider = cp5.addSlider("speed",0.0,100.0,20.0,150,5,300,15);
  speedSlider.getCaptionLabel().setColor(0);
  Controller periodSlider = cp5.addSlider("dx",0.0,50.0,20.0,150,22,300,15);
  periodSlider.getCaptionLabel().setColor(0);
  cp5.addButton("pause").setPosition(500,5).setSize(35,15);
  cp5.addButton("next").setPosition(538,5).setSize(35,15);
  phiChart = makeChart("phi", 0, 0, 180);
  psiChart = makeChart("psi", 1, 0, 180);
  thetaChart = makeChart("theta", 2, -90, 90);
  //phidChart = makeChart("phid", 2, -60, 60);
  //psidChart = makeChart("psid", 3, -60, 60);
  
  arduChart = cp5.addChart("Arduino")
               .setPosition(150,40)
               .setSize(300, 25)
               .setRange(0, 1024)
               .setColorCaptionLabel(color(40))
               .setView(Chart.BAR)
               .addDataSet("arduino")
               .setData("arduino", new float[] {300,900,700});
}

void updateCharts(){
  println(theta[0]+" "+phi[0]+" "+psi[0]);
  //println(theta[1]+" "+phi[1]+" "+psi[1]);
  println(theta[2]+" "+phi[2]+" "+psi[2]);
  //println(theta[3]+" "+phi[3]+" "+psi[3]);
  phiChart.push("phi", phi[0]);
  psiChart.push("psi", psi[0]);
  thetaChart.push("theta", theta[0]);
  updateArduinoChart(arduChart);
  //phidChart.push("phid", phid[0]);
  //psidChart.push("psid", psid[0]);
}


Chart makeChart(String name, int n, float min, float max){
  return cp5.addChart(name)
               .setPosition(620, 5 + n * 115)
               .setSize(400, 100)
               .setRange(min, max)
               .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
               .setStrokeWeight(1.5)
               .setColorCaptionLabel(color(40))
               .addDataSet(name)
               ;
}

void controlEvent(ControlEvent e) {
  if(e.getName()=="pause") pause = !pause;
  if(e.getName()=="next")
    if(!pause)
      pause = true;
    else
      time += 1.0 / FPS;
}