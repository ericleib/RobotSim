import controlP5.*;    // import controlP5 library

ControlP5 cp5;   // controlP5 object
Chart phiChart, psiChart, thetaChart, phidChart, psidChart, arduChart;
List<Controller> sliders = new ArrayList();

void setupCharts(){
  cp5 = new ControlP5(this);  
  cp5.addButton("prev").setPosition(3,22).setSize(38,15);
  cp5.addButton("pause").setPosition(43,22).setSize(38,15);
  cp5.addButton("next").setPosition(83,22).setSize(38,15);
  cp5.addButton("reset").setPosition(123,22).setSize(38,15);
  
  phiChart = makeChart("phi", 0, 0, 180);
  psiChart = makeChart("psi", 1, 0, 180);
  thetaChart = makeChart("theta", 2, -90, 90);
  //phidChart = makeChart("phid", 2, -60, 60);
  //psidChart = makeChart("psid", 3, -60, 60);

  arduChart = cp5.addChart("Arduino")
               .setPosition(3,45)
               .setSize(160, 60)
               .setRange(0, 1024)
               .setColorCaptionLabel(color(40))
               .setView(Chart.BAR)
               .addDataSet("arduino")
               .setData("arduino", new float[] {300,900,700});
}

void createSliders(){
  for(Controller c:sliders)
    cp5.remove(c.getName());
  sliders.clear();
  int i=0, j=0;
  for(Parameter p:move.parameters.values()){
    Controller c = cp5.addSlider(p.name, p.min, p.max, p.value, 170+j*350, 5+(i++)*17, 280, 15);
    c.getValueLabel().getStyle().margin(0,0,0,3);
    c.getCaptionLabel().getStyle().margin(0,0,0,3);
    sliders.add(c);
    c.getCaptionLabel().setColor(0);
    if(i==6){ j++; i=0;}
  }
}

void updateCharts(){
  if(cp5!=null){
    phiChart.push("phi", legs[0].phi);
    psiChart.push("psi", legs[0].psi);
    thetaChart.push("theta", legs[0].theta);
    updateArduinoChart(arduChart);
  }
}


Chart makeChart(String name, int n, float min, float max){
  return cp5.addChart(name)
               .setPosition(FRONT.x+15, TOP.y + n * TOP.h /3)
               .setSize((int)FRONT.w-30, (int)TOP.h /4)
               .setRange(min, max)
               .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
               .setColorBackground(color(255))
               .setStrokeWeight(3)
               .setColorCaptionLabel(color(40))
               .addDataSet(name)
               ;
}

void controlEvent(ControlEvent e) {
  if(e.getName()=="pause") pause = !pause;
  if(e.getName()=="next" || e.getName()=="prev")
    if(!pause)
      pause = true;
    else
      time += (e.getName()=="next"? 1.0 : -1.0) / FPS;
  if(e.getName()=="reset") reset = true;
  if(e.getController() instanceof Slider){
    move.set(e.getName(), e.getValue());
    move.update();
  }
}