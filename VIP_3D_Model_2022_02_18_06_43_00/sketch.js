let scaleSlider;
let hipSlider;
let thighSlider;
let calfSlider;

let foot_model;
function setup() {
  createCanvas(600, 600, WEBGL);
  leg = new Leg(createVector(0, 2, 2), 4.5, 1.5, 4.5, 1.5, 6.5);
  angleMode(RADIANS);
  
  scaleSlider = createSlider(10, 50, 30, 0);
  scaleSlider.position(20, 30);
  
  hipSlider = createSlider(-PI/2, PI/2, 0, 0);
  hipSlider.position(20, 60);
  
  thighSlider = createSlider(-PI/2, PI/2, 0, 0);
  thighSlider.position(20, 90);
  
  calfSlider = createSlider(0, PI, PI/2, 0);
  calfSlider.position(20, 120);
  
}

function draw() {
  background(200);
  rotateInputs();
  leg.render_conversion_scale = scaleSlider.value();
  leg.hip_theta = hipSlider.value();
  leg.thigh_theta = thighSlider.value();
  leg.calf_theta = calfSlider.value();
    
  leg.forward_kinematics();
  leg.render();
  
  leg.inverse_kinematics(createVector(0, 4, -4))
}

function rotateInputs(){
  if (keyIsDown(LEFT_ARROW)){
    leg.perspective_angleY -= 0.05;
  }
  if (keyIsDown(RIGHT_ARROW)){
    leg.perspective_angleY += 0.05;
  }
  if (keyIsDown(UP_ARROW)){
    leg.perspective_angleX -= 0.05;
  }
  if (keyIsDown(DOWN_ARROW)){
    leg.perspective_angleX += 0.05;
  }
}

function keyPressed(){
  if(keyCode === ENTER){
    leg.perspective_angleX = -PI/8;
    leg.perspective_angleY = PI/4;
    leg.perspective_angleZ = 0;
    hipSlider.value(0);
    thighSlider.value(0);
    calfSlider.value(PI/2);
    
  }
}