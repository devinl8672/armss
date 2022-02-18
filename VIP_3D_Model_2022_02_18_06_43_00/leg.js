// Leg class

//hip_axis_offset_vector is a vector of the x,y and z offset of the 4_bar frame from the hip frame

//calf_length is the entire length of the calf link including the output link (from coupler joint to foot)
let ground_frame_model;
class Leg {
  constructor(hip_axis_offset_vector, thigh_length, crank_length, coupler_length, output_length, calf_length) {
    this.hip_offsetx = hip_axis_offset_vector.x;
    this.hip_offsety = hip_axis_offset_vector.y;
    this.hip_offsetz = hip_axis_offset_vector.z;
    this.thigh_length = thigh_length;
    this.crank_length = crank_length;
    this.coupler_length = coupler_length;
    this.output_length = output_length;
    this.calf_length = calf_length;
    
    this.calf_theta = 0;
    this.output_theta1 = 0;
    this.output_theta2 = 0;
    this.coupler_angle = 0;
    
    this.thigh_theta = 0;
    this.hip_theta = 0;
    
    this.perspective_angleX = -PI/8;
    this.perspective_angleY = PI/4;
    this.perspective_angleZ = 0;
    
    this.render_conversion_scale = 25;
    
    this.animation1_direction = 1;
  }
  
  forward_kinematics(){
    let a = this.crank_length;
    let b = this.output_length;
    let h = this.coupler_length;
    let g = this.thigh_length;
    let A = 2*a*b*cos(this.calf_theta) - 2*g*b;
    let B = 2*a*b*sin(this.calf_theta);
    let C = sq(g) + sq(b) + sq(a) - sq(h) - 2*a*g*cos(this.calf_theta);
    this.output_theta1 = 2*atan((B + sqrt(sq(A) + sq(B) - sq(C)))/(A+C));
    this.output_theta2 = 2*atan((B - sqrt(sq(A) + sq(B) - sq(C)))/(A+C));
    this.coupler_angle = acos((sq(g)+sq(a)-sq(h)-sq(b)-2*a*g*cos(this.calf_theta))/(2*b*h));
  }
  
  inverse_kinematics(pos){
    let Q = pos;
    
    let p_legx = this.hip_offsetx;
    let p_legy = this.hip_offsety;
    let p_legz = this.hip_offsetx;
    
    //Position of leg outstretched so t he abs(p) = abs(q)
    let P = createVector(p_legx, sqrt(sq(Q.y) + sq(Q.z) - sq(p_legz)), p_legz);
    this.hip_theta = -atan2(P.y * Q.z - P.z * Q.y, P.y*Q.y + P.z*Q.z);
  }

  render() {
    let cyl_radius = 0.2 * this.render_conversion_scale;
    
    // Rotate perspective ("camera")
    rotateX(this.perspective_angleX);
    rotateY(this.perspective_angleY);
    rotateZ(this.perspective_angleZ);
    // Global frame of leg mechanism
    push();
    

    translate(-this.i2u(1), 0);
    box(this.i2u(1))
    pop();
    
    rotateX(this.hip_theta);
    box(this.i2u(1));
    // Hip frame
    push();
    
    translate(0, this.i2u(this.hip_offsety) / 2);
    box(this.i2u(0.75), this.i2u(this.hip_offsety), this.i2u(0.75));
    translate(0,this.i2u(this.hip_offsety) / 2);
    translate(0, 0, this.i2u(this.hip_offsetz) / 4);
    box(this.i2u(0.75), this.i2u(0.75), this.i2u(this.hip_offsetz)/2);
    translate(0, 0, this.i2u(this.hip_offsetz) / 4);
    rotateZ(this.thigh_theta);
    box(this.i2u(1));
    translate(0, 0, this.i2u(this.hip_offsetz) / 2);
    push();
    
    // Reference frame of 4-bar origin
    push();
    
    rotateZ(PI/2);
    // Origin rotated 90 degrees
    push();
    
    translate(0, this.i2u(this.thigh_length) / 2);
    cylinder(cyl_radius, this.i2u(this.thigh_length));
    translate(0, this.i2u(this.thigh_length) / 2);
    sphere(this.i2u(0.4));
    // Back to origin rotated 90 degrees
    pop();
    
    rotateZ(-this.calf_theta);
    box(this.i2u(0.6));
    // Rotated origin frame of bottom of crank link
    push();
    
    translate(0,-this.i2u(this.crank_length) / 2);
    cylinder(cyl_radius, this.i2u(this.crank_length));
    //Rotated frame of middle of crank link
    push();
    
    translate(0,-this.i2u(this.crank_length) / 2);
    rotateZ(-this.coupler_angle);
    sphere(this.i2u(0.4));
    // Crank-coupler joint frame (y-axis aligned with coupler)
    push();
    
    translate(0,-this.i2u(this.coupler_length) / 2);
    cylinder(cyl_radius, this.i2u(this.coupler_length));
    // Middle of coupler link frame
    push();
    
    translate(0,-this.i2u(this.coupler_length) / 2);
    rotateZ(-this.output_theta2);
    sphere(this.i2u(0.4));
    // Coupler-output joint frame (y-axis aligned with output)
    push();
    
    translate(0,-this.i2u(this.calf_length) / 2);
    cylinder(cyl_radius, this.i2u(this.calf_length));
    // Middle of output link frame
    push();
    
    translate(0,-this.i2u(this.calf_length) / 2);
    // Foot of output link frame
    push();
    
    //foot
    box(this.i2u(1));
    
  }
  
  // Conversion from inches to p5js units
  i2u(x){
    return x * this.render_conversion_scale;
  }
  
  rotate_perspective(x_speed, y_speed, z_speed){
    this.perspective_angleX += x_speed;
    this.perspective_angleY += y_speed;
    this.perspective_angleZ += z_speed;
  }
  
  
  animation1(){
    this.calf_theta += this.animation1_direction * (0.01 * sq(this.calf_theta) + 0.01);
    if (this.calf_theta >= PI){
      this.animation1_direction = -1;
    }
    if (this.calf_theta <= 0){
      this.animation1_direction = 1;
    }
    this.forward_kinematics();
    this.rotate_perspective(0, 0.005, 0);
    this.render();
  }
  
  
}