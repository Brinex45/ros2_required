#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <Cytron.h>
#include <Encoder.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <irc_interfaces/msg/ps4.h>

Encoder shoulder(41, 40);  
Encoder wrist(32, 33);
Encoder elbow(36, 37);   
Encoder turret(30, 31);

Cytron motorShoulder(2, 3, 0);
Cytron motorElbow(5, 4, 0);
Cytron motorWrist(7, 6, 1);
Cytron motorTurret(0, 1, 0);
Cytron motorRoll( 8, 9, 0); 
Cytron motorGrip(28, 29, 0);

#define slave_addr 9

rcl_subscription_t subscription;
irc_interfaces__msg__Ps4 ps4_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  digitalWrite(13, HIGH);
  delay(1000);
}

//used in IK calculation
float a1;
float a2;
float angle1;
float angle2;
float angle3;
float angle4;
float pos;  // z

float d1;
float d2;

float A;
float B;

float x;
float y;
float x2;
float y2;
float x3;
float y3;

float X;
float Y;

//lengths of links 
float length1 = 45;
float length2 = 48;
float length3 = 15;

//for data received from nav teensy
uint8_t data[5] = { 0 };
int8_t lx;
int8_t ly;
int8_t ry;
int8_t omegi;
uint8_t buttons;

// for storing values after mapping
double L_joystick_x;
double L_joystick_y;
double R_joystick_x;
double R_joystick_y;

bool right;
bool down;
bool up;
bool left;
bool square;
bool cross;
bool circle;
bool triangle;
  

  //for joint encoder angles
float Shoulder;
float Elbow;
float Wrist;                  
float Turret;

//  for speed of arm
double k = 0.0002;


//target coordinates
double x_target = 63;
double y_target = 45;
double z_target = 0;
double theta_target = 0;

// target coordinates of previous iteration
double prev_x_target;
double prev_y_target;
double prev_z_target;
double prev_theta_target;  

//for homing of arm
float error1 = 1, error2 = 1;
float prev1 = 0, prev2 = 0;

//flags for turret rotation 
bool manual_turret = false;
bool flag = false;
bool moving = false;

//pwm for turret, roll, grip motor
float pwm, pwm2, pwm3;


// void get_value() {
//   int i = 0;

//   while (Wire1.available() >= 5) {
//     Wire1.readBytes(data, 5);
//     buttons = data[0];
//     lx = data[1];
//     ly = data[2];
//     ry = data[3];
//     omegi = data[4];
//     // Serial.println((String) " "  + lx + "  " + ly +"  "+ry+"  "+ omegi);
//   }

//   up = ((buttons & (1 << 0)) ? 1 : 0);
//   right = ((buttons & (1 << 1)) ? 1 : 0);
//   down = ((buttons & (1 << 2)) ? 1 : 0);
//   left = ((buttons & (1 << 3)) ? 1 : 0);
//   triangle = ((buttons & (1 << 4)) ? 1 : 0);
//   circle = ((buttons & (1 << 5)) ? 1 : 0);
//   cross = ((buttons & (1 << 6)) ? 1 : 0);
//   square = ((buttons & (1 << 7)) ? 1 : 0);

//   L_joystick_x = map(lx, -127, 127, -65, 65);
//   L_joystick_y = map(ly, -127, 127, -60, 60);   
//   R_joystick_y = map(ry, -127, 127, -230, 230);

// }

void subscription_callback(const void * msgin)
{
  if (msgin == NULL) {
    return;
  }

  const irc_interfaces__msg__Ps4 * msg = (const irc_interfaces__msg__Ps4 *) msgin;

  // for (size_t i = 0; i < 6; i++) {
  //   joint_pwm.data.data[i] = msg->data.data[i];
  // }

  L_joystick_x = msg->ps4_data_analog[0];
  L_joystick_y = msg->ps4_data_analog[1];
  R_joystick_y = msg->ps4_data_analog[4];

  cross = msg->ps4_data_buttons[0];
  circle = msg->ps4_data_buttons[1];  
  triangle = msg->ps4_data_buttons[2];
  square = msg->ps4_data_buttons[3];

  up = msg->ps4_data_buttons[13];
  right = msg->ps4_data_buttons[14];
  down = msg->ps4_data_buttons[15];
  left = msg->ps4_data_buttons[16];

}

void getAngles() {

  Shoulder = 90.0 - (shoulder.read() * (360.0 / 8192.0));
  Elbow = 90.0 + (elbow.read() * (360.0 / 8192.0));
  Wrist = (wrist.read() * (360.0 / 8192.0));
  //Turret = (turret.read() * (360.0 / 460000.0));

}


void moveShoulder(float angle) {

  float error = angle - Shoulder;

  motorShoulder.rotate(constrain(error * 70, -120, 120));
}

void moveElbow(float angle) {       

  float error = angle - Elbow;

  motorElbow.rotate(constrain(-error * 200, -255, 255));
}

void moveWrist(float angle) {

  float error = angle - Wrist;

  motorWrist.rotate(constrain(error * 5, -30, 30));
}

void moveTurret(float angle) {

  float error = angle - Turret;

  motorTurret.rotate(constrain(error * 30, -70, 70));
}


void rotate() {

  getAngles();

  moveShoulder(angle1);
  moveElbow(angle2);
  moveWrist(angle3);

  // if (!manual_turret) {
  //   moveTurret(angle4);
  // }

  //moveMotor(l7, l8, constrain(pos2, -50, 50));
}


bool inside_ws() {

  if (isnan(angle1) || isnan(angle2) || isnan(angle3) || isnan(angle4) || angle1 > 90 || angle2 > 160 || angle2 < 90) {

    // Serial.println("coordinates out of workspace");
    motorShoulder.rotate(0);
    motorElbow.rotate(0);
    motorWrist.rotate(0);
    motorTurret.rotate(0);

    return false;
  }

  else {
    return true;
  }
}


void find_ik_2(float x, float y, float angle) {

  x3 = x - (length3 * cos(radians(angle)));
  y3 = y - (length3 * sin(radians(angle)));

  d1 = sqrt(pow(x3, 2) + pow(y3, 2));

  a1 = degrees(atan2(y3, x3));
  a2 = degrees(acos((pow(d1, 2) + pow(length1, 2) - pow(length2, 2)) / (2 * d1 * length1)));
  angle1 = (a1 + a2);

  x2 = length1 * cos(radians(angle1));
  y2 = length1 * sin(radians(angle1));
  d2 = sqrt(pow(x - x2, 2) + pow(y - y2, 2));

  angle2 = (degrees(acos((pow(length2, 2) + pow(length1, 2) - pow(d1, 2)) / (2 * length2 * length1))));

  angle3 = angle - angle1 + 180.0 - angle2;
}


void find_ik_3(float x, float y, float z,   float theta) {

  angle4 = degrees(atan2(z, x));
  find_ik_2(sqrt(pow(x, 2) + pow(z, 2)), y, theta);

  if (inside_ws()) {
    rotate();
  }

  else {

    x_target = prev_x_target;
    y_target = prev_y_target;
    z_target = prev_z_target;
    theta_target = prev_theta_target;

    // Serial.println("coordinates out of workspace");
  }
}


void move(float x, float y, float z, float theta) {

  find_ik_3(x, y, z, theta);
}

void home() {

  while (error1 != 0) {

    // Serial.println("shoulder going to home");
    float a = shoulder.read();
    motorShoulder.rotate(125);

    delay(800);
    float b = shoulder.read();
    error1 = a - b;
  }

  // Serial.println("shoulder is now in home");

  motorShoulder.rotate(0);

   delay(200);

  while (error2 != 0) {

    // Serial.println("elbow going to home");
    float a = elbow.read();
    motorElbow.rotate(255);
    delay(1500);

    float b = elbow.read();

    error2 = a - b;
  }

  // Serial.println("elbow is now in home");

  motorElbow.rotate(0);
  motorShoulder.rotate(0);
  shoulder.write(0);
  elbow.write(0);

}

void setup() {

  delay(2000);
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
  // Wire1.begin(9);
  // Wire1.onReceive(get_value);
  // Wire2.begin();

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_arm", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(irc_interfaces, msg, Ps4),
    "/ps4_data_arm"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscription,
    &ps4_msg,
    &subscription_callback,
    ON_NEW_DATA));

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  home();
  turret.write(0);
  elbow.write(0);
  wrist.write(0);
  shoulder.write(0);

}

void loop() {

  // get_value();

  if (L_joystick_x > 20 || L_joystick_x < -20) {
    x_target += L_joystick_x * k;
  }

  if (L_joystick_y > 20 || L_joystick_y < -20) {
    y_target += L_joystick_y * k;
  }

  if (R_joystick_y > 20 || R_joystick_y < -20) {
    theta_target += R_joystick_y * k;
  }

  if (square || circle) {

    manual_turret = true;
    if (square) pwm = 70;
    if (circle) pwm = -70;
    motorTurret.rotate(pwm);
    
  }

  else {
  
      motorTurret.rotate(0);
    }

  if (left || right) {

    if (right) pwm2 = 30;
    if (left) pwm2 = -30;
    motorRoll.rotate(pwm2);
    
  }

  else {
  
      motorRoll.rotate(0);
    }

 if (up || down) {

    if (up) pwm3 = 50;
    if (down) pwm3 = -50;
    motorGrip.rotate(pwm3);
    
  }

  else {
  
      motorGrip.rotate(0);
    }

  // Serial.print("X Target = ");
  // Serial.print(x_target);
  // Serial.print("       ");
  // Serial.print("Y Target = ");
  // Serial.print(y_target);
  // Serial.print("theta ");
  // Serial.print(theta_target);
  // Serial.print("       ");
  // Serial.print("wrist ");
  // Serial.print(Wrist);
  // Serial.print("   ");
  // Serial.print("shoulder ");
  // Serial.print(Shoulder);
  // Serial.print("   ");
  // Serial.print("elbow ");
  // Serial.print(Elbow);
  // Serial.print("   ");
  // Serial.print("Turret ");
  // Serial.print(Turret);
  // Serial.print("   ");
  // Serial.print("circle ");
  // Serial.println(circle);

  /*Serial.print(z_target);
  Serial.print("       ");
  Serial.print("theta Target = ");
  Serial.print(theta_target);
   Serial.print("       ");
  Serial.print("servo = ");
  Serial.print(servo_angle);
   Serial.print("       ");
   Serial.print("roll pwm = ");
  Serial.print(pwm2);
   Serial.print("       ");
  Serial.print("turret pwm = ");
  Serial.print(pwm);
  Serial.print("       ");
   Serial.print("omegi = ");
  Serial.print(omegi);
  Serial.println("               ");*/

//  if(x_target == prev_x_target && y_target == prev_y_target && z_target == prev_z_target && theta_target == prev_theta_target){
//   moving = false;
//  }

//  else{
//   moving = true;
//    }

//  if(moving){
//   move(x_target, y_target, z_target, theta_target);
// }

 move(x_target, y_target, z_target, theta_target);
  //moveShoulder(70);
  //moveElbow(110);

  prev_x_target = x_target;
  prev_y_target = y_target;
  prev_z_target = z_target;
  prev_theta_target = theta_target;


  delay(10);
}