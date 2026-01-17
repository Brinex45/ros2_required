#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <Cytron.h>
#include <Encoder.h>
#include <Wire.h>
// #include <Adafruit_PWMServoDriver.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <irc_interfaces/msg/ps4.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int64_multi_array.h>
#include <rmw/qos_profiles.h>

// Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40, Wire2);

// #define SERVO_CHANNEL_x 1
// #define SERVO_CHANNEL_y 2
// #define SERVO_MIN 150
// #define SERVO_MAX 600

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

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

// #define slave_addr 9

rcl_timer_t encoder_readings_timer;
rcl_timer_t comm_chk_timer;

rcl_subscription_t subscription;
rcl_subscription_t subscription_arm;
rcl_subscription_t subscription_rover;
rcl_subscription_t comm_check_sub_;
rcl_subscription_t reset_sub_;

rcl_publisher_t encoder_pub_;

irc_interfaces__msg__Ps4 ps4_msg;
std_msgs__msg__Bool reset;
std_msgs__msg__Bool check_;
std_msgs__msg__Int64MultiArray encoder_data;
int64_t encoder_buf[6];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  // digitalWrite(13, HIGH);
  delay(1000);
}

int x_servo_angle = 90;
int y_servo_angle = 90;

// //used in IK calculation
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

// for data received from nav teensy
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

double R_joystick_x_rover;
double R_joystick_y_rover;

bool right;
bool down;
bool up;
bool left;
bool square;
bool cross;
bool circle;
bool triangle;


//   //for joint encoder angles
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

long start_time = 0;
int count = 4;
bool flag = false;

void enc_pub_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    encoder_data.data.data[0] = elbow.read();
    encoder_data.data.data[1] = shoulder.read();
    encoder_data.data.data[2] = 0;
    encoder_data.data.data[3] = wrist.read();
    encoder_data.data.data[4] = 0;
    encoder_data.data.data[5] = 0;
    RCSOFTCHECK(rcl_publish(&encoder_pub_, &encoder_data, NULL));
  }
}

void comm_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    count--;
  }
}

void subscription_callback_arm(const void * msgin)
{
  if (msgin == NULL) {
    return;
  }
  
  const irc_interfaces__msg__Ps4 * msg = (const irc_interfaces__msg__Ps4 *) msgin;
  
  // for (size_t i = 0; i < 6; i++) {
    //   joint_pwm.data.data[i] = msg->data.data[i];
    // }
    
    L_joystick_x = double(msg->ps4_data_analog[0]);
    L_joystick_y = double(msg->ps4_data_analog[1]);
    R_joystick_y = double(msg->ps4_data_analog[4]);

    cross = msg->ps4_data_buttons[0];
    circle = msg->ps4_data_buttons[1];  
    triangle = msg->ps4_data_buttons[2];
    square = msg->ps4_data_buttons[3];
    
    up = msg->ps4_data_buttons[13];
    right = msg->ps4_data_buttons[15];
    down = msg->ps4_data_buttons[14];
    left = msg->ps4_data_buttons[16];
    
    // motorRoll.rotate(50);
    digitalWrite(13, HIGH);
}

void comm_check_callback(const void * msgin){

  if (msgin == NULL) {
    return;
  }

  const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*) msgin;

  count = 4;
}

void subscription_callback_rover(const void * msgin)
{
  if (msgin == NULL) {
    return;
  }

  const irc_interfaces__msg__Ps4 * msg = (const irc_interfaces__msg__Ps4 *) msgin;

  // L_joystick_x = double(msg->ps4_data_analog[0]);
  // L_joystick_y = double(msg->ps4_data_analog[1]);
  R_joystick_x_rover = double(msg->ps4_data_analog[3]);
  R_joystick_y_rover = double(msg->ps4_data_analog[4]);

  // cross = msg->ps4_data_buttons[0];
  // circle = msg->ps4_data_buttons[1];
  // triangle = msg->ps4_data_buttons[2];
  // square = msg->ps4_data_buttons[3];
  // up = msg->ps4_data_buttons[13];
  // right = msg->ps4_data_buttons[15];
  // down = msg->ps4_data_buttons[14];
  // left = msg->ps4_data_buttons[16];
  
  // digitalWrite(13, HIGH);
}

// void cam_servo_rotate(int channel, int angle) {
//   int pwm_servo = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
//   pca.setPWM(channel, 0, pwm_servo);
// }

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

void reset_callback(const void * msgin){

  if (msgin == NULL) {
    return;
  }
  
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;

  if(msg->data){
    home();
    turret.write(0);
    elbow.write(0);
    wrist.write(0);
    shoulder.write(0);
  }
}

void init_microros()
{
  set_microros_serial_transports(Serial);

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

  // Wait for agent successful ping for 5 seconds.
  const int timeout_ms = 1000;
  const uint8_t attempts = 5;

  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

  if (ret == RMW_RET_OK)
  {
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arm", "", &support));

    RCCHECK(rclc_subscription_init(
      &subscription_arm,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(irc_interfaces, msg, Ps4),
      "/ps4_data_arm",
      &qos_profile));

    RCCHECK(rclc_subscription_init(
      &subscription_rover,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(irc_interfaces, msg, Ps4),
      "/ps4_data_rover",
      &qos_profile));

    RCCHECK(rclc_subscription_init_default(
      &comm_check_sub_,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/comm_check"));

    RCCHECK(rclc_subscription_init_default(
      &reset_sub_,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/reset_arm"));

    RCCHECK(rclc_publisher_init_default(
      &encoder_pub_,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray),
      "/arm_encoder"));

    // create timer,
    const unsigned int timer_timeout = 10;
    RCCHECK(rclc_timer_init_default(
      &encoder_readings_timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      enc_pub_callback));

    // create timer,
    const unsigned int comm_check_timer = 1000;
    RCCHECK(rclc_timer_init_default(
      &comm_chk_timer,
      &support,
      RCL_MS_TO_NS(comm_check_timer),
      comm_callback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));

    RCCHECK(rclc_executor_add_timer(
      &executor, 
      &encoder_readings_timer));

    RCCHECK(rclc_executor_add_timer(
      &executor, 
      &comm_chk_timer));
    
    RCCHECK(rclc_executor_add_subscription(
      &executor,
      &subscription_arm,
      &ps4_msg,
      &subscription_callback_arm,
      ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
      &executor,
      &comm_check_sub_,
      &check_,
      &comm_check_callback,
      ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
      &executor,
      &subscription_rover,
      &ps4_msg,
      &subscription_callback_rover,
      ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
      &executor,
      &reset_sub_,
      &reset,
      &reset_callback,
      ON_NEW_DATA));

    encoder_data.data.data = encoder_buf;
    encoder_data.data.size = 6;
    encoder_data.data.capacity = 6;
    flag = true;
  }

}

void destroy_microros()
{
  rcl_timer_fini(&encoder_readings_timer);
  rcl_timer_fini(&comm_chk_timer);

  rcl_subscription_fini(&subscription_arm, &node);
  rcl_subscription_fini(&comm_check_sub_, &node);
  rcl_subscription_fini(&subscription_rover, &node);
  rcl_subscription_fini(&reset_sub_, &node);

  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  if (encoder_data.data.data != NULL)
  {
    free(encoder_data.data.data);
    encoder_data.data.data = NULL;
  }
  flag = false;
}

void setup() {

  delay(2000);
  Serial.begin(115200);
  delay(2000);


  init_microros();

  pinMode(13, OUTPUT);
  
  home();
  turret.write(0);
  elbow.write(0);
  wrist.write(0);
  shoulder.write(0);
  
  start_time = millis();

  // digitalWrite(13, HIGH);
}

void loop() {

  if (count == 0) {
    if(flag == true){
      destroy_microros();
    }
    // init_microros();
    start_time = millis();

    L_joystick_x = 0;
    L_joystick_y = 0;
    R_joystick_y = 0;

    cross = 0;
    circle = 0;
    triangle = 0;
    square = 0;
    
    up = 0;
    right = 0;
    down = 0;
    left = 0;
    count = 3;
  }

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

  // if(abs(R_joystick_x_rover) >= 20){
  //   x_servo_angle += R_joystick_x_rover * 0.1;
  //   x_servo_angle = constrain(x_servo_angle, 0, 180);
  // }
  // if(abs(R_joystick_y_rover) >= 20){
  //   y_servo_angle += R_joystick_y_rover * 0.1;
  //   y_servo_angle = constrain(y_servo_angle, 0, 180);
  // }
  // cam_servo_rotate(SERVO_CHANNEL_x, x_servo_angle);
  // cam_servo_rotate(SERVO_CHANNEL_y, y_servo_angle);

  move(x_target, y_target, z_target, theta_target);

  prev_x_target = x_target;
  prev_y_target = y_target;
  prev_z_target = z_target;
  prev_theta_target = theta_target;

  
  // delay(10);

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}