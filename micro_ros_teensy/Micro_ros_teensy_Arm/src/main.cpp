#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <Cytron.h>
#include <Encoder.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// #include <std_msgs/msg/int8_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int64_multi_array.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

int Turret_motor_pin=0;
int Turret_dir_pin=1;

int Shoulder_motor_pin=2;
int Shoulder_dir_pin=3;

int Elbow_motor_pin=7;
int Elbow_dir_pin=6;

int Wrist_pitch_motor_pin=5;
int Wrist_pitch_dir_pin=4;

int Wrist_roll_motor_pin=8;
int Wrist_roll_dir_pin=9;

Cytron Turret( Turret_motor_pin, Turret_dir_pin, true);

Cytron Shoulder( Shoulder_motor_pin, Shoulder_dir_pin, true);

Cytron Elbow( Elbow_motor_pin, Elbow_dir_pin, true);

Cytron Wrist_pitch( Wrist_pitch_motor_pin, Wrist_pitch_dir_pin, false);

Cytron Wrist_roll( Wrist_roll_motor_pin, Wrist_roll_dir_pin, true);

// Cytron* motors[5] = {
//   &Elbow,
//   &Shoulder,
//   &Turret,
//   &Wrist_pitch,
//   &Wrist_roll
// };


// int cpr[5] = {8192, 8192, 460000, 8192, 8192};

// int pwms[5] = {35, 200, 30, 12, 1};

// int limits[5] = {100, 255, 70, 40, 40};

Encoder Turret_encoder(30,31);
Encoder Shoulder_encoder(34,35);
Encoder Elbow_encoder(38,39);
Encoder Wrist_pitch_encoder(41,40);
Encoder Wrist_roll_encoder(26,27);


rcl_publisher_t publisher;
// rcl_publisher_t pub;
rcl_subscription_t subscription;

std_msgs__msg__Float32MultiArray joint_pwm;
std_msgs__msg__Int64MultiArray encoder_data;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

int64_t encoder_buffer[5];
float joint_pwm_buffer[5];

volatile bool got_cmd = false;

int error1;
int error2;

int prev1 = 0;
int prev2 = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  digitalWrite(13, HIGH);
  delay(1000);
}

void subscription_callback(const void * msgin)
{
  if (msgin == NULL) {
    return;
  }

  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *) msgin;

  for (size_t i = 0; i < 5; i++) {
    joint_pwm.data.data[i] = msg->data.data[i];
  }

  // got_cmd = true;
  
  // Set motor speeds
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    encoder_data.data.data[0] = Elbow_encoder.read();
    encoder_data.data.data[1] = Shoulder_encoder.read();
    encoder_data.data.data[2] = Turret_encoder.read();
    encoder_data.data.data[3] = Wrist_pitch_encoder.read();
    encoder_data.data.data[4] = Wrist_roll_encoder.read();
    
    RCSOFTCHECK(rcl_publish(&publisher, &encoder_data, NULL));
    // RCSOFTCHECK(rcl_publish(&pub, &joint_pwm, NULL));
    
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
  
  // while (error1 != 0 ) {
    
  //   Shoulder.rotate(-255);
    
  //   delay(500);
    
  //   float Shoulder = (Shoulder_encoder.read() * (360.0 / 8192.0));
    
  //   error1 = Shoulder - prev1;
    
  //   prev1 = Shoulder;
  // } 
  
  // Shoulder.rotate(0);
  
  // while (error2 != 0 ) {
    
  //   Elbow.rotate(50);
  //   delay(500);
    
  //   float Elbow = (Elbow_encoder.read() * (360.0 / 8192.0));
    
  //   error2 = Elbow - prev2;
    
  //   prev2 = Elbow;
    
  // }
  // Elbow.rotate(0);
  
  // Shoulder_encoder.write(0);
  // Elbow_encoder.write(0);
  
  // digitalWrite(13,HIGH);
  
  encoder_data.data.data = encoder_buffer;
  encoder_data.data.size = 5;
  encoder_data.data.capacity = 5;

  joint_pwm.data.data = joint_pwm_buffer;
  joint_pwm.data.size = 5;
  joint_pwm.data.capacity = 5;


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arm", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray),
    "/arm_encoders"));

  // RCCHECK(rclc_publisher_init_default(
  //   &pub,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  //   "/joint_data"));

  // create subscription
  RCCHECK(rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/joints_command"));

  // create timer,
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

    
    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(
      &executor,
      &subscription,
      &joint_pwm,
      &subscription_callback,
      ON_NEW_DATA));

}

void loop() {
  // delay(100);
  
  // if (got_cmd) {
    Elbow.rotate(-joint_pwm.data.data[0]);
    Shoulder.rotate(-joint_pwm.data.data[1]);
    Turret.rotate(-joint_pwm.data.data[2]);
    Wrist_pitch.rotate(joint_pwm.data.data[3]);
    Wrist_roll.rotate(-joint_pwm.data.data[4]);
    // }
    
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  // for (size_t i = 0; i < 5; i++)
  // {
  //   float angle = (encoder_data.data.data[i] / cpr[i]) * 360;
  //   if (angle < joint_pwm.data.data[i]){
  //     motors[i]->rotate( constrain((angle - joint_pwm.data.data[i]) * pwms[i], -limits[i], limits[i]));
  //   }
  //   else if (angle > joint_pwm.data.data[i]){
  //     motors[i]->rotate(- constrain((angle - joint_pwm.data.data[i]) * pwms[i], -limits[i], limits[i]));
  //   } else {
  //     motors[i]->rotate(0);
  //   }
  // }
  

}