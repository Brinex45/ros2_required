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
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/byte.h>

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

int Gripper_motor_pin=10;
int Gripper_dir_pin=11;

// 28_p 29_d

Cytron Turret( Turret_motor_pin, Turret_dir_pin, true);

Cytron Shoulder( Shoulder_motor_pin, Shoulder_dir_pin, 1);

Cytron Elbow( Elbow_motor_pin, Elbow_dir_pin, 1);

Cytron Wrist_pitch( Wrist_pitch_motor_pin, Wrist_pitch_dir_pin, 1);

Cytron Wrist_roll( Wrist_roll_motor_pin, Wrist_roll_dir_pin, true);

Cytron Gripper(Gripper_motor_pin, Gripper_dir_pin, 1);

Encoder Turret_encoder(30,31);
Encoder Shoulder_encoder(40,41); // 35 34
Encoder Elbow_encoder(37,36); //36 37  39 38
Encoder Wrist_pitch_encoder(32,33); 
Encoder Wrist_roll_encoder(26,27);


rcl_publisher_t publisher;
rcl_publisher_t publisher_kill_switch;
rcl_subscription_t subscription;
rcl_subscription_t subscription_encoder_reset;

std_msgs__msg__Float32MultiArray joint_pwm;
std_msgs__msg__Int64MultiArray encoder_data;
std_msgs__msg__Byte kill_switch_state;
std_msgs__msg__Bool encoder_reset_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

int64_t encoder_buffer[5];
float joint_pwm_buffer[6];

int shoulder_kill_switch = 7;
int elbow_kill_switch = 8;
int wrist_pitch_kill_switch = 9;

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

  for (size_t i = 0; i < 6; i++) {
    joint_pwm.data.data[i] = msg->data.data[i];
  }

}

void encoder_reset_callback(const void * msgin)
{
  if (msgin == NULL) {
    return;
  }

  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;

  if (msg->data) {
    Elbow_encoder.write(0);
    Shoulder_encoder.write(0);
    Wrist_pitch_encoder.write(0);
  }

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

    // Read kill switch states
    kill_switch_state.data = 0;
    if (digitalRead(shoulder_kill_switch) == LOW) {
      kill_switch_state.data |= 0b00000001;
    }
    if (digitalRead(elbow_kill_switch) == LOW) {
      kill_switch_state.data |= 0b00000010;
    }
    if (digitalRead(wrist_pitch_kill_switch) == LOW) {
      kill_switch_state.data |= 0b00000100;
    }
    
    encoder_data.data.data[0] = Elbow_encoder.read();
    encoder_data.data.data[1] = Shoulder_encoder.read();
    encoder_data.data.data[2] = Turret_encoder.read();
    encoder_data.data.data[3] = Wrist_pitch_encoder.read();
    encoder_data.data.data[4] = Wrist_roll_encoder.read();
    
    RCSOFTCHECK(rcl_publish(&publisher, &encoder_data, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_kill_switch, &kill_switch_state, NULL));
    // RCSOFTCHECK(rcl_publish(&pub, &joint_pwm, NULL));
    
  }
}

void setup() {
  // Configure serial transport
  delay(2000);
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
  
  encoder_data.data.data = encoder_buffer;
  encoder_data.data.size = 5;
  encoder_data.data.capacity = 5;

  joint_pwm.data.data = joint_pwm_buffer;
  joint_pwm.data.size = 6;
  joint_pwm.data.capacity = 6;


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

  RCCHECK(rclc_publisher_init_default(
    &publisher_kill_switch,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Byte),
    "/kill_switch_state"));

  // create subscription
  RCCHECK(rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/joints_command"));

  RCCHECK(rclc_subscription_init_default(
    &subscription_encoder_reset,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/encoder_reset"));

  // create timer,
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(
      &executor,
      &subscription,
      &joint_pwm,
      &subscription_callback,
      ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
      &executor,
      &subscription_encoder_reset,
      &encoder_reset_msg,
      &encoder_reset_callback,
      ON_NEW_DATA));

    digitalWrite(13, HIGH); // Indicate micro-ROS is running

}

void loop() {
  // delay(100);
  
  // if (got_cmd) {
    Elbow.rotate(-joint_pwm.data.data[0]);
    Shoulder.rotate(-joint_pwm.data.data[1]);
    Turret.rotate(-joint_pwm.data.data[2]);
    Wrist_pitch.rotate(-joint_pwm.data.data[3]);
    Wrist_roll.rotate(-joint_pwm.data.data[4]);
    Gripper.rotate(-joint_pwm.data.data[5]);
    // }
    
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  
}