#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <Cytron.h>
#include <Encoder.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <irc_interfaces/msg/ps4.h>
#include <geometry_msgs/msg/twist.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

int Right_front_motor_pin=8;
int Right_front_dir_pin=9;

int Left_front_motor_pin=0;
int Left_front_dir_pin=1; 

int Right_middle_motor_pin=10;
int Right_middle_dir_pin=11;

int Left_middle_motor_pin=7;
int Left_middle_dir_pin=6;

int Right_back_motor_pin=22;
int Right_back_dir_pin=23;

int Left_back_motor_pin=5;
int Left_back_dir_pin=4;

Cytron R_F( Right_front_motor_pin, Right_front_dir_pin, 0);
Cytron L_F( Left_front_motor_pin, Left_front_dir_pin, true);

Cytron R_M( Right_middle_motor_pin, Right_middle_dir_pin, 0);
Cytron L_M( Left_middle_motor_pin, Left_middle_dir_pin, false);

Cytron R_B( Right_back_motor_pin, Right_back_dir_pin, 0);
Cytron L_B( Left_back_motor_pin, Left_back_dir_pin, 0);

rcl_publisher_t publisher;
rcl_subscription_t subscription;

irc_interfaces__msg__Ps4 ps4_msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

float_t linear_x = 0.0;
float_t angular_z = 0.0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{
  if (msgin == NULL) {
    return;
  }

  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;

  twist_msg = *msg; 
  
  linear_x = twist_msg.linear.x;
  angular_z = twist_msg.angular.z;

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    float_t right_speed = linear_x + angular_z;
    float_t left_speed = linear_x - angular_z;
    
    R_F.rotate(right_speed);
    R_M.rotate(right_speed);
    R_B.rotate(right_speed);

    L_F.rotate(left_speed);
    L_M.rotate(left_speed);
    L_B.rotate(left_speed);

  }
}

void setup() {
  // Configure serial transport
  delay(2000);
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_chassis", "", &support));
  
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(irc_interfaces, msg, Ps4),
    "micro_ros_platformio_node_publisher"));
    
    // create subscription
  RCCHECK(rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));
      
      // create timer,
  const unsigned int timer_timeout = 0.05;
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
    &twist_msg,
    &subscription_callback,
    ON_NEW_DATA));
    
    digitalWrite(13, HIGH); // Indicate micro-ROS is running
}

void loop() {
  // delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}