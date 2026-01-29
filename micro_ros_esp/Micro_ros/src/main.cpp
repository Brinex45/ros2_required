#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <Cytron.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP085.h>

#include <irc_interfaces/msg/ps4.h>

#include <std_msgs/msg/float64_multi_array.h>

#define LED_PIN 2
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

TwoWire Wire2 = TwoWire(1);

ESP32Encoder enc;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire2);
Adafruit_AHTX0 aht21b;
Adafruit_BMP085 hw415;

double sensor_buf[7];

// PWM
#define MAX 100
#define PWM1 25
#define PWM2 75

// rhino
#define pwm1 15
#define dir1 5

// pump
#define pwm2 2
#define dir2 4
#define pwm3 27
#define dir3 14
#define pwm4 19
#define dir4 18

// interrupt
#define encA 25
#define encB 26

// pca - servo
// i2c
#define sda 33
#define scl 32
// pulse
#define servomin 150
#define servomax 650
// servo pin
#define servo1 15
#define servo2 14

Cytron m1(pwm1, dir1, 0, MAX);
Cytron m2(pwm2, dir2, 1, MAX);
Cytron m3(pwm3, dir3, 0, MAX);
Cytron m4(pwm4, dir4, 1, MAX);

#define mq2 34
#define mq7 35

#define sda_0 21
#define scl_0 22

rcl_publisher_t publisher;
rcl_subscription_t subscription_rover;

irc_interfaces__msg__Ps4 ps4_msg;

rcl_node_t node;
rcl_timer_t timer;
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;

std_msgs__msg__Float64MultiArray sensor_data;

bool micro_ros_init_successful;

bool right;
bool down;
bool up;
bool left;
bool square;
bool cross;
bool circle;
bool triangle;

// flags
bool tflag = false;
bool pflag = false;
bool cflag = false;
bool sflag = false;

// encoder
long error;
long nerror;
long count = 0;

// angle
int angle_1 = 0;
int angle_2 = 0;
int s1angle_1 = 3;
int s1angle_2 = 35;
int s2angle_1 = 18;
int s2angle_2 = 55;

//flap time
unsigned long servo = 0;
const unsigned long flap_1 = 200;
const unsigned long flap_2 = 400;

unsigned long sensor_start = 0;
const unsigned long sensor_stop = 500;

// pump
unsigned long start = 0;
const unsigned long time1 = 275;
const unsigned long time2 = 555;
const unsigned long time3 = 825;

float mq2data;
float mq7data;

float mq2std = 1800;
float mq7std = 1950;

float mq2index;
float mq7index;

float ahttemp;
float ahthumid;
float hwtemp;
float hwpressure;
float hwalti;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {

    sensor_data.data.data[0] = mq2index;
    sensor_data.data.data[1] = mq7index;
    sensor_data.data.data[2] = ahttemp;
    sensor_data.data.data[3] = ahthumid;
    sensor_data.data.data[4] = hwtemp;
    sensor_data.data.data[5] = hwpressure;
    sensor_data.data.data[6] = hwalti;

    rcl_publish(&publisher, &sensor_data, NULL);
  }
}

void subscription_callback_rover(const void * msgin)
{
  const irc_interfaces__msg__Ps4 * msg = (const irc_interfaces__msg__Ps4 *)msgin;

  up = msg->ps4_data_buttons[13];
  right = msg->ps4_data_buttons[15];
  down = msg->ps4_data_buttons[14];
  left = msg->ps4_data_buttons[16];
}

// angle -> pulse
int angle2pulse(int angle) {
  return map(angle, 0, 180, servomin, servomax);
}

// set servo
void moveservo(int servo_num, int target_angle) {
  switch (servo_num) {
    case 1:
      pwm.setPWM(servo1, 0, angle2pulse(target_angle));
      break;
    case 2:
      pwm.setPWM(servo2, 0, angle2pulse(target_angle));
      break;
    default:
      break;
  }
}

// encoder count
void enccount() {
  count = enc.getCount();
  count = count % 77488;
  count = map(count, 0, 77488, 0, 360);
  count = count < 0 ? count + 360 : count;
}

// setangle
void setangle(int pos) {
  error = pos - count;
  error = error < 0 ? error + 360 : error;
  nerror = 360 - abs(error);

  if (nerror > error && abs(error) > 1) {
    m1.rotate(-PWM1);
  } else if (error > nerror && abs(error) > 1) {
    m1.rotate(PWM1);
  } else if (abs(error) < 1) {
    m1.rotate(0);
  } else {
    m1.rotate(0);
  }
}

void pump() {
  if (circle) {
    pflag = true;
    // if (!pflag) {
    //   start = millis();
    //   pflag = true;
    //   // m2.rotate(PWM2);
    //   // m3.rotate(PWM2);
    //   m4.rotate(PWM2);
    //   delay(200);
    // m4.rotate(0);
    //   Serial.println("MOTOR ON");
  }

  if (pflag && !circle) {
    m2.rotate(PWM2);
    m3.rotate(PWM2);
    m4.rotate(PWM2);
    delay(200);
    m2.rotate(0);
    delay(200);
    m3.rotate(0);
    delay(300);
    m4.rotate(0);
    Serial.println("MOTOR ON");
    pflag = false;
  }


  if (!circle) {
    m4.rotate(0);
  }

  // if (pflag) {
  //   if (millis() - start >= time1) {
  //     m2.rotate(0);
  //   }
  //   if (millis() - start >= time2) {
  //     m3.rotate(0);
  //   }
  //   if (millis() - start >= time3) {
  //     m4.rotate(0);
  //   }
  //   Serial.println("MOTOR OFF");
  //   pflag = false;
  // }
}

// ps command
void psdata1() {
  //set disk
  if (square) {
    if (!sflag) {
      sflag = true;
      angle_1 = (angle_1 + 45) % 360;
    }
  }
  if (!square) {
    sflag = false;
  }
  setangle(angle_1);
}

void psdata3() {
  //servo
  if (triangle) {
    if (!tflag) {
      servo = millis();
      tflag = true;
      moveservo(1, s1angle_1);
      moveservo(2, s2angle_2);
    }
  }
  if (tflag) {
    if (millis() - servo >= flap_1) {
      moveservo(1, s1angle_2);
    }
    if (millis() - servo >= flap_2) {
      moveservo(2, s2angle_1);
      tflag = false;
    }
  }
}

void sensordata() {
  if (millis() - sensor_start >= sensor_stop) {
    sensor_start = millis();

    mq2data = analogRead(mq2);
    mq7data = analogRead(mq7);

    mq2index = mq2data / mq2std;
    mq7index = mq7data / mq7std;

    sensors_event_t humidityEvent, tempEvent;
    aht21b.getEvent(&humidityEvent, &tempEvent);
    ahttemp = tempEvent.temperature;
    ahthumid = humidityEvent.relative_humidity;

    hwtemp = hw415.readTemperature();
    hwpressure = hw415.readPressure();
    hwalti = hw415.readAltitude(101600);
  }
}

bool create_entities()
{

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "astro", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "sensor_data"));

  RCCHECK(rclc_subscription_init(
    &subscription_rover,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(irc_interfaces, msg, Ps4),
    "/ps4_data_rover",
    &qos_profile));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscription_rover,
    &ps4_msg,
    &subscription_callback_rover,
    ON_NEW_DATA));

  sensor_data.data.size = 6;
  sensor_data.data.capacity = 6;
  sensor_data.data.data = sensor_buf;

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscription_rover, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(sda_0, scl_0);
  Wire2.begin(sda, scl);

  pwm.begin();
  pwm.setPWMFreq(60);

  set_microros_serial_transports(Serial);
  pinMode(LED_PIN, OUTPUT);

  state = WAITING_AGENT;
  
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
      case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(25));
      }
      break;
      case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
      default:
      break;
    }

  enccount();
  psdata1();
  psdata3();
  pump();

  sensordata();
}