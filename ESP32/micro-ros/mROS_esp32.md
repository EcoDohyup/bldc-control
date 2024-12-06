# mROS on ESP32 board

## Installation and initial configuration
```bash
source /opt/ros/humble/setup.bash

# create ws and download tools
mkdir ws
cd ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

#build mROS tools and source them
colcon build
source install/local_setup.bash

# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

```

## create a mROS agent and connection
```bash

# Testing installation - Run and connect the micro-ros and the board
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# Check topic, node
ros2 node list
ros2 topic list
ros2 topic echo /topic_name

```

### [A boilerplate for mROS for ESP32](https://gist.github.com/rasheeddo/5a6dd95b206233ad58bda8304ae2f30d)
- 즉시 사용 가능
```cpp  
/*
 * This is a simple template to use microros with ESP32-Arduino
 * This sketch has sample of publisher to publish from timer_callback
 * and subscrption to listen of new data from other ROS node.
 * 
 * Some of the codes below are gathered from github and forums
 * 
 * Made by Rasheed Kittinanthapanya
 * 
*/


/*
 * TODO : Include your necessary header here
*/


/////////////////////
/// For Micro ROS ///
/////////////////////
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
/*
 * TODO : include your desired msg header file
*/
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <sensor_msgs/msg/imu.h>

/*
 * Optional,
 * LED pin to check connection
 * between micro-ros-agent and ESP32
*/
#define LED_PIN 23
#define LED_PIN_TEST 18 // subscription LED
bool led_test_state = false;

/*
 * Helper functions to help reconnect
*/
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

/*
 * Declare rcl object
*/
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

/*
 * TODO : Declare your 
 * publisher & subscription objects below
*/
rcl_publisher_t int16_pub;
rcl_publisher_t int16array_pub;
rcl_subscription_t led_sub;
rcl_subscription_t int16array_sub;
/*
 * TODO : Define your necessary Msg
 * that you want to work with below.
*/
std_msgs__msg__Int16 int16_msg;
std_msgs__msg__Int16MultiArray int16array_send_msg;
std_msgs__msg__Int16MultiArray int16array_recv_msg;
std_msgs__msg__Bool led_msg;

/*
 * TODO : Define your subscription callbacks here
 * leave the last one as timer_callback()
*/
void led_callback(const void *msgin) {

  std_msgs__msg__Bool * led_msg = (std_msgs__msg__Bool *)msgin;
  /*
   * Do something with your receive message
   */
  led_test_state = led_msg->data;
  digitalWrite(LED_PIN_TEST, led_test_state);

}
void int16array_callback(const void *msgin) {

  std_msgs__msg__Int16MultiArray * int16array_recv_msg = (std_msgs__msg__Int16MultiArray *)msgin;
  /*
   * Do something with your receive message
   */
  int16array_send_msg.data.data[0] = int16array_recv_msg->data.data[0];
  int16array_send_msg.data.data[1] = int16array_recv_msg->data.data[1];

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {

    /*
       TODO : Publish anything inside here
       
       For example, we are going to echo back
       the int16array_sub data to int16array_pub data,
       so we could see the data reflect each other.

       And also keep incrementing the int16_pub
    */

    rcl_publish(&int16array_pub, &int16array_send_msg, NULL);
    
    int16_msg.data++;
    rcl_publish(&int16_pub, &int16_msg, NULL);
    

  }
}

/*
   Create object (Initialization)
*/
bool create_entities()
{
  /*
     TODO : Define your
     - ROS node name
     - namespace
     - ROS_DOMAIN_ID : 사용중인 ros2 시스템과 같아야
  */
  const char * node_name = "esp32_boiler_plate_node";
  const char * ns = "";
  const int domain_id = 0;
  
  /*
   * Initialize node
   */
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, domain_id);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, node_name, ns, &support);

  
  /*
   * TODO : Init your publisher and subscriber 
   */
  rclc_publisher_init(
    &int16array_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/esp/int16array_pub", &rmw_qos_profile_default);

  rclc_publisher_init(
    &int16_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/esp/int_pub", &rmw_qos_profile_default);

  rclc_subscription_init(
    &int16array_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/esp/int16array_sub", &rmw_qos_profile_default);

  rclc_subscription_init(
    &led_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/esp/led", &rmw_qos_profile_default);

  /*
   * Init timer_callback
   * TODO : change timer_timeout
   * 50ms : 20Hz
   * 20ms : 50Hz
   * 10ms : 100Hz
   */
  const unsigned int timer_timeout = 50;
  rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  /*
   * Init Executor
   * TODO : make sure the num_handles is correct
   * num_handles = total_of_subscriber + timer
   * publisher is not counted
   * 
   * TODO : make sure the name of sub msg and callback are correct
   */
  unsigned int num_handles = 3;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_subscription(&executor, &int16array_sub, &int16array_recv_msg, &int16array_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  return true;
}
/*
 * Clean up all the created objects
 */
void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_init_options_fini(&init_options);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  /*
   * TODO : Make sue the name of publisher and subscriber are correct
   */
  rcl_publisher_fini(&int16array_pub, &node);
  rcl_publisher_fini(&int16_pub, &node);
  rcl_subscription_fini(&int16array_sub, &node);
  rcl_subscription_fini(&led_sub, &node);
  
}

void setup() {
  /*
   * TODO : select either of USB or WiFi 
   * comment the one that not use
   */
  set_microros_transports();
  //set_microros_wifi_transports("WIFI-SSID", "WIFI-PW", "HOST_IP", 8888);

  /*
   * Optional, setup output pin for LEDs
   */
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_TEST, OUTPUT);

  /*
   * TODO : Initialze the message data variable
   */
  int16_t array_data[2] = {0, 0};
  int16array_send_msg.data.capacity = 2; // number of array length
  int16array_send_msg.data.size = 2;     // number of array length
  int16array_send_msg.data.data = array_data;

  int16array_recv_msg.data.capacity = 2; // number of array length
  int16array_recv_msg.data.size = 2;     // number of array length
  int16array_recv_msg.data.data = array_data;

  int16_msg.data = 0;

  led_msg.data = false;

  /*
   * Setup first state
   */
  state = WAITING_AGENT;

}

void loop() {
  /*
   * Try ping the micro-ros-agent (HOST PC), then switch the state 
   * from the example
   * https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
   * 
   */
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
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
  /*
   * Output LED when in AGENT_CONNECTED state
   */
  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }

  /*
   * TODO : 
   * Do anything else you want to do here,
   * like read sensor data,  
   * calculate something, etc.
   */

}
```


### motor control successful
```cpp
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <ESP32Servo.h>

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define MotorPin 12                 // Pin connected to the BLDC motor ESC signal wire
#define LED_PIN1 21
#define LED_PIN2 19
#define LED_PIN3 13                   // LED pin for error indication
#define CCW_MAX_BLDC_Speed 1860     // Counter-Clockwise maximum speed
#define CW_MAX_BLDC_Speed 1060      // Clockwise maximum speed
#define BLDC_Center 1500            // Stop position


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

class BLDC_Motor {
  Servo servo;

public:
  BLDC_Motor() {}

  void Attach(int pin) {
    servo.attach(pin, 1000, 2000);  // Attach servo with PWM range
    servo.write(BLDC_Center);       // Set motor to stop position
    delay(100);                     // Delay to ensure ESC processes the signal
  }

  void setSpeed(int speed) {
    if (speed >= CW_MAX_BLDC_Speed && speed <= CCW_MAX_BLDC_Speed) {
      servo.write(speed);           // Send speed command to ESC
    }
  }
};

BLDC_Motor bldcMotor;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN1, !digitalRead(LED_PIN1));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN1, (msg->data == 0) ? LOW : HIGH);  

  int speed = msg->data;

  // Clamp speed to valid range
  if (speed < CW_MAX_BLDC_Speed) speed = CW_MAX_BLDC_Speed;
  if (speed > CCW_MAX_BLDC_Speed) speed = CCW_MAX_BLDC_Speed;

  // Set motor speed
  bldcMotor.setSpeed(speed);
  
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN1, OUTPUT);
  digitalWrite(LED_PIN1, HIGH);  
  pinMode(LED_PIN2, OUTPUT);
  digitalWrite(LED_PIN2, HIGH);  
  pinMode(LED_PIN3, OUTPUT);
  digitalWrite(LED_PIN3, HIGH);  

  // initializing motor
  bldcMotor.Attach(MotorPin);
  delay(2000);


  digitalWrite(LED_PIN1, 0);  
  delay(1000);
  digitalWrite(LED_PIN2, 0);
  delay(1000);
  digitalWrite(LED_PIN3, 0);
  delay(1000);


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_subscriber"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  digitalWrite(LED_PIN1, 1);  
  digitalWrite(LED_PIN2, 1);
  digitalWrite(LED_PIN3, 1);
  delay(1000);
  digitalWrite(LED_PIN1, 0);  
  digitalWrite(LED_PIN2, 0);
  digitalWrite(LED_PIN3, 0);
  delay(1000);
  
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
```