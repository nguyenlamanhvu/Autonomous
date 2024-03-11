#include <mainpp.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

// Node:
ros::NodeHandle nh;

// Publisher:
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";

// Subscriber:
void led_cb(const std_msgs::Empty &msg) {
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}
ros::Subscriber<std_msgs::Empty> led_sub("toggle_led", &led_cb);

// Publisher IMU:
sensor_msgs::Imu imu_msg;
ros::Publisher mpu9250("imu", &imu_msg);

// Setup node:
void setup(void) {
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(mpu9250);
  nh.subscribe(led_sub);
}

// Loop:
void loop(void) {
  // Publish message:
  str_msg.data = hello;
  chatter.publish(&str_msg);
  // Imu message:
  imu_msg.linear_acceleration.x = 0;
  imu_msg.linear_acceleration.y = 9.8;
  imu_msg.linear_acceleration.z = 0;
  mpu9250.publish(&imu_msg);

  nh.spinOnce();
  HAL_Delay(500);
}
