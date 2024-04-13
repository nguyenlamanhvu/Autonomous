#include <mainpp.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

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
ros::Publisher mpu9250("imu/data_raw", &imu_msg);
sensor_msgs::MagneticField mag_msg;
ros::Publisher magnetic("imu/mag",&mag_msg);

// Setup node:
void setup(void) {
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(mpu9250);
  nh.advertise(magnetic);
  nh.subscribe(led_sub);
}

// Loop:
void loop(void) {
  // Publish message:
  str_msg.data = hello;
  chatter.publish(&str_msg);
  // Imu message:
  imu_msg.header.frame_id = "imu_link";
  imu_msg.linear_acceleration.x = imu_9250_0->pt1_p.acc_x;
  imu_msg.linear_acceleration.y = imu_9250_0->pt1_p.acc_y;
  imu_msg.linear_acceleration.z = imu_9250_0->pt1_p.acc_z;

  imu_msg.angular_velocity.x = imu_9250_0->pt1_p.gyro_x;
  imu_msg.angular_velocity.y = imu_9250_0->pt1_p.gyro_y;
  imu_msg.angular_velocity.z = imu_9250_0->pt1_p.gyro_z;
  imu_msg.header.stamp = nh.now();
  mpu9250.publish(&imu_msg);

  mag_msg.header.frame_id = "imu_link";
  mag_msg.magnetic_field.x = imu_9250_0->pt1_p.mag_x;
  mag_msg.magnetic_field.y = imu_9250_0->pt1_p.mag_y;
  mag_msg.magnetic_field.z = imu_9250_0->pt1_p.mag_z;
  mag_msg.header.stamp = nh.now();
  magnetic.publish(&mag_msg);

  nh.spinOnce();
  HAL_Delay(100);
}
