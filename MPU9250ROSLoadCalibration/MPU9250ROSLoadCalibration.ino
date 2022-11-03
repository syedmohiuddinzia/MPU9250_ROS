#include "MPU9250.h"
#include "eeprom_utils.h"
#include "ros.h"
#include <sensor_msgs/Temperature.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt64.h>

MPU9250 mpu;
ros::NodeHandle  nh;
sensor_msgs::Temperature temperature_msg;
ros::Publisher pub_temperature("/MPU9250_Temperature", &temperature_msg);
geometry_msgs::Vector3 magnetometer_msg;
ros::Publisher pub_magnetometer("/MPU9250_Magnetic_Field", &magnetometer_msg);
geometry_msgs::Vector3 accelerometer_msg;
ros::Publisher pub_acceleration("/MPU9250_Accelerometer", &accelerometer_msg);
geometry_msgs::Vector3 gyrometer_msg;
ros::Publisher pub_gyrometer("/MPU9250_Gyrometer", &gyrometer_msg);
geometry_msgs::Vector3 lin_acc_msg;
ros::Publisher pub_lin_acc("/MPU9250_Linear_Acceleration", &lin_acc_msg);
geometry_msgs::Vector3 ypr_msg;
ros::Publisher pub_ypr("/MPU9250_Yaw_Pitch_Role", &ypr_msg);
geometry_msgs::Vector3 euler_msg;
ros::Publisher pub_euler("/MPU9250_Euler_Transform", &euler_msg);
geometry_msgs::Quaternion quaternion_msg;
ros::Publisher pub_quaternion("/MPU9250_Quaternion_Transform", &quaternion_msg);

std_msgs::UInt64 heading_msg;
ros::Publisher pub_heading("/MPU9250_Heading", &heading_msg);

float declinationAngle = 0.02234021, heading; // Magnetic Declination in Karachi is 0.02234021 radians, 1.28 degrees.
int headingDegrees;

void setup() {
    nh.initNode();
    nh.advertise(pub_temperature);
    nh.advertise(pub_acceleration);
    nh.advertise(pub_magnetometer);
    nh.advertise(pub_gyrometer);
    nh.advertise(pub_lin_acc);
    nh.advertise(pub_ypr);
    nh.advertise(pub_euler);
    nh.advertise(pub_quaternion);
    nh.advertise(pub_heading);
    
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {delay(5000);}
    }
        printCalibration();
        loadCalibration();
}

void mpu9250(){ 
      if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            magnetometer();
            accelerometer();
            gyrometer();
            linear_acc();
            quaternion();
            euler();
            roll_pitch_yaw();
            temperature();
            mag_heading();
            prev_ms = millis();
        }
    }
}

void ROS() {
   nh.spinOnce();
}

void loop() {
   mpu9250();
   ROS();
}

void accelerometer() {
  accelerometer_msg.x = mpu.getAccX();
  accelerometer_msg.y = mpu.getAccY();
  accelerometer_msg.z = mpu.getAccZ();
  pub_acceleration.publish(&accelerometer_msg);
}

void gyrometer() {
  gyrometer_msg.x = mpu.getGyroX();
  gyrometer_msg.y = mpu.getGyroX();
  gyrometer_msg.z = mpu.getGyroX();
  pub_gyrometer.publish(&gyrometer_msg);
}

void magnetometer() {
  magnetometer_msg.x = mpu.getMagX();
  magnetometer_msg.y = mpu.getMagY();
  magnetometer_msg.z = mpu.getMagZ();
  pub_magnetometer.publish(&magnetometer_msg);
}

void linear_acc() {
  lin_acc_msg.x = mpu.getLinearAccX();
  lin_acc_msg.y = mpu.getLinearAccY();
  lin_acc_msg.z = mpu.getLinearAccZ();
  pub_lin_acc.publish(&lin_acc_msg);
}

void quaternion() {
  quaternion_msg.x = mpu.getQuaternionX();
  quaternion_msg.y = mpu.getQuaternionY();
  quaternion_msg.z = mpu.getQuaternionZ();
  quaternion_msg.w = mpu.getQuaternionW();
  pub_quaternion.publish(&quaternion_msg);
}

void euler() {
  euler_msg.x = mpu.getEulerX();
  euler_msg.y = mpu.getEulerY();
  euler_msg.z = mpu.getEulerZ();
  pub_euler.publish(&euler_msg);
}

void roll_pitch_yaw() {
  ypr_msg.x = mpu.getYaw();
  ypr_msg.y = mpu.getPitch();
  ypr_msg.z = mpu.getRoll();
  pub_ypr.publish(&ypr_msg);
}

void temperature(){
  temperature_msg.temperature = mpu.getTemperature();
  pub_temperature.publish(&temperature_msg);
}

void mag_heading(){
  headingDegrees = atan2(mpu.getMagY(), mpu.getMagX())/0.0174532925;
  if(headingDegrees<=0)
  {headingDegrees+=360;}
  heading_msg.data=headingDegrees;
  pub_heading.publish(&heading_msg);
}
