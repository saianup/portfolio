#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_utilities/string_utilities.h>

// IMU Setup
Adafruit_MPU6050 mpu;
#define IMU_HZ 100
unsigned long last_imu_time = 0;

// Encoder Pins
#define LM_ENC_A 18
#define LM_ENC_B 23
#define RM_ENC_A 19
#define RM_ENC_B 32

// Motor Pins
#define LMF 27
#define LMB 26
#define RMF 25
#define RMB 33

// Encoder Values
volatile long left_encoder_value = 0;
volatile long right_encoder_value = 0;

// Robot Parameters
#define TICKS_PER_REV 695.0
#define WHEEL_RADIUS 0.045   // 4.5 cm
#define WHEEL_BASE   0.21    // 21 cm

// ROS 2 Setup
rcl_publisher_t imu_pub;
rcl_publisher_t odom_pub;
rcl_subscription_t cmd_vel_sub;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// Encoder ISRs
void IRAM_ATTR leftUpdateEncoder() {
  bool A = digitalRead(LM_ENC_A);
  bool B = digitalRead(LM_ENC_B);
  left_encoder_value += (A == B) ? 1 : -1;
}

void IRAM_ATTR rightUpdateEncoder() {
  bool A = digitalRead(RM_ENC_A);
  bool B = digitalRead(RM_ENC_B);
  right_encoder_value += (A == B) ? 1 : -1;
}

// Pose Estimate
float x = 0.0;
float y = 0.0;
float th = 0.0;

long prev_left = 0;
long prev_right = 0;

// Motor control
void setMotorSpeed(float left_speed, float right_speed) {
  digitalWrite(LMF, left_speed > 0.1);
  digitalWrite(LMB, left_speed < -0.1);
  digitalWrite(RMF, right_speed > 0.1);
  digitalWrite(RMB, right_speed < -0.1);
}

// Callback for /cmd_vel
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear = msg->linear.x;
  float angular = msg->angular.z;
  float left_speed = linear + angular;
  float right_speed = linear - angular;
  setMotorSpeed(left_speed, right_speed);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  set_microros_transports();
  
  if (!mpu.begin()) {
    while (1) {
      Serial.println("MPU6050 not found");
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  pinMode(LM_ENC_A, INPUT);
  pinMode(LM_ENC_B, INPUT);
  pinMode(RM_ENC_A, INPUT);
  pinMode(RM_ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LM_ENC_A), leftUpdateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RM_ENC_A), rightUpdateEncoder, CHANGE);

  pinMode(LMF, OUTPUT);
  pinMode(LMB, OUTPUT);
  pinMode(RMF, OUTPUT);
  pinMode(RMB, OUTPUT);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "robot_sensors_node", "", &support);

  rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data"
  );

  rclc_publisher_init_default(
    &odom_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom"
  );

  rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  );

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);

  imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");
  odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
  odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_link");
}

void loop() {
  unsigned long now = millis();

  if (now - last_imu_time >= (1000 / IMU_HZ)) {
    last_imu_time = now;

    sensors_event_t acc, gyro, temp;
    mpu.getEvent(&acc, &gyro, &temp);

    unsigned long usec = micros();
    imu_msg.header.stamp.sec = usec / 1000000;
    imu_msg.header.stamp.nanosec = (usec % 1000000) * 1000;

    imu_msg.linear_acceleration.x = acc.acceleration.x;
    imu_msg.linear_acceleration.y = acc.acceleration.y;
    imu_msg.linear_acceleration.z = acc.acceleration.z;

    imu_msg.angular_velocity.x = gyro.gyro.x;
    imu_msg.angular_velocity.y = gyro.gyro.y;
    imu_msg.angular_velocity.z = gyro.gyro.z;

    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;

    rcl_publish(&imu_pub, &imu_msg, NULL);
  }

  static unsigned long last_odom_time = 0;
  if (now - last_odom_time >= 50) {
    last_odom_time = now;

    long left = left_encoder_value;
    long right = right_encoder_value;

    long delta_left = left - prev_left;
    long delta_right = right - prev_right;

    prev_left = left;
    prev_right = right;

    float dist_per_tick = 2 * PI * WHEEL_RADIUS / TICKS_PER_REV;
    float d_left = delta_left * dist_per_tick;
    float d_right = delta_right * dist_per_tick;
    float d_center = (d_left + d_right) / 2.0;
    float dth = (d_right - d_left) / WHEEL_BASE;

    x += d_center * cos(th + dth / 2.0);
    y += d_center * sin(th + dth / 2.0);
    th += dth;

    unsigned long usec = micros();
    odom_msg.header.stamp.sec = usec / 1000000;
    odom_msg.header.stamp.nanosec = (usec % 1000000) * 1000;

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(th / 2.0);
    odom_msg.pose.pose.orientation.w = cos(th / 2.0);

    odom_msg.twist.twist.linear.x = d_center / 0.05;
    odom_msg.twist.twist.angular.z = dth / 0.05;

    rcl_publish(&odom_pub, &odom_msg, NULL);
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
