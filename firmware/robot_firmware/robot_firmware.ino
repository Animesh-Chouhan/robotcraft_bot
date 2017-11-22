/* ROS Libs */
#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

/* Motor Driver Serial Bridge lib */
#include "robot_serial_bridge.h"

/* General use lib */
#include <math.h>

/* I2C Sonars*/
#include <Wire.h>

/* Smart LED lib */
#include "Adafruit_NeoPixel.h"
#ifdef __AVR__
#include <avr/power.h>
#endif

/* Auxiliary function to normalize angle to the -pi, pi domain */
#define NORMALIZE(z) atan2(sin(z), cos(z))

/* General defines */
#define SMARTLED_PIN 7                        // SmartLED Arduino PIN
#define SMARTLEDS_NUMBER 4                    // Total of LEDs in the array
#define IR_SENSOR_PIN A0                      // Infrared proximity sensor PIN
#define SONAR_SENSOR_LEFT_ADD  113            // I2C address of left sonar
#define SONAR_SENSOR_RIGHT_ADD  112            // I2C address of right sonar

#define WHEEL_RADIUS 0.040                    // in meters
#define AXIS_LENGTH 0.170                     // in meters
#define PULSES_REV 34.0
#define NORMALIZE(z) atan2(sin(z), cos(z))    // auxiliary function to normalize angle to the -pi, pi domain 

// Init SmartLed Class
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(SMARTLEDS_NUMBER, SMARTLED_PIN, NEO_GRB + NEO_KHZ800);

// Init Serial communication class
// Communication between motor driver and Arduino Mega use Serial1 TX1 e RX1 (Pins 18 and 19)
robot_serial_bridge robot = robot_serial_bridge(&Serial1);

// Global struct with robot data
struct {
  float robot_pose[3] = {0.0};                    // x, y, theta
  float robot_vel[4] = {0.0};                     // linearVel, angularVel, leftAngularWheelVel, rightAngularWheelVel
  float Vx_desired_ = 0.0;
  float Vw_desired_ = 0.0;
} robot_data;

// PID declarations
const float Kp = 0.7, Kd = 0.01, Ki = 0.4;                             // PID gains
float Kp_errorR = 0, Kp_errorR_ = 0, Kd_errorR = 0, Ki_errorR = 0;
float Kp_errorL = 0, Kp_errorL_ = 0, Kd_errorL = 0, Ki_errorL = 0;

// Flags and timers
bool ONCE = true;
unsigned long ros_pub_elapsed_time = 0;
unsigned long pid_elapsed_time = 0;
unsigned long cmd_vel_timeout = 0;

// ROS declarations
ros::NodeHandle nh;
geometry_msgs::Pose2D odometry;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
std_msgs::Float32MultiArray velocities;
std_msgs::Float32MultiArray distance_sensors;

ros::Publisher odometryPublisher("odom", &odometry);
ros::Publisher velocitesPublisher("velocities", &velocities);
ros::Publisher sensorsPublisher("distance_sensors", &distance_sensors);



void cmd_velCallback(const geometry_msgs::Twist &msg) {
  robot_data.Vx_desired_ = msg.linear.x;
  robot_data.Vw_desired_ = msg.angular.z;

  cmd_vel_timeout = millis();
}

void initialPoseCallback(const geometry_msgs::Pose2D &msg) {
  robot_data.robot_pose[0] = msg.x;
  robot_data.robot_pose[1] = msg.y;
  robot_data.robot_pose[2] = msg.theta;
}

void ledsCallback(const std_msgs::UInt8MultiArray &msg) {
  pixels.setPixelColor(0, pixels.Color(msg.data[0], msg.data[1], msg.data[2]));
  pixels.setPixelColor(1, pixels.Color(msg.data[3], msg.data[4], msg.data[5]));
  pixels.setPixelColor(2, pixels.Color(msg.data[6], msg.data[7], msg.data[8]));
  pixels.setPixelColor(3, pixels.Color(msg.data[9], msg.data[10], msg.data[11]));

  pixels.show();
}

ros::Subscriber<geometry_msgs::Twist> cmd_velSubscriber("cmd_vel", &cmd_velCallback);
ros::Subscriber<geometry_msgs::Pose2D> initialPoseSubscriber("initial_pose", &initialPoseCallback);
ros::Subscriber<std_msgs::UInt8MultiArray> ledsSubscriber("RGB_leds", &ledsCallback);



/************************************************************************

   Function:  leds_off()
   Objective:
   Issues:

 *************************************************************************/
void leds_off() {

  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  pixels.setPixelColor(2, pixels.Color(0, 0, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 0));

  pixels.show();
}

/************************************************************************

   Function:  leds_test()
   Objective:
   Issues:

 *************************************************************************/
void leds_test() {

  for (int i = 0; i < 2; i++) {

    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 255, 0));
    pixels.setPixelColor(2, pixels.Color(0, 0, 255));
    pixels.setPixelColor(3, pixels.Color(0, 255, 0));
    pixels.show();
    delay(500);
    leds_off();
    delay(500);
  }
}

/************************************************************************

   Function: requestUltrasonics()
   Objective: Request distance values single I2C sensors
   Inputs parameters: I2C sensor address
   Outputs parameters:
   Return:
   Notes:

 *************************************************************************/
void requestUltrasonic(int address) {

  Wire.beginTransmission(address);
  Wire.write(byte(0x00));
  Wire.write(byte(0x51));
  Wire.endTransmission();
}

/************************************************************************

   Function: readUltrasonic()
   Objective: Read ultrasonic I2C sensor
   Inputs parameters: I2C sensor address
   Outputs parameters:
   Return:
   Notes:

 *************************************************************************/
int readUltrasonic(int address) {
  int reading = 0;

  Wire.beginTransmission(address);
  Wire.write(byte(0x02));
  Wire.endTransmission();

  Wire.requestFrom(address, 2);

  if (2 <= Wire.available()) {
    reading = Wire.read();
    reading = reading << 8;
    reading |= Wire.read();
  }

  return reading;
}

/************************************************************************

   Function: requestUltrasonics()
   Objective: Request distance values from both I2C sensors
   Inputs parameters:
   Outputs parameters:
   Return:
   Notes:

 *************************************************************************/
void requestUltrasonics() {
  requestUltrasonic(SONAR_SENSOR_RIGHT_ADD);
  requestUltrasonic(SONAR_SENSOR_LEFT_ADD);
}

/************************************************************************

   Function: readLeft()
   Objective: Read left I2C distance sensor
   Inputs parameters:
   Outputs parameters:
   Return:
   Notes:

 *************************************************************************/
float s_value_L = 0.0;
float readLeft_sonar() {

  float s_value = readUltrasonic(SONAR_SENSOR_LEFT_ADD) / 100.0;

  if (s_value == 0.0 || s_value > 2.50) {
    s_value = s_value_L;
  }
  s_value_L = s_value;

  return s_value;
}

/************************************************************************

   Function: readRight()
   Objective: Read right I2C distance sensor
   Inputs parameters:
   Outputs parameters:
   Return:
   Notes:

 *************************************************************************/
float s_value_R = 0.0;
float readRight_sonar() {

  float s_value = readUltrasonic(SONAR_SENSOR_RIGHT_ADD) / 100.0;

  if (s_value == 0.0 || s_value > 2.50) {
    s_value = s_value_R;
  }
  s_value_R = s_value;

  return s_value;
}

/************************************************************************

   Function: readFront()
   Objective: Read IR distance sensor
   Inputs parameters:
   Outputs parameters:
   Return:
   Notes:

 *************************************************************************/
float readFront() {

  float reading = (float)(29.988 * pow((analogRead(IR_SENSOR_PIN) * 0.0048828125), -1.173)) / 100.0;

  if (reading > 0.8)
    reading = 0.8;

  return reading;
}



/************************************************************************

   Function:  stop_motors_pid()
   Objective:
   Issues:

 *************************************************************************/
void stop_motors_reset_pid() {

      robot_data.Vx_desired_ = 0.0;
      robot_data.Vw_desired_ = 0.0;
      Kp_errorR = 0.0;
      Kp_errorR_ = 0.0;
      Kd_errorR = 0.0;
      Ki_errorR = 0.0;
      Kp_errorL = 0.0;
      Kp_errorL_ = 0,0;
      Kd_errorL = 0.0;
      Ki_errorL = 0.0;
      robot.stop_motors();

}

/************************************************************************

   Function:  cmd_vel2wheels()
   Objective:  This function convert cmd_vel (Vx linear velocity and Vw angular velicity)from ROS side to wheels velocity (rad/s)
   Issues:

 *************************************************************************/
void cmd_vel2wheels(float Vx_desired, float Vw_desired, float &vel_wheel1_desired, float &vel_wheel2_desired, int deltaT) {

  vel_wheel1_desired = (Vx_desired + (AXIS_LENGTH / 2.0) * Vw_desired) / WHEEL_RADIUS;
  vel_wheel2_desired = (Vx_desired - (AXIS_LENGTH / 2.0) * Vw_desired) / WHEEL_RADIUS;

}

/************************************************************************

   Function:  pose_update()
   Objective: Calculate the odometry
   Issues:

 *************************************************************************/
void pose_update(float pose[], float &vel_wheel1_real, float &vel_wheel2_real, int deltaT) {

  // motor 1 -> right
  // motor 2 -> left

  float pose_old[3];
  pose_old[0] = pose[0];
  pose_old[1] = pose[1];
  pose_old[2] = pose[2];

  long int enc1_diff;
  long int enc2_diff;

  robot.get_encoders_diff(enc1_diff, enc2_diff);

  //calulate Odometry
  float Vx_real = (((float)(enc1_diff + enc2_diff) / 2.0) * (2.0 * PI * WHEEL_RADIUS)) / PULSES_REV;
  float Vw_real = (((float)(enc1_diff - enc2_diff) / AXIS_LENGTH) * (2.0 * PI * WHEEL_RADIUS)) / PULSES_REV;

  // Update odometry
  pose[2] = NORMALIZE(pose[2] + Vw_real);          // radians
  pose[0] = pose[0] + Vx_real * cos(pose[2]);      // meters
  pose[1] = pose[1] + Vx_real * sin(pose[2]);      // meters


  vel_wheel1_real = (Vx_real + (AXIS_LENGTH / 2.0) * Vw_real) / WHEEL_RADIUS / (deltaT / 1000.0);
  vel_wheel2_real = (Vx_real - (AXIS_LENGTH / 2.0) * Vw_real) / WHEEL_RADIUS / (deltaT / 1000.0);


  // Calculate robot speed
  robot_data.robot_vel[0] = (Vx_real) / (deltaT / 1000.0);
  robot_data.robot_vel[1] = (Vw_real) / (deltaT / 1000.0);
  robot_data.robot_vel[2] = vel_wheel1_real;
  robot_data.robot_vel[3] = vel_wheel2_real;

}


/************************************************************************

   Function:  pid_controller()
   Objective:
   Issues:

 *************************************************************************/
void pid_controller(float vel_wheel1_desired, float vel_wheel2_desired, float vel_wheel1_real, float vel_wheel2_real, int deltaT) {

  Kp_errorR = (vel_wheel1_desired - vel_wheel1_real);
  Kd_errorR = Kp_errorR - Kp_errorR_;
  Ki_errorR = Ki_errorR + Kp_errorR;
  Kp_errorR_ = Kp_errorR;
  float V_errorR =  Kp_errorR * Kp + (Kd_errorR / (deltaT / 1000.0)) * Kd + (Ki_errorR * (deltaT / 1000.0)) * Ki;

  Kp_errorL = (vel_wheel2_desired - vel_wheel2_real);
  Kd_errorL = Kp_errorL - Kp_errorL_;
  Ki_errorL = Ki_errorL + Kp_errorL;
  Kp_errorL_ = Kp_errorL;
  float V_errorL =  Kp_errorL * Kp + (Kd_errorL / (deltaT / 1000.0)) * Kd + (Ki_errorL * (deltaT / 1000.0)) * Ki;

  if (V_errorR > 30)
    V_errorR = 30;

  if (V_errorR < -30)
    V_errorR = -30;

  if (V_errorL > 30)
    V_errorL = 30;

  if (V_errorL < -30)
    V_errorL = -30;

  robot.move_motors_pwm((int)V_errorR, (int)V_errorL);
}



/************************************************************************

   Function:  loop()
   Objective:
   Issues:

 *************************************************************************/
void setup() {

  /* ROS init */
  nh.getHardware()->setBaud(115200);
  nh.initNode();


  // ROS Publishers
  nh.advertise(odometryPublisher);
  nh.advertise(velocitesPublisher);
  nh.advertise(sensorsPublisher);

  // ROS Subscribers
  nh.subscribe(cmd_velSubscriber);
  nh.subscribe(initialPoseSubscriber);
  nh.subscribe(ledsSubscriber);

  // ROS tf broadcaster
  broadcaster.init(nh);

  // Init ROS msg array
  velocities.data = (float *)malloc(sizeof(float) * 4);
  velocities.data_length = 4;
  distance_sensors.data = (float *)malloc(sizeof(float) * 3);
  distance_sensors.data_length = 3;

  // Init Serial connection with robot
  robot.begin();

  // Stop motors just in case
  robot.stop_motors();

  // Init I2C comm
  Wire.begin();

  // Init NeoPixel
  pixels.begin();
  leds_test();

}



/************************************************************************

   Function:  loop()
   Objective:
   Issues:

 *************************************************************************/

void loop() {


  if (millis() - pid_elapsed_time > 250 ) {

    float vel_w_d1;
    float vel_w_d2;

    float vel_w_r1;
    float vel_w_r2;

    int deltaT = millis() - pid_elapsed_time;

    pid_elapsed_time = millis();

    // Timeout cmd_vel
    if (millis() - cmd_vel_timeout > 1000) {
      // Reset cmd_vel
      stop_motors_reset_pid();
    }

    cmd_vel2wheels(robot_data.Vx_desired_, robot_data.Vw_desired_, vel_w_d1, vel_w_d2, deltaT);
    pose_update(robot_data.robot_pose, vel_w_r1, vel_w_r2, deltaT);
    pid_controller(vel_w_d1, vel_w_d2, vel_w_r1, vel_w_r2, deltaT);
  }



  if (millis() - ros_pub_elapsed_time > 200 ) {

    ros_pub_elapsed_time = millis();

    // Read distance sensors
    distance_sensors.data[0] = readLeft_sonar();
    distance_sensors.data[1] = readFront();
    distance_sensors.data[2] = readRight_sonar();
    sensorsPublisher.publish(&distance_sensors);

    // Request new values to be ready for next iteration
    requestUltrasonics();

    // Odometry
    odometry.x = robot_data.robot_pose[0];
    odometry.y = robot_data.robot_pose[1];
    odometry.theta = robot_data.robot_pose[2];
    odometryPublisher.publish(&odometry);

    // Velocities
    velocities.data[0] = robot_data.robot_vel[0];
    velocities.data[1] = robot_data.robot_vel[1];
    velocities.data[2] = robot_data.robot_vel[2];
    velocities.data[3] = robot_data.robot_vel[3];
    velocitesPublisher.publish(&velocities);

    // Advertise tf
    t.header.frame_id = "/odom";
    t.child_frame_id = "/base_link";
    t.transform.translation.x = odometry.x;
    t.transform.translation.y = odometry.y;
    t.transform.rotation = tf::createQuaternionFromYaw(odometry.theta);
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);

  }

  nh.spinOnce();

}

