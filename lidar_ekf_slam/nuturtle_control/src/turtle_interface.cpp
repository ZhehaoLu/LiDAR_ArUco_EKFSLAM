
/// \file
/// \brief turtle interface node - which implements control of the turtlebot via
/// geometry_msgs/Twist message
///
/// \author Jonas Lu (Zhehao Lu)
/// \date March 16 2022
/// \copyright All rights are reserved by Center For Robotics And Biosystems
/// (CRB) of Northwestern University
///
///
/// PUBLISHES:
///     joint_state (sensor_msgs/JointState):  provide the angle (in radians)
///     and velocity (in rad/sec) wheel_cmd (nuturtlebot/WheelCommands): make
///     the turtlebot follow the specified twist
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist): subscribe to cmd_vel commands
///     sensor_data (nuturtlebot/SensorData): subscribe to encoder data

#include "geometry_msgs/Twist.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "sensor_msgs/JointState.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <ros/ros.h>
#include <string>

static constexpr double PI = 3.14159265358979323846;
std::string left_wheel_js_name = "";
std::string right_wheel_js_name = "";

class Interface {
private:
  ros::Subscriber sensor_data_sub;
  ros::Subscriber cmd_vel_sub;
  ros::Publisher joint_state_pub;
  ros::Publisher wheel_cmd_pub;

  turtlelib::DiffDrive diff_drive;

  double m_track_width;
  double m_motor_cmd_max;
  double m_encoder_ticks_to_rad;
  double m_motor_cmd_to_radsec;
  double m_wheel_radius;

  double m_wheel_linear_maxvel;
  double m_wheel_angular_maxvel;
  double m_motor_rotate_maxvel;

  /// \brief Param initialization
  void param_initialization(ros::NodeHandle &nh);

  /// \brief Param transmission test
  void param_test();

public:
  /// \brief constructor - define private members
  Interface(ros::NodeHandle &nh);

  /// \brief transform sensor data (encoder readings in the range of [0,4096])
  /// into joint states
  void sensor_data_callback(const nuturtlebot_msgs::SensorData &msg);

  /// \brief transform twist into wheelcommands ([-256,256] that are
  /// proportional to wheel velocity)
  void cmd_vel_callback(const geometry_msgs::Twist &msg);
};

Interface::Interface(ros::NodeHandle &nh) {
  param_initialization(nh);
  // param_test();
  diff_drive = turtlelib::DiffDrive(
      turtlelib::Transform2D(turtlelib::Vector2D(0.0, 0.0), 0.0), m_track_width,
      m_wheel_radius);
  // ROS_INFO("%f,%f:", m_track_width,m_wheel_radius);
  sensor_data_sub = nh.subscribe("/sensor_data", 1000,
                                 &Interface::sensor_data_callback, this);
  cmd_vel_sub =
      nh.subscribe("cmd_vel", 1000, &Interface::cmd_vel_callback, this);
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  wheel_cmd_pub =
      nh.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd", 10);
};

void Interface::param_initialization(ros::NodeHandle &nh) {
  nh.getParam("/track_width", m_track_width);
  nh.getParam("/motor_cmd_max", m_motor_cmd_max);
  nh.getParam("/encoder_ticks_to_rad", m_encoder_ticks_to_rad);
  nh.getParam("/motor_cmd_to_radsec", m_motor_cmd_to_radsec);
  nh.getParam("/wheel_radius", m_wheel_radius);
  nh.getParam("/wheel_left", left_wheel_js_name);
  nh.getParam("/wheel_right", right_wheel_js_name);

  m_motor_rotate_maxvel = m_motor_cmd_max * m_motor_cmd_to_radsec;
  m_wheel_linear_maxvel =
      m_motor_cmd_max * m_motor_cmd_to_radsec * m_wheel_radius;
  m_wheel_angular_maxvel = 2 * m_wheel_linear_maxvel / m_track_width;
}
void Interface::param_test() {

  ROS_INFO("%f", m_track_width);
  ROS_INFO("%f", m_motor_cmd_max);
  ROS_INFO("%f", m_encoder_ticks_to_rad);
  ROS_INFO("%f", m_motor_cmd_to_radsec);
  ROS_INFO("%f", m_wheel_radius);
  ROS_INFO("%f", m_wheel_linear_maxvel);
  ROS_INFO("%f", m_wheel_angular_maxvel);
  ROS_INFO("%f", m_motor_rotate_maxvel);
}
void Interface::sensor_data_callback(const nuturtlebot_msgs::SensorData &msg) {

  double left_wheel_angle, right_wheel_angle;
  left_wheel_angle = 2 * PI * (msg.left_encoder) / m_encoder_ticks_to_rad;
  right_wheel_angle = 2 * PI * (msg.right_encoder) / m_encoder_ticks_to_rad;
  turtlelib::normalize_angle(left_wheel_angle);
  turtlelib::normalize_angle(right_wheel_angle);

  turtlelib::WheelVel vel_calculate =
      diff_drive.FK_Calculate(left_wheel_angle, right_wheel_angle);

  sensor_msgs::JointState js;
  js.header.stamp = ros::Time::now();
  js.header.frame_id = "odom";

  js.name.push_back(left_wheel_js_name);
  js.position.push_back(left_wheel_angle);
  js.velocity.push_back(vel_calculate.m_ul);

  js.name.push_back(right_wheel_js_name);
  js.position.push_back(right_wheel_angle);
  js.velocity.push_back(vel_calculate.m_ur);

  joint_state_pub.publish(js);
  return;
}

void Interface::cmd_vel_callback(const geometry_msgs::Twist &msg) {

  double linear_vel = msg.linear.x;
  double angular_vel = msg.angular.z;

  // ROS_INFO("Calculated twist command: [%f,%f,0]:", angular_vel, linear_vel);

  turtlelib::Twist2D t_cmd(angular_vel, linear_vel, 0);
  // ROS_INFO("Calculated twist command: [%f,%f,0]:", angular_vel, linear_vel);

  turtlelib::WheelVel v_cmd = diff_drive.IK_Calculate(t_cmd);
  // ROS_INFO("left: %f, right: %f", v_cmd.m_ul,v_cmd.m_ur);

  v_cmd.m_ul = v_cmd.m_ul / m_motor_rotate_maxvel * m_motor_cmd_max;
  v_cmd.m_ur = v_cmd.m_ur / m_motor_rotate_maxvel * m_motor_cmd_max;

  // ROS_INFO("left: %f, right: %f", v_cmd.m_ul,v_cmd.m_ur);
  nuturtlebot_msgs::WheelCommands wheel_cmd;
  wheel_cmd.left_velocity = (int)(v_cmd.m_ul);
  wheel_cmd.right_velocity = (int)(v_cmd.m_ur);

  wheel_cmd_pub.publish(wheel_cmd);
  return;
}

int main(int argc, char **argv) {
  ROS_INFO("turtle_interface node in the loop.");
  ros::init(argc, argv, "turtle_interface");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
  Interface turtle_interface = Interface(nh);
  ros::spin();
  ROS_INFO("turtle_interface is closed.");
  return 0;
}