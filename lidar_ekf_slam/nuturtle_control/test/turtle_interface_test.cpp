#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"

TEST_CASE("adding duration gives future time", "[ROS Time]") {
  
  ros::NodeHandle nh("~"); // this initializes time
  ros::Time time_now = ros::Time::now();
  ros::Time time_future = time_now + ros::Duration(5.0);
  REQUIRE(time_future > time_now);

}

static double ul = 0.0;
static double ur = 0.0;
static double wl_pos=0.0;
static double wr_pos=0.0;
static double wl_vel=0.0;
static double wr_vel=0.0;
static bool if_sub= false;

void wheel_cmd_callback(const nuturtlebot_msgs::WheelCommands &msg){
    ul = msg.left_velocity;
    ur = msg.right_velocity;
    if_sub=true;
}

TEST_CASE("turtle_interface_test: TEST1 ", "[TEST1: cmd_vel -> wheel_cmd]") {
  
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub= nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber wheel_cmd_sub = nh.subscribe("wheel_cmd", 1000, wheel_cmd_callback);

  geometry_msgs::Twist t;
  t.angular.z=0;
  t.linear.x=0.15;
  while(cmd_vel_pub.getNumSubscribers()==0){
    ros::spinOnce();
  }
  cmd_vel_pub.publish(t);
  while (!if_sub)
    {
        ros::spinOnce();
    }
  if_sub=false;
  Approx target1 = Approx(189.0).epsilon(10e-2);
  Approx target2 = Approx(189.0).epsilon(10e-2);
  REQUIRE(ul==target1);
  REQUIRE(ur==target2);
}

TEST_CASE("turtle_interface_test: TEST2 ", "[TEST2: cmd_vel -> wheel_cmd]") {
  
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub= nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber wheel_cmd_sub = nh.subscribe("wheel_cmd", 1000, wheel_cmd_callback);

  geometry_msgs::Twist t;
  t.angular.z=0.2;
  t.linear.x=0.0;
  while(cmd_vel_pub.getNumSubscribers()==0){
    ros::spinOnce();
  }
  cmd_vel_pub.publish(t);
  while (!if_sub)
    {
        ros::spinOnce();
    }
  if_sub=false;
  Approx target1 = Approx(-20.20).epsilon(10e-2);
  Approx target2 = Approx(20.20).epsilon(10e-2);
  REQUIRE(ul==target1);
  REQUIRE(ur==target2);
}

void sensor_data_callback(const sensor_msgs::JointState& msg){
    wl_pos = msg.position.at(0);
    wr_pos = msg.position.at(1);
    wl_vel=msg.velocity.at(0);
    wr_vel=msg.velocity.at(1);
    if_sub=true;
}

TEST_CASE("turtle_interface_test: TEST3 ", "[TEST3: sensor_data -> joint_state]") {
  
  ros::NodeHandle nh;
  ros::Publisher sensor_data_pub= nh.advertise<nuturtlebot_msgs::SensorData>("sensor_data", 10);
  ros::Subscriber joint_state_sub = nh.subscribe("joint_states", 1000, sensor_data_callback);

  nuturtlebot_msgs::SensorData msg;
  msg.left_encoder=500;
  msg.right_encoder=800;
  
  while(sensor_data_pub.getNumSubscribers()==0){
    ros::spinOnce();
  }
  sensor_data_pub.publish(msg);
  while (!if_sub)
    {
        ros::spinOnce();
    }
  if_sub=false;
  Approx target1 = Approx(0.767).epsilon(10e-2);
  Approx target2 = Approx(1.227).epsilon(10e-2);
  REQUIRE(wl_pos==target1);
  REQUIRE(wr_pos==target2);

}

TEST_CASE("turtle_interface_test: TEST4 ", "[TEST4: sensor_data -> joint_state]") {
  
  ros::NodeHandle nh;
  ros::Publisher sensor_data_pub= nh.advertise<nuturtlebot_msgs::SensorData>("sensor_data", 10);
  ros::Subscriber joint_state_sub = nh.subscribe("joint_states", 1000, sensor_data_callback);

  nuturtlebot_msgs::SensorData msg;
  msg.left_encoder=500;
  msg.right_encoder=500;
  
  while(sensor_data_pub.getNumSubscribers()==0){
    ros::spinOnce();
  }
  sensor_data_pub.publish(msg);
  while (!if_sub)
    {
        ros::spinOnce();
    }
  if_sub=false;
  Approx target1 = Approx(0.767).epsilon(10e-2);
  Approx target2 = Approx(0.767).epsilon(10e-2);
  REQUIRE(wl_pos==target1);
  REQUIRE(wr_pos==target2);
  turtlelib::DiffDrive diff_drive(turtlelib::Transform2D(turtlelib::Vector2D(0.0,0.0),0.0),0.16,0.033);
  diff_drive.FK_Calculate(wl_pos,wr_pos);
  turtlelib::Transform2D tf = diff_drive.getpose();

  Approx target3 = Approx(0.0253).epsilon(10e-2);
  REQUIRE(tf.translation().x==target3);
  REQUIRE(tf.translation().y==0.0);
  REQUIRE(tf.rotation()==0.0);

}


