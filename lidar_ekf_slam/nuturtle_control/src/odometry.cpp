/// \file
/// \brief odometry node - which publishes odometry messages and the odometry transform
///
/// \author Jonas Lu (Zhehao Lu)
/// \date March 16 2022
/// \copyright All rights are reserved by Center For Robotics And Biosystems (CRB) of Northwestern University
///
/// 
/// PARAMETERS:
///     body_id: The name of the body frame of the robot
///     odom_id: The name of the odometry frame
///     track_width: The track width of the robot
///     wheel_radius: The wheel radius of the robot
///     x_0,y_0,theta_0 (double): Initial (x,y,theta) position of the robot 
/// PUBLISHES:
///     odom (sensor_msgs/JointState):  publishes odometry messages
///     pose (turtlesim/Pose): publish the actual pose
/// SUBSCRIBES:
///     joint_states (sensor_msgs::JointState): subscribe to joint_states (to publish odometry)

#include <ros/ros.h>
#include <string>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf/transform_broadcaster.h"
#include "turtlesim/Pose.h"
#include "nav_msgs/Path.h"
#include "nuturtle_control/set_pose.h"

static std::string body_id = "blue/base_footprint";
static std::string odom_id = "odom";
static double track_width=0.0;
static double wheel_radius=0.0;
static double x_0=0.0, y_0=0.0, theta_0=0.0;
static turtlelib::DiffDrive diff_drive;
ros::Publisher odom_pub;
ros::Publisher pose_pub;
ros::Publisher path_pub;
ros::Subscriber joint_state_sub;
nav_msgs::Path Path;
geometry_msgs::PoseStamped PoseStamped;

/// \brief transform from joint_states (to publish odometry
void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);

/// \brief broadcast transform from "body_id" to "odom_id"
/// \return - true or false
bool broadcast_transform(tf2_ros::TransformBroadcaster& odom_broadcaster);

/// \brief set the pose of the robot from srv
/// \return - true or false
bool set_pose_callback(nuturtle_control::set_pose::Request& req, nuturtle_control::set_pose::Response& res);

/// \brief Param initialization
bool param_initialization(ros::NodeHandle &nh);

int main(int argc, char** argv){

    ROS_INFO("odometry node in the loop.");
    ros::init(argc,argv,"odometry");
    ros::NodeHandle nh;
    param_initialization(nh);

    path_pub = nh.advertise<nav_msgs::Path>("path", 1000);
    Path.header.frame_id = "odom";
    PoseStamped.header.frame_id = "odom";

    diff_drive = turtlelib::DiffDrive(turtlelib::Transform2D(turtlelib::Vector2D(x_0,y_0),theta_0),track_width,wheel_radius);
    
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom",10);
    joint_state_sub = nh.subscribe("joint_states",1000,joint_state_callback);
    pose_pub = nh.advertise<turtlesim::Pose>("nusim/pose_sub",10);
    ros::ServiceServer set_pose_service = nh.advertiseService("set_pose", set_pose_callback);
    tf2_ros::TransformBroadcaster odom_broadcaster;
    ros::Rate loop_rate(100);

    while(ros::ok()){
        broadcast_transform(odom_broadcaster);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("odometry node is closed.");
    return 0;

}

void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg){
    double left_angle = msg->position.at(0);
    double right_angle = msg->position.at(1);

    //ROS_INFO("left_angle,right_angle: %f, %f", left_angle,right_angle);
    diff_drive.FK_Calculate(left_angle,right_angle);
    turtlelib::Transform2D transform = diff_drive.getpose();

    turtlesim::Pose pose;
    pose.x=transform.translation().x;
    pose.y=transform.translation().y;
    pose.theta=transform.rotation();
    pose_pub.publish(pose);
    
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_id;

    odom.pose.pose.position.x = transform.translation().x;
    odom.pose.pose.position.y = transform.translation().y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(transform.rotation());
    
    //ROS_INFO("%f, %f, %f :", transform.translation().x, transform.translation().y, transform.rotation());
    double wl_vel = msg->velocity.at(0);
    double wr_vel = msg->velocity.at(1);

    turtlelib::WheelVel v(wl_vel,wr_vel);
    turtlelib::Twist2D t = diff_drive.wheelvelTotwist(v);
    odom.child_frame_id = body_id;
    odom.twist.twist.linear.x = t.m_translation_vx;
    odom.twist.twist.linear.y = t.m_translation_vy;
    odom.twist.twist.angular.z = t.m_angular_v;
    odom_pub.publish(odom);
}

bool broadcast_transform(tf2_ros::TransformBroadcaster& odom_broadcaster){
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = odom_id;
    tf.child_frame_id = body_id;
    
    turtlelib::Transform2D transform = diff_drive.getpose();
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(transform.rotation());
    tf.transform.translation.x = transform.translation().x;
    tf.transform.translation.y = transform.translation().y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = q;
    //ROS_INFO("[x,y,theta]: %f,%f,%f",transform.translation().x,transform.translation().y,transform.rotation());
    odom_broadcaster.sendTransform(tf);

    PoseStamped.header.stamp = ros::Time::now();
    PoseStamped.pose.position.x = transform.translation().x;
    PoseStamped.pose.position.y = transform.translation().y;
    PoseStamped.pose.position.z = 0.0;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(transform.rotation());

    PoseStamped.pose.orientation.x = goal_quat.x;
    PoseStamped.pose.orientation.y = goal_quat.y;
    PoseStamped.pose.orientation.z = goal_quat.z;
    PoseStamped.pose.orientation.w = goal_quat.w;
    Path.header.stamp = ros::Time::now();
    Path.poses.push_back(PoseStamped);
    path_pub.publish(Path);
    
    return true;
}

bool set_pose_callback(nuturtle_control::set_pose::Request& req, nuturtle_control::set_pose::Response& res){
    double theta = req.theta;
    double x = req.x;
    double y = req.y;
    turtlelib::Transform2D transform = turtlelib::Transform2D(turtlelib::Vector2D(x,y),theta);
    diff_drive.setpose(transform);
    //ROS_INFO("pose reset [x,y,theta]: [%f,%f,%f]",x,y,theta);
    res.success=true;
    return true;
};

bool param_initialization(ros::NodeHandle &nh){
    nh.getParam("/track_width",track_width);
    nh.getParam("/wheel_radius",wheel_radius);
    nh.getParam("/initial_pose/x_0", x_0);
    nh.getParam("/initial_pose/y_0", y_0);
    nh.getParam("/initial_pose/theta_0", theta_0);
    return true;
}