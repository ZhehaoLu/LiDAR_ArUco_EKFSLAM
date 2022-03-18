/// \file
/// \brief circle node - publishes cmd_vel commands to cause the robot to drive in a circle of a specified radius at a specified speed
///
/// \author Jonas Lu (Zhehao Lu)
/// \date March 16 2022
/// \copyright All rights are reserved by Center For Robotics And Biosystems (CRB) of Northwestern University
///
/// 
/// PARAMETERS:
///     angular_vel: angular velocity of the circle
///     arc_radius: radius of the circle
///     stopped: whether it is stopped or not (when stopped - publish zero twist) 
/// PUBLISHES:
///     cmd_vel (geometry_msgs/Twist): publish cmd_vel commands
/// SERVICES:
///     control_service (nuturtle_control::control): control the angular velocity and radius of the circle
///     reverse_service (std_srvs::Empty): reverse the trajectory of the robot
///     stop_service (std_srvs::Empty): stop the robot

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nuturtle_control/control.h"
#include "std_srvs/Empty.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"

static double angular_vel = 0.0;
static double arc_radius = 0.0;
static bool stopped = false;
static double track_width=0.0;
static int frequency = 100;

/// \brief control the angular velocity and radius of the circle
/// \return true or false
bool control_callback(nuturtle_control::control::Request& req, nuturtle_control::control::Response& res);

/// \brief reverse the trajectory of the robot
/// \return true or false
bool reverse_callback(std_srvs::Empty::Request& , std_srvs::Empty::Response& );

/// \brief stop the robot
/// \return true or false
bool stop_callback(std_srvs::Empty::Request& , std_srvs::Empty::Response& );

int main(int argc, char** argv){

    ROS_INFO("circle node in the loop.");
    ros::init(argc,argv,"circle");
    ros::NodeHandle nh;
    
    nh.getParam("track_width",track_width);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::ServiceServer control_service = nh.advertiseService("control", control_callback);
    ros::ServiceServer reverse_service = nh.advertiseService("reverse", reverse_callback);
    ros::ServiceServer stop_service = nh.advertiseService("stop", stop_callback);

    ros::Rate loop_rate(frequency);

    while(ros::ok()){
        if(stopped==true){
            geometry_msgs::Twist t;
            t.linear.x=0.0;
            t.angular.z=0.0;
            cmd_vel_pub.publish(t);
        }
        else{
            geometry_msgs::Twist t;
            t.linear.x = - arc_radius * angular_vel;
            t.angular.z = angular_vel;
            cmd_vel_pub.publish(t);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("circle node is closed.");
    return 0;

}

bool control_callback(nuturtle_control::control::Request& req, nuturtle_control::control::Response& res){
    angular_vel = req.velocity;
    arc_radius = req.radius;
    res.success = true;
    ROS_INFO("angular_vel: %f, arc_radius: %f", angular_vel, arc_radius);
    return true;
}

bool reverse_callback(std_srvs::Empty::Request& , std_srvs::Empty::Response& ){
    angular_vel = -angular_vel;
    ROS_INFO("direction reversed");
    return true;
}

bool stop_callback(std_srvs::Empty::Request& , std_srvs::Empty::Response& ){

    stopped = true;
    ROS_INFO("robot stopped.");
    return true;
}