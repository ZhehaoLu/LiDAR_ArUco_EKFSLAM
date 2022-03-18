#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include "nuturtle_control/set_pose.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"

/// \brief - first attempt to see whether it works in ROS
TEST_CASE("adding duration gives future time", "[ROS Time]") {
  
  ros::NodeHandle nh("~");
  ros::Time time_now = ros::Time::now();
  ros::Time time_future = time_now + ros::Duration(5.0);
  REQUIRE(time_future > time_now);
}

TEST_CASE("odometry_test: TEST1 ", "[TEST1: set_pose]"){
    ros::NodeHandle nh;
    ros::ServiceClient set_pose_client = nh.serviceClient<nuturtle_control::set_pose>("/blue/set_pose");
    nuturtle_control::set_pose srv;
    srv.request.theta = 1.0;
    srv.request.x = 1.0;
    srv.request.y = 1.0;
    set_pose_client.call(srv);
    REQUIRE(srv.response.success==true);
} 

TEST_CASE("odometry_test: TEST2 ", "[TEST2: odombroadcaster]"){
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf_listener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    ros::Rate loop_rate(100);
    for(int i=0;ros::ok()&& i<10;i++){
      try{
        transformStamped = tfBuffer.lookupTransform("odom","blue/base_footprint",ros::Time(0));
      }
      catch(tf2::TransformException &ex){
        ros::Duration(1.0).sleep();
        continue;
      }
      REQUIRE(transformStamped.transform.translation.x==1.0);
      REQUIRE(transformStamped.transform.translation.y==1.0);
      REQUIRE(transformStamped.transform.translation.z==0);
      ros::spinOnce();
      loop_rate.sleep();
    }

} 


