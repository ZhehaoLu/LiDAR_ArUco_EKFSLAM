#include <aruco_ekf_slam/aruco_ekf_slam.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
using namespace std;

int main(int argc, char **argv) {

  ros::init(argc, argv, "reslam_odom");
  ArUcoEKFSLAM slam;
  ros::spin();
  return 0;
}
