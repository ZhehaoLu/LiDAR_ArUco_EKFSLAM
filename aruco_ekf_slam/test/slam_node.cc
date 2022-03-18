#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <aruco_ekf_slam/aruco_ekf_slam.h>
using namespace std;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "reslam_odom");
    ArUcoEKFSLAM slam;
    ros::spin();
    return 0;
}
