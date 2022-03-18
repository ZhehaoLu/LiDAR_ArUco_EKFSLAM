#ifndef ARUCO_EKF_SLAM_H
#define ARUCO_EKF_SLAM_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/aruco.hpp>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
using namespace Eigen;
using namespace cv;
using namespace std;

class Observation
{
public:
    Observation(){};
    Observation(const int &aruco_id, const double &r, const double &phi) : aruco_id_(aruco_id), r_(r), phi_(phi){};

    int aruco_id_;
    double r_;
    double phi_;

}; // an obervation contains: aruco_id, r, phi.

class ArUcoEKFSLAM
{
private:
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node;

    ros::Publisher g_landmark_pub;
    ros::Publisher g_robot_pose_pub;
    image_transport::Publisher g_img_pub;

    ros::Subscriber encoder_sub;
    ros::Subscriber image_sub;

    string image_topic, encoder_topic;

    // whether the SLAM system is initiated
    bool is_init_;

    // the system parameters which is read from default.yaml
    cv::Mat K_, dist_;      
    double kl_, kr_, b_;    
    Eigen::Matrix4d T_r_c_; 
    double k_;             
    double k_r_;            
    double k_phi_;          

    int n_markers_;
    int marker_size_;
    double marker_length_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat marker_img_;

    // last encoder values (left & right)
    double last_enl_, last_enr_;

    // the mean and covariance of extended state 
    Eigen::MatrixXd mu_;            
    Eigen::MatrixXd sigma_;     
    std::vector<int> aruco_ids_; 

    void GetTopicName();

    void GetParameter();

    void addEncoder(const double &enl, const double &enr);

    void addImage(const Mat &img);

    int getObservations(const cv::Mat &img, std::vector<Observation> &obs);

    void normAngle(double &angle)
    {
        const static double PI = 3.1415926;
        static double Two_PI = 2.0 * PI;
        if (angle >= PI)
            angle -= Two_PI;
        if (angle < -PI)
            angle += Two_PI;
    }

    bool checkLandmark(int aruco_id, int &landmark_idx)
    {
        for (size_t i = 0; i < aruco_ids_.size(); i++)
        {
            if (aruco_id == aruco_ids_.at(i))
            {
                landmark_idx = i;
                return true;
            }
        }
        return false;
    }

    visualization_msgs::MarkerArray toRosMarker(double scale);

    geometry_msgs::PoseWithCovarianceStamped toRosPose()
    {
        geometry_msgs::PoseWithCovarianceStamped rpose;
        rpose.header.frame_id = "world";
        rpose.pose.pose.position.x = mu_(0, 0);
        rpose.pose.pose.position.y = mu_(1, 0);
        rpose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(mu_(2, 0));

        rpose.pose.covariance.at(0) = sigma_(0, 0);
        rpose.pose.covariance.at(1) = sigma_(0, 1);
        rpose.pose.covariance.at(6) = sigma_(1, 0);
        rpose.pose.covariance.at(7) = sigma_(1, 1);
        rpose.pose.covariance.at(5) = sigma_(0, 2);
        rpose.pose.covariance.at(30) = sigma_(2, 0);
        rpose.pose.covariance.at(35) = sigma_(2, 2);

        return rpose;
    }

    Eigen::MatrixXd &mu() { return mu_; }
    Eigen::MatrixXd &sigma() { return sigma_; }
    cv::Mat markedImg() { return marker_img_; }

public:
    ArUcoEKFSLAM();

    ~ArUcoEKFSLAM(){};

    void ImageCallback(const sensor_msgs::ImageConstPtr &img_ptr);

    void EncoderCallback(const geometry_msgs::QuaternionStamped::ConstPtr &encoder_ptr);
};

#endif