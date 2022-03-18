/// \file
/// \brief slam_assoc node - which visualizes the slam robot (both map frame and odom frame) with EKF algorithm (unknown data association)
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
///     epsilon: state vector in EKF algorithm
///     sigma: covariance in EKF algorithm
///     id_to_statevec:  mapping from landmark_id to the index of state vector (epsilon)
/// PUBLISHES:
///     odom (sensor_msgs/JointState):  publishes odometry messages
///     pose (turtlesim/Pose): publish the actual pose
///     slam_marker (visualization_msgs/MarkerArray): publish the marker postion detected by slam robot
/// SUBSCRIBES:
///     joint_states (sensor_msgs/JointState): subscribe to joint_states (to calculate odometry)
///     sensed_landmark (visualization_msgs/MarkerArray): get measurements (from laser scan data)

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
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unordered_map>

static std::string body_id = "green/base_footprint";
static std::string odom_id = "odom";

static double track_width = 0.0;
static double wheel_radius = 0.0;
static double marker_radius = 0.0;
static double x_0 = 0.0, y_0 = 0.0, theta_0 = 0.0;
static double dist_threshold = 2.50;

static turtlelib::DiffDrive diff_drive_slam;
static turtlelib::DiffDrive diff_drive;
static turtlelib::Transform2D transform;

ros::Publisher odom_pub;
ros::Publisher pose_pub;
ros::Publisher path_pub;
ros::Publisher slam_marker_pub;
ros::Subscriber joint_state_sub;
ros::Subscriber sim_lidar_sub;
ros::Subscriber fake_sensor_sub;
ros::Subscriber sense_landmark_sub;
ros::Publisher _marker_pub;
nav_msgs::Path Path;
geometry_msgs::PoseStamped PoseStamped;
class measure;
static std::vector<measure> mea;
static bool if_mea = false;

using std::cout;
using std::endl;

// epsilon and sigma (EKF state)
static Eigen::VectorXd epsilon = Eigen::VectorXd::Zero(3);
static Eigen::MatrixXd sigma = Eigen::MatrixXd::Zero(3, 3);

// noise matrix
Eigen::Matrix<double, 3, 3> Q = (Eigen::Matrix<double, 3, 3>() << 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001).finished();
Eigen::Matrix<double, 2, 2> R = (Eigen::Matrix<double, 2, 2>() << 0.01, 0, 0, 0.01).finished();

// mapping [landmark_id -> vector]
std::unordered_map<int, int> id_to_statevec;

class measure
{
public:
    int m_id;
    double m_range;
    double m_bear;

public:
    measure(int id, double range, double bear) : m_id(id), m_range(range), m_bear(bear){};
    ~measure(){};
};

/// \brief transform from joint_states (to publish odometry
void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);

/// \brief broadcast transform from "body_id" to "odom_id"
/// \return - true or false
bool broadcast_transform(tf2_ros::TransformBroadcaster &odom_broadcaster);

bool broadcast_transform_maptoodom(tf2_ros::TransformBroadcaster &odom_broadcaster);

/// \brief set the pose of the robot from srv
/// \return - true or false
// bool set_pose_callback(nuturtle_control::set_pose::Request &req, nuturtle_control::set_pose::Response &res);

/// \brief Param initialization
bool param_initialization(ros::NodeHandle &nh);

/// \brief transform lidar data from [bear,range] to [x,y] coordinate
/// \return - relative postion (x,y)
turtlelib::Vector2D bear_to_vector(double range, double theta);

/// \brief measurement initialization (for first time observation)
/// \return - global postion (x,y) represented by Eigen
Eigen::Vector2d initialize_measure(const measure &m);

/// \brief calculate and store the global position of measurement from markerarray data
void sense_landmark_callback(const visualization_msgs::MarkerArray &msg);

/// \brief visualize slam markers
void visualize_measurement();

/// \brief update state vector (epsilon) by EKF algorithm (known data association)
void EKF_update(turtlelib::Transform2D &transform, turtlelib::Twist2D &t);

/// \brief pubblish odom and pose
void odom_pose_publish(turtlelib::Transform2D &transform, turtlelib::Twist2D &t);

int main(int argc, char **argv)
{

    ROS_INFO("slam_assoc_turtlebot node in the loop.");
    ros::init(argc, argv, "slam_assoc_turtlebot");
    ros::NodeHandle nh;
    param_initialization(nh);

    path_pub = nh.advertise<nav_msgs::Path>("path", 1000);
    Path.header.frame_id = "map";
    PoseStamped.header.frame_id = "map";

    diff_drive_slam = turtlelib::DiffDrive(turtlelib::Transform2D(turtlelib::Vector2D(x_0, y_0), theta_0), track_width, wheel_radius);
    diff_drive = turtlelib::DiffDrive(turtlelib::Transform2D(turtlelib::Vector2D(x_0, y_0), theta_0), track_width, wheel_radius);

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    joint_state_sub = nh.subscribe("joint_states", 1000, joint_state_callback);
    pose_pub = nh.advertise<turtlesim::Pose>("nusim/pose_sub", 10);

    sense_landmark_sub = nh.subscribe("/sensed_landmark", 1000, sense_landmark_callback);

    // fake_sensor_sub = nh.subscribe("/fake_sensor", 1000, fake_sensor_callback);
    slam_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/slam_marker", 10);

    // sim_lidar_sub = nh.subscribe("/sim_lidar",1000,sim_lidar_callback);
    // slam_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("slam_marker",50);

    // ros::ServiceServer set_pose_service = nh.advertiseService("set_pose", set_pose_callback);
    tf2_ros::TransformBroadcaster odom_broadcaster;
    tf2_ros::TransformBroadcaster odom_broadcaster_maptoodom;
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        broadcast_transform(odom_broadcaster);
        broadcast_transform_maptoodom(odom_broadcaster_maptoodom);
        visualize_measurement();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("slam_assoc_turtlebot node is closed.");
    return 0;
}

turtlelib::Vector2D bear_to_vector(double range, double theta)
{

    double x = range * std::cos(turtlelib::deg2rad(theta));
    double y = range * std::sin(turtlelib::deg2rad(theta));
    return turtlelib::Vector2D(x, y);
}

Eigen::Vector2d initialize_measure(const measure &m)
{
    double x = epsilon(1) + m.m_range * std::cos(m.m_bear + epsilon(0));
    double y = epsilon(2) + m.m_range * std::sin(m.m_bear + epsilon(0));
    Eigen::Vector2d v;
    v(0) = x;
    v(1) = y;
    return v;
}

void sense_landmark_callback(const visualization_msgs::MarkerArray &msg)
{
    // cout << "number:" << msg.markers.size() << endl;
    // for (unsigned int i = 0; i < msg.markers.size(); i++)
    // {
    //     cout << "pos:" << msg.markers[i].pose.position.x << "," << msg.markers[i].pose.position.y << endl;
    // }

    if (!if_mea)
    {
        mea.clear();
        for (unsigned int i = 0; i < msg.markers.size(); i++)
        {
            double x = msg.markers[i].pose.position.x;
            double y = msg.markers[i].pose.position.y;
            double range = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
            double bear = std::atan2(y, x);
            bear = turtlelib::normalize_angle(bear);
            mea.emplace_back(measure(i, range, bear));
        }

        if_mea = true;
    }
}

void visualize_measurement()
{
    int n = (epsilon.size() - 3) / 2;
    // ROS_INFO("n:%i",n);
    if (n == 0)
        return;
    visualization_msgs::MarkerArray markerarray;
    std::vector<visualization_msgs::Marker> slam_markers(n);
    int cnt = 0;
    std::string markerns = "slam_sensor";

    for (auto &marker : slam_markers)
    {
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = markerns + std::to_string(cnt);
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.pose.position.x = epsilon(3 + cnt * 2);
        marker.pose.position.y = epsilon(3 + cnt * 2 + 1);
        // ROS_INFO("pose [x,y]: %d,%f,%f",cnt,marker_x[cnt],marker_y[cnt]);
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker_radius;
        marker.scale.y = marker_radius;
        marker.scale.z = 0.25;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markerarray.markers.emplace_back(marker);
        cnt++;
    }
    slam_marker_pub.publish(markerarray);
}

void EKF_update(turtlelib::Transform2D &transform, turtlelib::Twist2D &t)
{
    Eigen::VectorXd epsilon_next = epsilon;
    epsilon_next(0) = transform.rotation();
    epsilon_next(1) = transform.translation().x;
    epsilon_next(2) = transform.translation().y;

    Eigen::MatrixXd At = Eigen::MatrixXd::Identity(sigma.rows(), sigma.cols());

    if (turtlelib::almost_equal(t.m_angular_v, 0.0))
    {
        At(1, 0) = -t.m_translation_vx * std::sin(epsilon(0));
        At(2, 0) = t.m_translation_vx * std::cos(epsilon(0));
    }
    else
    {
        At(1, 0) = -(t.m_translation_vx / t.m_angular_v) * std::cos(epsilon(0)) +
                   (t.m_translation_vy / t.m_angular_v) * std::cos(epsilon(0) + t.m_angular_v);
        At(2, 0) = -(t.m_translation_vx / t.m_angular_v) * std::sin(epsilon(0)) +
                   (t.m_translation_vy / t.m_angular_v) * std::sin(epsilon(0) + t.m_angular_v);
    }

    Eigen::MatrixXd Q_extended = Eigen::MatrixXd::Zero(sigma.rows(), sigma.cols());
    Q_extended.block(0, 0, 3, 3) = Q;
    sigma = At * sigma * At.transpose() + Q_extended;

    // update epsilon with epsilon_next
    epsilon = epsilon_next;

    // ROS_INFO("epsilon before:%f,%f,%f",epsilon(0),epsilon(1),epsilon(2));

    // correct with measurement data
    if (if_mea)
    {
        // std::cout<<"for each measurement"<<std::endl;
        //  for each measurement i
        for (auto const &m : mea)
        {
            // initialize temporary landmark
            // Eigen::Vector2d z_temp = initialize_measure(m);
            // std::cout<<"z_temp: "<<z_temp<<std::endl;

            // for each already-seen landmark
            int landmark_cnt = (epsilon.size() - 3) / 2;
            // std::cout << "landmark_cnt: "<<landmark_cnt << std::endl;

            double dist_min = dist_threshold;
            int assoc_idx = 0;

            // std::cout<<"for each landmark"<<std::endl;
            for (int i = 0; i < landmark_cnt; i++)
            {

                // 1. compute Hk
                int idx = 3 + 2 * i;

                double range_calculate = std::sqrt(
                    std::pow(epsilon(idx) - epsilon(1), 2) + std::pow(epsilon(idx + 1) - epsilon(2), 2));

                double bear_calculate = turtlelib::normalize_angle(
                    std::atan2(epsilon(idx + 1) - epsilon(2), epsilon(idx) - epsilon(1)) - epsilon(0));

                // ROS_INFO("[range_calculate,bear_calculate]:%f,%f",range_calculate,bear_calculate);

                double delta_x = epsilon(idx) - epsilon(1);
                double delta_y = epsilon(idx + 1) - epsilon(2);

                // ROS_INFO("[dx,dy]: %f,%f",delta_x,delta_y);

                double d = std::pow(delta_x, 2) + std::pow(delta_y, 2);
                double sqrt_d = std::sqrt(d);

                Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, epsilon.size());
                H(0, 0) = 0;
                H(1, 0) = -1;
                H(0, 1) = -delta_x / sqrt_d;
                H(1, 1) = delta_y / d;
                H(0, 2) = -delta_y / sqrt_d;
                H(1, 2) = -delta_x / d;
                H(0, idx) = delta_x / sqrt_d;
                H(0, idx + 1) = delta_y / sqrt_d;
                H(1, idx) = -delta_y / d;
                H(1, idx + 1) = delta_x / d;

                // 2. compute covariance
                Eigen::MatrixXd cov = H * sigma * H.transpose() + R;

                // 3. compute zk
                Eigen::Vector2d zk;
                zk(0) = range_calculate;
                zk(1) = bear_calculate;

                Eigen::Vector2d zi;
                zi(0) = m.m_range;
                zi(1) = m.m_bear;

                // Eigen::Vector2d z_diff = zi - zk;
                Eigen::Vector2d z_diff;
                z_diff(0) = range_calculate - m.m_range;
                z_diff(1) = turtlelib::normalize_angle(bear_calculate - m.m_bear);
                double dist = (z_diff.transpose()) * (cov.inverse()) * z_diff;
                // std::cout<<"["<<" "<<i<<"]: "<<" zi:"<<zi(0)<<" "<<zi(1)<<" zk:"<<zk(0)<<" "<<zk(1)<<std::endl;
                // std::cout<<" z_diff:"<<z_diff(0)<<" "<<z_diff(1)<<" dist:"<<dist<<std::endl;

                // associate with already-seen landmark
                if (dist < dist_min)
                {
                    dist_min = dist;
                    assoc_idx = idx;
                    // std::cout<<"already_seen - assoc_idx:"<<assoc_idx<<std::endl;
                }
            }

            // initialize new landmark (not seen before)
            if (dist_min == dist_threshold)
            {
                // 1. state vector extended
                int idx = epsilon.size();
                epsilon.conservativeResize(idx + 2);
                epsilon(idx) = epsilon(1) + m.m_range * std::cos(m.m_bear + epsilon(0));
                epsilon(idx + 1) = epsilon(2) + m.m_range * std::sin(m.m_bear + epsilon(0));

                // 2. intialize sigma
                sigma.conservativeResize(Eigen::NoChange, idx + 2);
                sigma.block(0, idx, idx, 2) = Eigen::MatrixXd::Zero(idx, 2);
                sigma.conservativeResize(idx + 2, Eigen::NoChange);
                sigma.block(idx, 0, 2, idx + 2) = Eigen::MatrixXd::Zero(2, idx + 2);
                sigma(idx, idx) = 1000;
                sigma(idx + 1, idx + 1) = 1000;

                assoc_idx = idx;
                // std::cout<<"new landmark - assoc_idx:"<<assoc_idx<<std::endl;
            }

            double range_calculate = std::sqrt(
                std::pow(epsilon(assoc_idx) - epsilon(1), 2) + std::pow(epsilon(assoc_idx + 1) - epsilon(2), 2));
            double bear_calculate = turtlelib::normalize_angle(
                std::atan2(epsilon(assoc_idx + 1) - epsilon(2), epsilon(assoc_idx) - epsilon(1)) - epsilon(0));

            // ROS_INFO("[range_calculate,bear_calculate]:%f,%f",range_calculate,bear_calculate);

            double delta_x = epsilon(assoc_idx) - epsilon(1);
            double delta_y = epsilon(assoc_idx + 1) - epsilon(2);

            // ROS_INFO("[dx,dy]: %f,%f",delta_x,delta_y);

            double d = std::pow(delta_x, 2) + std::pow(delta_y, 2);
            double sqrt_d = std::sqrt(d);

            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, epsilon.size());
            H(0, 0) = 0;
            H(1, 0) = -1;
            H(0, 1) = -delta_x / sqrt_d;
            H(1, 1) = delta_y / d;
            H(0, 2) = -delta_y / sqrt_d;
            H(1, 2) = -delta_x / d;
            H(0, assoc_idx) = delta_x / sqrt_d;
            H(0, assoc_idx + 1) = delta_y / sqrt_d;
            H(1, assoc_idx) = -delta_y / d;
            H(1, assoc_idx + 1) = delta_x / d;

            Eigen::Vector2d mea_diff;
            double bear_diff = turtlelib::normalize_angle(m.m_bear - bear_calculate);
            double range_diff = m.m_range - range_calculate;
            mea_diff << range_diff, bear_diff;

            // ROS_INFO("mea_diff: [%f,%f]", mea_diff(0),mea_diff(1));

            Eigen::MatrixXd K = sigma * H.transpose() * ((H * sigma * H.transpose() + R).inverse());

            // ROS_INFO("[be]: %f,%f,%f",epsilon(1),epsilon(2),epsilon(0));

            epsilon = epsilon + K * mea_diff;

            // ROS_INFO("[af]: %f,%f,%f",epsilon(1),epsilon(2),epsilon(0));

            sigma = (Eigen::MatrixXd::Identity(epsilon.size(), epsilon.size()) - K * H) * sigma;
        }
        if_mea = false;
    }
}

void odom_pose_publish(turtlelib::Transform2D &transform, turtlelib::Twist2D &t)
{
    turtlesim::Pose pose;
    pose.x = transform.translation().x;
    pose.y = transform.translation().y;
    pose.theta = transform.rotation();
    pose_pub.publish(pose);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_id;

    odom.pose.pose.position.x = transform.translation().x;
    odom.pose.pose.position.y = transform.translation().y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(transform.rotation());

    odom.child_frame_id = body_id;
    odom.twist.twist.linear.x = t.m_translation_vx;
    odom.twist.twist.linear.y = t.m_translation_vy;
    odom.twist.twist.angular.z = t.m_angular_v;
    odom_pub.publish(odom);
}

void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    double left_angle = msg->position.at(0);
    double right_angle = msg->position.at(1);
    double wl_vel = msg->velocity.at(0);
    double wr_vel = msg->velocity.at(1);

    turtlelib::WheelVel v(wl_vel, wr_vel);
    turtlelib::Twist2D t = diff_drive.wheelvelTotwist(v);

    diff_drive.FK_Calculate(left_angle, right_angle);
    transform = diff_drive.getpose();

    // ROS_INFO("%f, %f, %f :", transform.translation().x, transform.translation().y, transform.rotation());
    //  ROS_INFO("left_angle,right_angle: %f, %f", left_angle, right_angle);
    diff_drive_slam.FK_Calculate(left_angle, right_angle);
    turtlelib::Transform2D transform_slam = diff_drive_slam.getpose();

    static int cnt = 0;
    if (cnt < 1)
    {
        cnt++;
        return;
    }
    else
        cnt = 0;

    // EKF update
    odom_pose_publish(transform_slam, t);
    EKF_update(transform_slam, t);

    diff_drive_slam.setpose(turtlelib::Transform2D(turtlelib::Vector2D(epsilon(1), epsilon(2)), epsilon(0)));

    // std::cout << "epsilon: " << std::endl
    //           << epsilon << std::endl;
    //  std::cout<<"sigma: "<<std::endl<<sigma<<std::endl;
}

bool broadcast_transform(tf2_ros::TransformBroadcaster &odom_broadcaster)
{
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = odom_id;
    tf.child_frame_id = body_id;

    // turtlelib::Transform2D transform = diff_drive.getpose();
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(transform.rotation());
    tf.transform.translation.x = transform.translation().x;
    tf.transform.translation.y = transform.translation().y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = q;
    // ROS_INFO("[x,y,theta]: %f,%f,%f",transform.translation().x,transform.translation().y,transform.rotation());
    odom_broadcaster.sendTransform(tf);

    return true;
}

bool broadcast_transform_maptoodom(tf2_ros::TransformBroadcaster &odom_broadcaster)
{

    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";

    // turtlelib::Transform2D transform = diff_drive.getpose();
    turtlelib::Transform2D T_m_b = turtlelib::Transform2D(turtlelib::Vector2D(epsilon(1), epsilon(2)), epsilon(0));
    turtlelib::Transform2D T_o_b = turtlelib::Transform2D(turtlelib::Vector2D(transform.translation().x, transform.translation().y), transform.rotation());
    turtlelib::Transform2D T_m_o = T_m_b * (T_o_b.inv());

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(T_m_o.rotation());
    tf.transform.translation.x = T_m_o.translation().x;
    tf.transform.translation.y = T_m_o.translation().y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = q;

    // ROS_INFO("[x,y,theta]: %f,%f,%f",epsilon(1),epsilon(2),epsilon(0));
    // ROS_INFO("[x,y,theta]2: %f,%f,%f",transform.translation().x,transform.translation().y,transform.rotation());

    odom_broadcaster.sendTransform(tf);

    PoseStamped.header.stamp = ros::Time::now();
    PoseStamped.pose.position.x = T_m_b.translation().x;
    PoseStamped.pose.position.y = T_m_b.translation().y;
    PoseStamped.pose.position.z = 0.0;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(T_m_b.rotation());

    PoseStamped.pose.orientation.x = goal_quat.x;
    PoseStamped.pose.orientation.y = goal_quat.y;
    PoseStamped.pose.orientation.z = goal_quat.z;
    PoseStamped.pose.orientation.w = goal_quat.w;
    Path.header.stamp = ros::Time::now();
    Path.poses.push_back(PoseStamped);
    path_pub.publish(Path);

    return true;
}

bool param_initialization(ros::NodeHandle &nh)
{
    nh.getParam("/track_width", track_width);
    nh.getParam("/wheel_radius", wheel_radius);
    nh.getParam("/initial_pose/x_0", x_0);
    nh.getParam("/initial_pose/y_0", y_0);
    nh.getParam("/initial_pose/theta_0", theta_0);

    nh.getParam("/marker_radius", marker_radius);
    nh.getParam("/dist_threshold", dist_threshold);

    epsilon(0) = theta_0;
    epsilon(1) = x_0;
    epsilon(2) = y_0;

    return true;
}
