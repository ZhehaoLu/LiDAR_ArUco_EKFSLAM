/// \file
/// \brief Simulate and visualize turtlebot3
///
/// \author Jonas Lu (Zhehao Lu)
/// \date March 16 2022
/// \copyright All rights are reserved by Center For Robotics And Biosystems (CRB) of Northwestern University
///
/// 
/// PARAMETERS:
///     time_step_cnt (int): The timer for the program
///     x_0,y_0,theta_0 (double): Initial (x,y,theta) position of the robot
///     loop_rate_setting (int): Loop rate of ROS
///     actual_x,actual_y,actual_theta (double): Current x y theta position of the robot
///     marker_x_robot, marker_y_robot, marker_x, marker_y, marker_x3, marker_y3, marker_radius (double): Three marker (x,y) position and marker radius
///     js: Joint state message variable
/// PUBLISHERS:
///     marker_pub (visualization_msgs::Marker): Publish obstacles
///     timestep_pub (std_msgs::UInt64): Publish timestep
///     jointstate_pub (sensor_msgs::JointState): Publish jointstate
///     transform_pub (geometry_msgs::TransformStamped): Publish transformation matrix between world and red/base_footprint
///     fake_sensor_pub (visualization_msgs::MarkerArray): Publish fake sensor data for marker visualization
///     sim_lidar_pub (sensor_msgs::LaserScan): Publish simulated data for lidar range
/// SERVICES:
///     reset_service (std_srvs::Trigger): Reset timestep and turtlebot position
///     teleport_service (nusim::teleport): Moving the robot to a desired (x,y,theta) pose

#include "ros/ros.h"
#include "std_msgs/UInt64.h"
#include "nusim/reset.h"
#include "nusim/teleport.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "std_srvs/Trigger.h"
#include "nav_msgs/Odometry.h"
#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <vector>
#include <string>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <random>
#include "nav_msgs/Path.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

static constexpr double PI = 3.14159265358979323846;
static int time_step_cnt = 0;
static double x_0 = 0.0, y_0 = 0.0, theta_0 = 0.0;
static double x_length = 0.0, y_length = 0.0, wall_thickness = 0.0;
static int loop_rate_setting = 100;
static double actual_x = 0.0, actual_y = 0.0, actual_theta = 0.0;
static double marker_radius = 1.0;
static int marker_number = 0;
static sensor_msgs::JointState js;
static double left_wheel_angle = 0.0, right_wheel_angle = 0.0;
static double wheel_cmd_max = 0.0;
static double motor_rotate_maxvel = 0.0;
static double encoder_ticks_to_rad = 0.0;

static double track_width = 0.0, wheel_radius = 0.0;
static double left_encoder = 0.0, right_encoder = 0.0;
ros::Publisher sensor_data_pub;
ros::Publisher fake_sensor_pub;
ros::Publisher sim_lidar_pub;
ros::Publisher path_pub;
nav_msgs::Path Path;
geometry_msgs::PoseStamped poseStamped;

static double gaussian_mean = 0.0, gaussian_variance = 0.0;
static double slip_min = 0.0, slip_max = 0.0;
static double basic_sensor_mean = 0.0, basic_sensor_variance = 0.0;
static double max_range_visible = 0.0;
static double sim_lidar_mean = 0.0,sim_lidar_variance =0.0;

static std::vector<double> marker_x(1,0.0),marker_y(1,0.0);
static std::vector<double> fake_marker_x(1,0.0),fake_marker_y(1,0.0);
static double collision_radius=0.0;
static std::string scene="";
static turtlelib::DiffDrive diff_drive;

/// \brief Control the turtle to a desired (x,y,theta) pose
/// \return true or false
bool teleport_callback(nusim::teleport::Request &req, nusim::teleport::Response &);

/// \brief Reset timestep and turtlebot position
/// \return true or false
bool reset_callback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res);

/// \brief Reset timestep and turtlebot position
void pose_callback(const turtlesim::Pose &msg);

/// \brief Broadcast the transformation between world and red/base_footprint
/// \param transform_pub - the publisher variable that execute the order
/// \return true or false
bool broadcast_transform(ros::Publisher *transform_pub);

/// \brief Get initial parameter from yaml file
/// \param nh - the nodehandle used to call the getparam function
/// \return true or false
bool param_initialization(ros::NodeHandle &nh);

/// \brief Get random value
std::mt19937 &get_random()
{
    static std::random_device rd{};
    static std::mt19937 mt{rd()};
    return mt;
}

/// \brief Transform wheel_cmd msg into sensor data and tf information
void wheel_cmd_callback(const nuturtlebot_msgs::WheelCommands &msg);

/// \brief Visualize fake sensor data into markerarray topic 
void publish_fake_sensor(ros::Publisher &fake_sensor_pub, ros::NodeHandle &nh);

/// \brief Detect the value of range in fake lidar data
double obstacle_detection(double theta, double x_obs, double y_obs, double r_obs);

/// \brief Publish simulated lidar data
void publish_sim_lidar(ros::Publisher &sim_lidar_pub);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh;
    param_initialization(nh);
    ros::Rate loop_rate(loop_rate_setting);
    //obstacles
    marker_x.resize(marker_number);
    marker_y.resize(marker_number);
    std::string markername_x = "/marker_x";
    std::string markername_y = "/marker_y";
    for (int i = 0; i < marker_number; i++)
    {
        std::string markername_xi = markername_x + std::to_string(i);
        std::string markername_yi = markername_y + std::to_string(i);
        nh.getParam(markername_xi, marker_x[i]);
        nh.getParam(markername_yi, marker_y[i]);
    }

    std::vector<ros::Publisher> marker_pubs(marker_number);
    for (auto &marker_pub : marker_pubs)
        marker_pub = nh.advertise<visualization_msgs::Marker>("obstacles", 1000, true);
    std::vector<visualization_msgs::Marker> markers(marker_number);

    int cnt = 0;
    std::string markerns = "obstacles/obstacle_";
    for (auto &marker : markers)
    {
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = markerns + std::to_string(cnt);
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = marker_x[cnt];
        marker.pose.position.y = marker_y[cnt];
        // ROS_INFO("pose [x,y]: %d,%f,%f",cnt,marker_x[cnt],marker_y[cnt]);
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker_radius;
        marker.scale.y = marker_radius;
        marker.scale.z = 0.25;
        marker.color.a = 1.0;
        marker.color.r = 0.8;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_pubs[cnt].publish(marker);
        cnt++;
    }

    // walls
    std::vector<ros::Publisher> wall_pubs(4);
    for (auto &wall_pub : wall_pubs)
        wall_pub = nh.advertise<visualization_msgs::Marker>("Wall", 1000, true);
    std::vector<visualization_msgs::Marker> walls(4);

    double scale[4][4] = {
        {0.0, -0.5 * y_length, x_length, wall_thickness},
        {0.0, 0.5 * y_length, x_length, wall_thickness},
        {-0.5 * x_length, 0.0, wall_thickness, y_length},
        {0.5 * x_length, 0.0, wall_thickness, y_length}};

    cnt = 0;
    std::string wallns = "Walls/Wall_";
    for (auto &wall : walls)
    {
        wall.header.frame_id = "world";
        wall.header.stamp = ros::Time();
        wall.ns = wallns + std::to_string(cnt);
        wall.type = visualization_msgs::Marker::CUBE;
        wall.action = visualization_msgs::Marker::ADD;
        wall.pose.position.x = scale[cnt][0];
        wall.pose.position.y = scale[cnt][1];
        wall.pose.position.z = 0;
        wall.pose.orientation.x = 0.0;
        wall.pose.orientation.y = 0.0;
        wall.pose.orientation.z = 0.0;
        wall.pose.orientation.w = 1.0;
        wall.scale.x = scale[cnt][2];
        wall.scale.y = scale[cnt][3];
        wall.scale.z = 0.25;
        wall.color.a = 1.0;
        wall.color.r = 0.8;
        wall.color.g = 0.0;
        wall.color.b = 0.0;
        wall_pubs.at(cnt).publish(wall);
        cnt++;
    }

    if(scene=="nuwall"){
        ROS_INFO("nuwall in the loop.");
        while(ros::ok()){
            ros::spinOnce();
            loop_rate.sleep();
        }
        ROS_INFO("nuwall is closed.");
        return 0;
    }

    actual_x = x_0;
    actual_y = y_0;
    actual_theta = theta_0;

    ros::Publisher timestep_pub = nh.advertise<std_msgs::UInt64>("timestep", 1000);

    ros::ServiceServer reset_service = nh.advertiseService("reset", reset_callback);

    ros::Publisher jointstate_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

    js.name.push_back("red/wheel_left_joint");
    js.name.push_back("red/wheel_right_joint");
    js.position.push_back(0);
    js.position.push_back(0);

    ros::Subscriber transform_sub = nh.subscribe("nusim/pose_sub", 1000, &pose_callback);
    ros::Publisher transform_pub = nh.advertise<geometry_msgs::TransformStamped>("nusim/pose_pub", 50);
    ros::ServiceServer teleport_service = nh.advertiseService("teleport", teleport_callback);

    sensor_data_pub = nh.advertise<nuturtlebot_msgs::SensorData>("/sensor_data", 50);
    fake_sensor_pub = nh.advertise<visualization_msgs::MarkerArray>("/fake_sensor", 50);
    sim_lidar_pub = nh.advertise<sensor_msgs::LaserScan>("/sim_lidar",50);
    
    path_pub = nh.advertise<nav_msgs::Path>("path", 1000);
    Path.header.frame_id = "world";
    poseStamped.header.frame_id = "red/base_footprint";

    ros::Subscriber wheel_cmd_sub = nh.subscribe("/wheel_cmd", 1000, wheel_cmd_callback);

    ROS_INFO("nusim in the loop.");

    static int hz_cnt=0;
    while (ros::ok())
    {
        std_msgs::UInt64 time_step;
        time_step.data = ++time_step_cnt;
        // ROS_INFO("%i",time_step.data);
        timestep_pub.publish(time_step);

        js.header.frame_id = "odom";
        js.header.stamp = ros::Time::now();
        jointstate_pub.publish(js);

        broadcast_transform(&transform_pub);

        if(hz_cnt==19){
            //fake sensor - 5Hz
            publish_fake_sensor(fake_sensor_pub, nh);
            //lidar - 5Hz
            publish_sim_lidar(sim_lidar_pub);
            hz_cnt=0;
        }
        else hz_cnt++;

        ros::spinOnce();
        loop_rate.sleep();
        
    }
    ROS_INFO("nusim is closed.");
    return 0;
}

bool teleport_callback(nusim::teleport::Request &req, nusim::teleport::Response &)
{
    actual_x = req.req_x;
    actual_y = req.req_y;
    actual_theta = req.req_theta;
    ROS_INFO("nusim teleport.");
    return true;
}

void publish_fake_sensor(ros::Publisher &fake_sensor_pub, ros::NodeHandle &nh)
{

    turtlelib::Transform2D tf_rw = turtlelib::Transform2D(turtlelib::Vector2D(actual_x, actual_y), actual_theta);
    turtlelib::Transform2D tf_wr = tf_rw.inv();

    fake_marker_x.resize(marker_number);
    fake_marker_y.resize(marker_number);

    std::string markername_x = "/marker_x";
    std::string markername_y = "/marker_y";

    for (int i = 0; i < marker_number; i++)
    {
        std::string markername_xi = markername_x + std::to_string(i);
        std::string markername_yi = markername_y + std::to_string(i);
        nh.getParam(markername_xi, fake_marker_x[i]);
        nh.getParam(markername_yi, fake_marker_y[i]);
    }
    visualization_msgs::MarkerArray markerarray;
    std::vector<visualization_msgs::Marker> fake_markers(marker_number);
    int cnt = 0;
    std::string markerns = "fake_sensor";
    for (auto &marker : fake_markers)
    {

        std::normal_distribution<> pos_distribution(basic_sensor_mean, basic_sensor_variance);
        double pos_noise = pos_distribution(get_random());

        turtlelib::Vector2D v(fake_marker_x[cnt], fake_marker_y[cnt]);
        turtlelib::Vector2D res = tf_wr(v);
        marker.header.frame_id = "red/base_footprint";
        marker.header.stamp = ros::Time();
        marker.ns = markerns + std::to_string(cnt);
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.pose.position.x = res.x + pos_noise;
        marker.pose.position.y = res.y + pos_noise;
        // ROS_INFO("pose [x,y]: %d,%f,%f",cnt,marker_x[cnt],marker_y[cnt]);
        double dist = (res.x + pos_noise) * (res.x + pos_noise) + (res.y + pos_noise) * (res.y + pos_noise);
        if (dist > max_range_visible * max_range_visible)
            marker.action = visualization_msgs::Marker::DELETE;
        else
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
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markerarray.markers.emplace_back(marker);
        cnt++;
    }
    fake_sensor_pub.publish(markerarray);
}

double obstacle_detection(double theta, double x_obs, double y_obs, double r_obs)
{
    //if(turtlelib::almost_equal(theta,2.0*PI *270/360.0,0.001) && turtlelib::almost_equal(x_obs,0.0,0.001)) return -y_obs-marker_radius;
    //if(turtlelib::almost_equal(theta,2.0*PI *90/360.0,0.001) && turtlelib::almost_equal(x_obs,0.0,0.001)) return y_obs-marker_radius;

    double x_robot = -x_obs, y_robot = -y_obs;
    double k = std::tan(theta);
    double b = y_robot - k * x_robot;
    double x, y;
    if (x_obs >= 0)
    {
        x = x_robot + 0.0001;
        y = k * x + b;
    }
    else
    {
        x = x_robot - 0.0001;
        y = k * x + b;
    }

    
    double dx = x - x_robot;
    double dy = y - y_robot;
    double dr = std::sqrt(std::pow(dx,2) + std::pow(dy,2));
    double D = x_robot * y - x * y_robot;
    double sgn=dy<0?-1:1;
    double delta = std::pow(r_obs,2) * std::pow(dr,2)  - std::pow(D,2);

    double distance;
    double point_x, point_y;


    if (delta < 0)
    {
        distance = 3.5;
        return distance;
    }
    else if (delta == 0)
    {
        point_x = D * dy / (std::pow(dr,2));
        point_y = -D * dx / (std::pow(dr,2));
    }
    else
    {
        double point_x1 = (D * dy - sgn * dx * std::sqrt(delta)) / (std::pow(dr,2));
        double point_y1 = (-D * dx - std::abs(dy) * std::sqrt(delta)) / (std::pow(dr,2));
        double point_x2 = (D * dy + sgn * dx * std::sqrt(delta)) / (std::pow(dr,2));
        double point_y2 = (-D * dx + std::abs(dy) * std::sqrt(delta)) / (std::pow(dr,2));

        double dist1 = std::pow(point_x1-x_robot,2) + std::pow(point_y1 - y_robot, 2);
        double dist2 = std::pow(point_x2-x_robot,2) + std::pow(point_y2 - y_robot, 2);

        if(dist1<dist2){
            point_x = point_x1;
            point_y=point_y1;
        }
        else {
            point_x = point_x2;
            point_y=point_y2;
        }
    }


    distance = std::sqrt(std::pow(point_x - x_robot, 2) + std::pow(point_y - y_robot, 2));

    if (((point_x - x_robot) / distance / std::cos(theta)) <= 0 || ((point_y - y_robot) / distance / std::sin(theta)) <= 0)
    {
        distance = 3.5;
        return distance;
    }

    return distance;
}

void publish_sim_lidar(ros::Publisher &sim_lidar_pub){
    static sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "red/base_footprint";
    scan.angle_min=0.0;
    scan.angle_max=6.2657318115234375;
    scan.angle_increment=0.01745329238474369;
    scan.time_increment=0.0005592841189354658;
    scan.range_min=0.11999999731779099;
    scan.range_max=3.5;
    scan.ranges.resize(360);
    scan.intensities.resize(360);

    std::normal_distribution<> lidar_distribution(sim_lidar_mean, sim_lidar_variance);
    
    for(unsigned int i=0;i<360;i++){
        double lim_range=3.5;
        for(int j=0;j<marker_number;j++){
            turtlelib::Transform2D tf_rw = turtlelib::Transform2D(turtlelib::Vector2D(actual_x, actual_y), actual_theta);
            turtlelib::Transform2D tf_wr = tf_rw.inv();
            turtlelib::Vector2D v(fake_marker_x[j],fake_marker_y[j]);
            turtlelib::Vector2D res = tf_wr(v);

            lim_range = std::min(lim_range,obstacle_detection(2.0*PI *i/360.0,res.x,res.y,marker_radius));
        }
        //double lidar_noise = lidar_distribution(get_random());
        //if(lim_range=3.5) scan.ranges[i]=lim_range;
        //else scan.ranges[i]=lim_range+lidar_noise;
        scan.ranges[i]=lim_range;
        scan.intensities[i]=1;
    }
    sim_lidar_pub.publish(scan);
}

bool reset_callback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
    time_step_cnt = 0;
    res.success = 1;
    actual_x = x_0;
    actual_y = y_0;
    actual_theta = theta_0;
    res.message = "time step reset to zero.";
    ROS_INFO("time step reset to zero.");
    return true;
}

void pose_callback(const turtlesim::Pose &msg)
{
    actual_x = msg.x;
    actual_y = msg.y;
    actual_theta = msg.theta;
}

bool broadcast_transform(ros::Publisher *transform_pub)
{
    static tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "world";
    tf.child_frame_id = "red/base_footprint";
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(actual_theta);
    tf.transform.translation.x = actual_x;
    tf.transform.translation.y = actual_y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = q;
    // //ROS_INFO("%f",actual_x);
    odom_broadcaster.sendTransform(tf);
    transform_pub->publish(tf);

    return true;
}

void wheel_cmd_callback(const nuturtlebot_msgs::WheelCommands &msg)
{

    double left_velocity = static_cast<double>(msg.left_velocity);
    double right_velocity = static_cast<double>(msg.right_velocity);

    std::normal_distribution<> distribution(gaussian_mean, gaussian_variance);

    double noise = distribution(get_random());

    if (!turtlelib::almost_equal(left_velocity, 0))
        left_velocity += noise;
    if (!turtlelib::almost_equal(right_velocity, 0))
        right_velocity += noise;

    double ul = left_velocity / (double)wheel_cmd_max * (double)motor_rotate_maxvel;
    double ur = right_velocity / (double)wheel_cmd_max * (double)motor_rotate_maxvel;
    // ROS_INFO("[ul,ur]: %f,%f", ul,ur);
    turtlelib::WheelVel v = turtlelib::WheelVel(ul, ur);

    left_encoder = left_encoder + (v.m_ul / 10.0 / (2 * PI) * encoder_ticks_to_rad);
    right_encoder = right_encoder + (v.m_ur / 10.0 / (2 * PI) * encoder_ticks_to_rad);

    // ROS_INFO("[le,re]: %f,%f",left_encoder,right_encoder);

    nuturtlebot_msgs::SensorData sensor_data;
    sensor_data.left_encoder = std::round(left_encoder);
    sensor_data.right_encoder = std::round(right_encoder);
    // ROS_INFO("[left_encoder,right_encoder]: [%f,%f]",left_encoder,right_encoder);
    sensor_data_pub.publish(sensor_data);

    left_wheel_angle = left_wheel_angle + v.m_ul / 10.0;
    right_wheel_angle = right_wheel_angle + v.m_ur / 10.0;
    // ROS_INFO("[ul,ur]: %f,%f",left_wheel_angle,right_wheel_angle);

    std::uniform_real_distribution<double> slip_distribution(slip_min, slip_max);
    double slip_noise= distribution(get_random());

    left_wheel_angle += slip_noise * v.m_ul / 10.0;
    right_wheel_angle += slip_noise * v.m_ur / 10.0;
    // ROS_INFO("[noise]: %f,%f",left_wheel_angle,right_wheel_angle);

    static turtlelib::DiffDrive diff_drive = turtlelib::DiffDrive(turtlelib::Transform2D(turtlelib::Vector2D(x_0, y_0), theta_0), track_width, wheel_radius);
    diff_drive.FK_Calculate(left_wheel_angle, right_wheel_angle);
    actual_x = diff_drive.getpose().translation().x;
    actual_y = diff_drive.getpose().translation().y;
    actual_theta = diff_drive.getpose().rotation();

    // ROS_INFO("%f,%f,%f:",diff_drive.getpose().translation().x,diff_drive.getpose().translation().y,diff_drive.getpose().rotation());

    for(int i=0;i<marker_number;i++){
        double dist = marker_radius + collision_radius;
        if(std::pow(marker_x[i]-actual_x,2) + std::pow(marker_y[i]-actual_y,2)<= std::pow(dist,2)){
            double K = (marker_y[i] - actual_y)/(marker_x[i]-actual_x);
            double B = actual_y - K * actual_x;
            double a = 1.0 + K * K;
            double b = 2 * B * K - 2 * marker_y[i] * K -2 * marker_x[i];
            double c = B * B - 2 * B * marker_y[i] + std::pow(marker_x[i],2) + std::pow(marker_y[i],2) - dist * dist;

            if(actual_x < marker_x[i]){
                actual_x = (-b - std::pow(b * b - 4 * a * c, 0.5)) / (2 * a);
            }
            else actual_x = (-b + std::pow(b * b - 4 * a * c, 0.5)) / (2 * a);
            actual_y = K * actual_x + B;
            break;
        }
    }

    // path

    turtlelib::Transform2D transform_modified(turtlelib::Vector2D(actual_x,actual_y),actual_theta);
    diff_drive.setpose(transform_modified);
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.pose.position.x = actual_x;
    poseStamped.pose.position.y = actual_y;
    poseStamped.pose.position.z = 0.0;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(actual_theta);
    poseStamped.pose.orientation.x = goal_quat.x;
    poseStamped.pose.orientation.y = goal_quat.y;
    poseStamped.pose.orientation.z = goal_quat.z;
    poseStamped.pose.orientation.w = goal_quat.w;
    Path.header.stamp = ros::Time::now();

    Path.poses.push_back(poseStamped);
    path_pub.publish(Path);

}

bool param_initialization(ros::NodeHandle &nh)
{
    nh.getParam("/initial_setting/loop_rate_setting", loop_rate_setting);
    nh.getParam("/initial_pose/x_0", x_0);
    nh.getParam("/initial_pose/y_0", y_0);
    nh.getParam("/initial_pose/theta_0", theta_0);
    nh.getParam("/marker_radius", marker_radius);
    nh.getParam("/marker_number", marker_number);
    nh.getParam("/track_width", track_width);
    nh.getParam("/wheel_radius", wheel_radius);
    nh.getParam("/wall_settings/x_length", x_length);
    nh.getParam("/wall_settings/y_length", y_length);
    nh.getParam("/wall_settings/wall_thickness", wall_thickness);
    nh.getParam("/wheel_cmd_max", wheel_cmd_max);
    nh.getParam("/encoder_ticks_to_rad", encoder_ticks_to_rad);
    nh.getParam("/motor_rotate_maxvel", motor_rotate_maxvel);

    nh.getParam("/gaussian_distribution/mean", gaussian_mean);
    nh.getParam("/gaussian_distribution/variance", gaussian_variance);

    nh.getParam("/uniform_distribution/slip_min", slip_min);
    nh.getParam("/uniform_distribution/slip_max", slip_max);

    nh.getParam("/basic_sensor_noise/basic_sensor_mean", basic_sensor_mean);
    nh.getParam("/basic_sensor_noise/basic_sensor_variance", basic_sensor_variance);
    nh.getParam("/max_range", max_range_visible);

    nh.getParam("/collision_radius",collision_radius);
    nh.getParam("/red/nusim/scene",scene);

    nh.getParam("/sim_lidar_noise/sim_lidar_mean",sim_lidar_mean);
    nh.getParam("/sim_lidar_noise/sim_lidar_variance",sim_lidar_variance);

    return true;
}