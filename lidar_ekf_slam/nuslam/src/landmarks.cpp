/// \file
/// \brief landmark detection and publishing
///
/// \author Jonas Lu (Zhehao Lu)
/// \date March 16 2022
/// \copyright All rights are reserved by Center For Robotics And Biosystems (CRB) of Northwestern University
///
/// 
/// PUBLISHES:
///     landmarks (nuslam/TurtleMap): publish the center and radius of the landmarks the node detects
///     clusters (visualization_msgs/Marker): publish markers of clustered points
/// SUBSCRIBERS:
///     scan (sensor_msgs/LaserScan): subscribe to laser data

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include "visualization_msgs/MarkerArray.h"

ros::Subscriber scan_sub;
ros::Publisher sense_landmark_pub;

static double threshold_dist = 0.4;
static double threshold_singularvalue = 10e-12;
static double threshold_minmean = 0.50, threshold_maxmean = 0.75, threshold_variance = 0.15;
static double threshold_minradius = 0.50, threshold_maxradius = 1.50, marker_radius = 0.15;
static double sim_lidar_maxrange = 3.50;
static int min_pointnumber = 3;
static std::string lidar_source = "fake";
static constexpr double PI = 3.14159265358979323846;

class circle
{
public:
    double m_radius;
    turtlelib::Vector2D m_center;

public:
    circle(double radius, double x, double y) : m_radius(radius), m_center(turtlelib::Vector2D(x, y)){};
    circle(double radius, turtlelib::Vector2D center) : m_radius(radius), m_center(center){};
    ~circle(){};
};

turtlelib::Vector2D bear_to_vector(double range, double theta)
{

    double x = range * std::cos(turtlelib::deg2rad(theta));
    double y = range * std::sin(turtlelib::deg2rad(theta));
    return turtlelib::Vector2D(x, y);
}

double diff_calculate(const turtlelib::Vector2D &v1, const turtlelib::Vector2D &v2)
{
    return std::sqrt(std::pow((v1.x - v2.x), 2) + std::pow((v1.y - v2.y), 2));
}

bool circle_verify(const std::vector<turtlelib::Vector2D> &cluster)
{
    turtlelib::Vector2D v_first = cluster.front();
    turtlelib::Vector2D v_last = cluster.back();

    std::vector<double> angles;

    for (unsigned int i = 1; i < cluster.size() - 1; i++)
    {
        turtlelib::Vector2D v_curr = cluster[i];

        // std::cout<<"v_first:"<<v_first<<" v_last:"<<v_last<<" v_curr:"<<v_curr<<std::endl;

        double c = diff_calculate(v_first, v_last);
        double a = diff_calculate(v_first, v_curr);
        double b = diff_calculate(v_last, v_curr);

        double angle = std::acos((std::pow(a, 2) + std::pow(b, 2) - std::pow(c, 2)) / (2 * a * b));
        // std::cout<<"c:"<<c<<" a:"<<a<<" b:"<<b<<" angle:"<<angle<<std::endl;
        angles.emplace_back(angle);
    }

    // std::cout<<"angle number:"<<angles.size()<<std::endl;

    double sum = 0.0;
    for (auto &angle : angles)
        sum += angle;
    double mean = sum / ((double)cluster.size() - 2.0);
    // std::cout<<"mean:"<<mean<<std::endl;

    double variance = 0.0;
    for (auto &angle : angles)
        variance += std::pow(angle - mean, 2);
    variance /= ((double)cluster.size() - 2.0);
    variance = std::sqrt(variance);

    // std::cout<<"variance:"<<variance<<std::endl;

    if (variance < threshold_variance && mean > threshold_minmean * PI && mean < threshold_maxmean * PI)
        return true;
    else
        return false;
}

bool radius_verify(const double R)
{
    double R_min = marker_radius * threshold_minradius;
    double R_max = marker_radius * threshold_maxradius;
    if (R > R_min && R < R_max)
        return true;
    else
        return false;
}

std::vector<circle> circle_fitting(std::vector<std::vector<turtlelib::Vector2D>> &cluster_vec)
{

    std::vector<circle> circle_vec;
    std::vector<std::vector<turtlelib::Vector2D>> cluster_verify = cluster_vec;

    for (unsigned int i = 0; i < cluster_vec.size(); i++)
    {
        double x_mean = 0.0, y_mean = 0.0;
        for (unsigned int j = 0; j < cluster_vec[i].size(); j++)
        {
            x_mean += cluster_vec[i][j].x;
            y_mean += cluster_vec[i][j].y;
        }
        x_mean /= cluster_vec[i].size();
        y_mean /= cluster_vec[i].size();

        // std::cout<<"["<<"x_mean"<<","<<"y_mean"<<"]: "<<x_mean<<" "<<y_mean<<std::endl;
        for (unsigned int j = 0; j < cluster_vec[i].size(); j++)
        {
            cluster_vec[i][j].x -= x_mean;
            cluster_vec[i][j].y -= y_mean;
        }
        std::vector<double> z(cluster_vec[i].size(), 0);
        double z_mean = 0.0;

        for (unsigned int j = 0; j < cluster_vec[i].size(); j++)
        {
            z[j] = std::pow(cluster_vec[i][j].x, 2) + std::pow(cluster_vec[i][j].y, 2);
        }

        for (auto &zi : z)
        {
            z_mean += zi;
        }
        z_mean /= (double)z.size();
        // std::cout<<"["<<"z_mean"<<"]: "<<z_mean<<std::endl;

        Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(z.size(), 4);
        for (unsigned int j = 0; j < z.size(); j++)
        {
            Z(j, 0) = z[j];
            Z(j, 1) = cluster_vec[i][j].x;
            Z(j, 2) = cluster_vec[i][j].y;
            Z(j, 3) = 1;
        }
        // std::cout<<"["<<"Z_matrix"<<"]: "<<Z<<std::endl;
        Eigen::MatrixXd M = (1.0 / (double)z.size()) * Z.transpose() * Z;
        // std::cout<<"["<<"M_matrix"<<"]: "<<M<<std::endl;
        Eigen::Matrix<double, 4, 4> H;
        H << 8 * z_mean, 0, 0, 2,
            0, 1, 0, 0,
            0, 0, 1, 0,
            2, 0, 0, 0;
        // std::cout<<"["<<"H_matrix"<<"]: "<<H<<std::endl;

        Eigen::Matrix<double, 4, 4> H_inv;
        H_inv << 0, 0, 0, 0.5,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0.5, 0, 0, -2 * z_mean;
        // std::cout<<"["<<"Hinv_matrix"<<"]: "<<H_inv<<std::endl;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Z, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd singular_value = svd.singularValues();
        auto U = svd.matrixU();
        auto V = svd.matrixV();

        //std::cout << "singular_value" << singular_value << std::endl;
        //std::cout << "V:" << V << std::endl;

        if (singular_value.size() != 4)
            break;

        if (singular_value(3) < threshold_singularvalue)
        {
            Eigen::MatrixXd A = V.col(3);
            //std::cout << "A: " << A << std::endl;
            double a = (-A(1)) / (2 * A(0));
            double b = (-A(2)) / (2 * A(0));
            double R = std::sqrt((std::pow(A(1), 2) + std::pow(A(2), 2) - 4 * A(0) * A(3)) / (4 * std::pow(A(0), 2)));
            circle C = circle(R, turtlelib::Vector2D(a + x_mean, b + y_mean));
            //std::cout << "i: " << i << " Radius: " << R << "; a " << a + x_mean << "; b " << b + y_mean << std::endl;
            if (circle_verify(cluster_verify[i]) && radius_verify(R))
            //if(circle_verify(cluster_verify[i])) 
            //if(radius_verify(R)) 
            circle_vec.push_back(C);
        }
        else
        {
            Eigen::MatrixXd sigma = singular_value.array().matrix().asDiagonal();
            //     // sigma << singular_value(0), 0, 0, 0,
            //     //     0, singular_value(1), 0, 0,
            //     //     0, 0, singular_value(2), 0,
            //     //     0, 0, 0, singular_value(3);
            //     // std::cout<<"sigma:"<<sigma<<std::endl;

            Eigen::MatrixXd Y = V * sigma * V.transpose();
            Eigen::MatrixXd Q = Y * H_inv * Y;

            Eigen::JacobiSVD<Eigen::MatrixXd> svd_Q(Q, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::VectorXd singular_value_Q = svd_Q.singularValues();
            auto U_Q = svd_Q.matrixU(), V_Q = svd_Q.matrixV();

            // std::cout<<"VQ: "<<singular_value_Q<<std::endl;

            Eigen::MatrixXd A_str = V_Q.col(3);
            // std::cout<<"A_str: "<<A_str<<std::endl;

            Eigen::MatrixXd A = Y.colPivHouseholderQr().solve(A_str);
            // std::cout<<"A: "<<A<<std::endl;
            double a = (-A(1)) / (2 * A(0));
            double b = (-A(2)) / (2 * A(0));
            double R = std::sqrt((std::pow(A(1), 2) + std::pow(A(2), 2) - 4 * A(0) * A(3)) / (4 * std::pow(A(0), 2)));
            circle C = circle(R, turtlelib::Vector2D(a + x_mean, b + y_mean));
            // std::cout << "i: "<<i<<" Radius: " << R << "; a " << a+x_mean << "; b " << b+y_mean << std::endl;
            if (circle_verify(cluster_verify[i]) && radius_verify(R))
            //if(circle_verify(cluster_verify[i]))    
            //if(radius_verify(R)) 
            circle_vec.push_back(C);
        }
    }
    
    //ROS_INFO("number of circles: %i", (int)cluster_vec.size());
    // for(unsigned int i=0;i<circle_vec.size();i++){
    //     std::cout<<"i: "<<i<<" "<<circle_vec[i].m_radius<<" "<<circle_vec[i].m_center.x<<" "<<circle_vec[i].m_center.y<<std::endl;
    // }

    return circle_vec;
}

bool generate_sensed_marker(const std::vector<circle> &circle_vec)
{
    visualization_msgs::MarkerArray markerarray;
    std::string markerns = "circle";
    int cnt = 0;

    //ROS_INFO("number of circles: %i", (int)circle_vec.size());
    for (auto const &circle : circle_vec)
    {
        visualization_msgs::Marker marker;
        if(lidar_source == "real") marker.header.frame_id = "green/base_footprint";
        else marker.header.frame_id = "red/base_footprint";
        marker.header.stamp = ros::Time::now();
        marker.ns = markerns + std::to_string(cnt);
        marker.id = cnt;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.pose.position.x = circle.m_center.x;
        marker.pose.position.y = circle.m_center.y;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.25);
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = circle.m_radius;
        marker.scale.y = circle.m_radius;
        marker.scale.z = 0.25;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        markerarray.markers.emplace_back(marker);
        cnt++;
    }
    sense_landmark_pub.publish(markerarray);
    return true;
}

std::vector<std::vector<turtlelib::Vector2D>> laserscan_to_clustervec(const sensor_msgs::LaserScan &msg)
{
    std::vector<std::vector<turtlelib::Vector2D>> cluster_vec;

    for (int i = 0; i < 360; i++)
    {
        if (msg.ranges[i] > sim_lidar_maxrange - 0.1)
            continue;
        std::vector<turtlelib::Vector2D> cluster;
        turtlelib::Vector2D v_curr = bear_to_vector(msg.ranges[i], i);
        cluster.emplace_back(v_curr);

        int j = 1;
        for (; i + j < 360; j++)
        {

            turtlelib::Vector2D v_prev = bear_to_vector(msg.ranges[i + j - 1], i + j - 1);
            turtlelib::Vector2D v_next = bear_to_vector(msg.ranges[i + j], i + j);
            double dist = diff_calculate(v_prev, v_next);

            if (dist <= threshold_dist)
            {
                cluster.emplace_back(v_next);
            }
            else
            {
                i = i + j - 1;
                break;
            }
        }

        cluster_vec.push_back(cluster);
        if (i + j >= 360)
            break;
    }

    if (!cluster_vec.empty() && diff_calculate(cluster_vec[0][0], cluster_vec[cluster_vec.size() - 1].back()) < threshold_dist)
    {
        cluster_vec[cluster_vec.size() - 1].insert(
            cluster_vec[cluster_vec.size() - 1].end(),
            cluster_vec[0].begin(),
            cluster_vec[0].end());

        cluster_vec.erase(cluster_vec.begin());
    }

    cluster_vec.erase(std::remove_if(
                          cluster_vec.begin(), cluster_vec.end(),
                          [](const std::vector<turtlelib::Vector2D> &v)
                          {
                              return (int)v.size() < min_pointnumber;
                          }),
                      cluster_vec.end());

    //std::cout << "number of clusters: " << cluster_vec.size() << std::endl;

    //ROS_INFO("number of clusters: %i", cluster_vec.size());

    // for(unsigned int i=0;i<cluster_vec.size();i++){
    //     for(unsigned int j=0;j<cluster_vec[i].size();j++){
    //         std::cout<<cluster_vec[i][j]<<" ";
    //     }
    //     std::cout<<std::endl;
    // }
    return cluster_vec;
}

void scan_callback(const sensor_msgs::LaserScan &msg)
{
    std::vector<std::vector<turtlelib::Vector2D>> cluster_vec = laserscan_to_clustervec(msg);
    std::vector<circle> circle_vec = circle_fitting(cluster_vec);
    generate_sensed_marker(circle_vec);
};

bool param_initialization(ros::NodeHandle &nh)
{
    nh.getParam("/circle_fitting/threshold_dist", threshold_dist);
    nh.getParam("/circle_fitting/threshold_singularvalue", threshold_singularvalue);
    nh.getParam("/circle_fitting/threshold_minmean", threshold_minmean);
    nh.getParam("/circle_fitting/threshold_maxmean", threshold_maxmean);
    nh.getParam("/circle_fitting/threshold_variance", threshold_variance);
    nh.getParam("/circle_fitting/threshold_minradius", threshold_minradius);
    nh.getParam("/circle_fitting/threshold_maxradius", threshold_maxradius);
    nh.getParam("/circle_fitting/min_pointnumber", min_pointnumber);
    nh.getParam("/sim_lidar/sim_lidar_maxrange", sim_lidar_maxrange);
    nh.getParam("/marker_radius", marker_radius);
    nh.getParam("/landmarks/lidar_source",lidar_source);
    return true;
}

bool test_param(ros::NodeHandle &)
{
    std::cout << "threshold_dist: " << threshold_dist << std::endl;
    std::cout << "threshold_singularvalue: " << threshold_singularvalue << std::endl;
    std::cout << "threshold_minmean: " << threshold_minmean << std::endl;
    std::cout << "threshold_maxmean: " << threshold_maxmean << std::endl;
    std::cout << "min_pointnumber: " << min_pointnumber << std::endl;
    std::cout << "sim_lidar_maxrange: " << sim_lidar_maxrange << std::endl;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;
    param_initialization(nh);
    //test_param(nh);

    ros::Rate loop_rate(100);
    if(lidar_source =="fake") scan_sub = nh.subscribe("/sim_lidar", 1000, scan_callback);
    else scan_sub = nh.subscribe("/scan", 1000, scan_callback);
    sense_landmark_pub = nh.advertise<visualization_msgs::MarkerArray>("sensed_landmark", 1000);

    ROS_INFO("landmarks node in the loop.");

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("landmarks node in closed.");
    return 0;
}