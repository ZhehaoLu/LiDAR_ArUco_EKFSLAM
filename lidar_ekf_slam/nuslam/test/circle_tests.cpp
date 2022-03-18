#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

TEST_CASE("adding duration gives future time", "[ROS Time]")
{

    ros::NodeHandle nh("~"); // this initializes time
    ros::Time time_now = ros::Time::now();
    ros::Time time_future = time_now + ros::Duration(5.0);
    REQUIRE(time_future > time_now);
}

static double threshold_singularvalue = 10e-12;
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

        std::cout << "singular_value" << singular_value << std::endl;

        if (singular_value.size() != 4)
            break;

        if (singular_value(3) < threshold_singularvalue)
        {
            Eigen::MatrixXd A = V.col(3);
            std::cout << "A: " << A << std::endl;
            double a = (-A(1)) / (2 * A(0));
            double b = (-A(2)) / (2 * A(0));
            double R = std::sqrt((std::pow(A(1), 2) + std::pow(A(2), 2) - 4 * A(0) * A(3)) / (4 * std::pow(A(0), 2)));
            circle C = circle(R, turtlelib::Vector2D(a + x_mean, b + y_mean));
            std::cout << "i: " << i << " Radius: " << R << "; a " << a + x_mean << "; b " << b + y_mean << std::endl;
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
            circle_vec.push_back(C);
        }
    }

    // for(unsigned int i=0;i<circle_vec.size();i++){
    //     std::cout<<"i: "<<i<<" "<<circle_vec[i].m_radius<<" "<<circle_vec[i].m_center.x<<" "<<circle_vec[i].m_center.y<<std::endl;
    // }

    return circle_vec;
}

TEST_CASE("TEST1: circle fitting algorithm", "[std::vector<circle> circle_fitting(std::vector<std::vector<turtlelib::Vector2D>> &cluster_vec)]")
{
    ros::NodeHandle nh;
    std::vector<std::vector<turtlelib::Vector2D>> cluster_vec(1,std::vector<turtlelib::Vector2D>{
        turtlelib::Vector2D(1,7),
        turtlelib::Vector2D(2,6),
        turtlelib::Vector2D(5,8),
        turtlelib::Vector2D(7,7),
        turtlelib::Vector2D(9,5),
        turtlelib::Vector2D(3,7)
    });
    
    std::vector<circle> circle = circle_fitting(cluster_vec);
    Approx center_x = Approx(4.615482).epsilon(10e-4);
    Approx center_y = Approx(2.807354).epsilon(10e-4);
    Approx radius = Approx(4.8275).epsilon(10e-4);
    REQUIRE(circle.at(0).m_center.x==center_x);
    REQUIRE(circle.at(0).m_center.y==center_y);
    REQUIRE(circle.at(0).m_radius==radius);
}

TEST_CASE("TEST2: circle fitting algorithm", "[std::vector<circle> circle_fitting(std::vector<std::vector<turtlelib::Vector2D>> &cluster_vec)]")
{
    ros::NodeHandle nh;
    std::vector<std::vector<turtlelib::Vector2D>> cluster_vec(1,std::vector<turtlelib::Vector2D>{
        turtlelib::Vector2D(-1,0),
        turtlelib::Vector2D(-0.3,-0.06),
        turtlelib::Vector2D(0.3,0.1),
        turtlelib::Vector2D(1,0)
    });
    
    std::vector<circle> circle = circle_fitting(cluster_vec);
    Approx center_x = Approx(0.4908357).epsilon(10e-4);
    Approx center_y = Approx(-22.15212).epsilon(10e-4);
    Approx radius = Approx(22.17979).epsilon(10e-4);
    REQUIRE(circle.at(0).m_center.x==center_x);
    REQUIRE(circle.at(0).m_center.y==center_y);
    REQUIRE(circle.at(0).m_radius==radius);
}