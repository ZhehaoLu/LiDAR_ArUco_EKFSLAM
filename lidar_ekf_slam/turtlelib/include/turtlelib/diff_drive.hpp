#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Kinematics calculations for turtlebot.

#include "turtlelib/rigid2d.hpp"


namespace turtlelib{

    struct WheelVel{
        double m_ul;
        double m_ur;

        /// \brief default constructor to initialze WheelVel as (0,0)
        WheelVel();
        /// \brief default constructor to initialze WheelVel as (vl,vr)
        /// \param ul - the left wheel velocity
        /// \param ur - the right wheel velocity
        WheelVel(double ul,double ur);
    };

    /// \brief models the kinematics of a differential drive robot
    /// with a given wheel track and wheel radius
    class DiffDrive{
        private:
        Transform2D m_pose;

        double m_track_width;
        double m_wheel_radius;
        WheelVel m_wheelvel;
        double m_leftwheel_last;
        double m_rightwheel_last;

        public:
        /// \brief default constructor which creates a robot at (0,0,0)
        DiffDrive();

        /// \brief constructor which set a spesific pose, track width and radius
        /// \param pose - the current pose of the robot
        /// \param track_width - the distance between two wheel centers (robot width)
        /// \param wheel_radius - the raidus of the wheel
        DiffDrive(Transform2D pose, double track_width, double wheel_radius);


        /// \brief compute the wheel velocities required to make the robot move at a given body twist
        /// \param t - the desired twist in the body frame of the robot
        /// \returns - the wheel velocities
        /// \throws std::exception if twist cannot be accomplished     
        WheelVel IK_Calculate(Twist2D t);

        /// \brief determine the body twist of the robot (given its wheel velocities)
        /// \param v - the desired wheel velocities
        /// \returns - the body twist
        /// \throws std::exception if twist cannot be accomplished 
        Twist2D wheelvelTotwist(WheelVel v);


        /// \brief Update the transformation for the robot based on current encoder values
        /// \param leftwheel_cur - left wheel angle (in radians)
        /// \param rightwheel_cur - right wheel angle (in radians)
        /// \return wheel velocities
        WheelVel FK_Calculate(double leftwheel_cur, double rightwheel_cur);

        /// \brief get current robot pose
        /// \return current robot pose
        Transform2D getpose();

        /// \brief reset the odometry so that the robot thinks it is at the requested configuration
        void setpose(turtlelib::Transform2D transform);

        /// \brief set the velocity of wheels
        void setvelocity(turtlelib::WheelVel v);
    };
};

#endif