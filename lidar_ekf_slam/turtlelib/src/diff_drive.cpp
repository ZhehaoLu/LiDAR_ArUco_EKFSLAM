#include "turtlelib/diff_drive.hpp"
#include <iostream>

namespace turtlelib{

    WheelVel::WheelVel(){};

    WheelVel::WheelVel(double ul,double ur):m_ul(ul),m_ur(ur){};

    DiffDrive::DiffDrive():m_pose(Vector2D(0.0,0.0),0.0),m_track_width(0.8),m_wheel_radius(0.033),m_wheelvel(WheelVel(0.0,0.0)),m_leftwheel_last(0.0),m_rightwheel_last(0.0){};

    DiffDrive::DiffDrive(Transform2D pose, double track_width, double wheel_radius):m_pose(pose),m_track_width(track_width),m_wheel_radius(wheel_radius),m_wheelvel(WheelVel(0.0,0.0)),m_leftwheel_last(0.0),m_rightwheel_last(0.0){};

    WheelVel DiffDrive::IK_Calculate(Twist2D t){
        if(t.m_translation_vy!=0){
            throw std::logic_error("Twist cannot be accomplished without the wheels slipping");
            return WheelVel(0,0);
        }
        else{
            double ul = (t.m_translation_vx - this->m_track_width/2 * t.m_angular_v)/this->m_wheel_radius;
            double ur = (t.m_translation_vx + this->m_track_width/2 * t.m_angular_v)/this->m_wheel_radius;
            return WheelVel(ul,ur);
        }
    }

    Twist2D DiffDrive::wheelvelTotwist(WheelVel v){

        double angular_v=this->m_wheel_radius * (-v.m_ul+v.m_ur)/this->m_track_width;
        double translation_v=this->m_wheel_radius * (v.m_ul + v.m_ur)/2.0;
        return Twist2D(angular_v,translation_v,0);
    }

    WheelVel DiffDrive::FK_Calculate(double leftwheel_cur, double rightwheel_cur){
        
        double ul = normalize_angle(leftwheel_cur-m_leftwheel_last);
        double ur = normalize_angle(rightwheel_cur-m_rightwheel_last);
        m_leftwheel_last=normalize_angle(leftwheel_cur);
        m_rightwheel_last=normalize_angle(rightwheel_cur);
        m_wheelvel=WheelVel(ul,ur);
        turtlelib::Twist2D t = this->wheelvelTotwist(m_wheelvel);
        Transform2D tf = integrate_twist(t);
        m_pose*=tf;
        return m_wheelvel;
    }

    Transform2D DiffDrive::getpose(){return this->m_pose;}

    void DiffDrive::setpose(turtlelib::Transform2D transform){
        m_pose=transform;
    }

    void DiffDrive::setvelocity(turtlelib::WheelVel v){
        m_wheelvel = v;
    }
}