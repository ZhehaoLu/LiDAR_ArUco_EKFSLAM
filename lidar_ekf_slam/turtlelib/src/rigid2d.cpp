#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace turtlelib;

namespace turtlelib
{
    Vector2D::Vector2D() : x(0.0), y(0.0) {}

    Vector2D::Vector2D(double x, double y) : x(x), y(y){};

    double magnitude(const Vector2D &v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }

    double angle(const Vector2D &v1, const Vector2D &v2)
    {
        double radian = std::atan2(v2.y,v2.x)-std::atan2(v1.y,v1.x);
        if(radian>PI) radian-=2*PI;
        else if(radian<-PI) radian+=2*PI;
        return rad2deg(radian);
    }

    std::ostream &operator<<(std::ostream &os, const Vector2D &v)
    {
        os << "[" << v.x << " " << v.y << "]" << std::endl;
        return os;
    }

    std::istream &operator>>(std::istream &is, Vector2D &v)
    {

        std::string str_x, str_y;
        is >> str_x >> str_y;

        if (str_x.at(0) == '[')
        {
            str_x.erase(str_x.begin());
            str_y.pop_back();
        }

        std::stringstream s_x(str_x), s_y(str_y);
        s_x >> v.x;
        s_y >> v.y;
        return is;
    }

    Vector2D &Vector2D::operator+=(const Vector2D &rhs)
    {
        this->x = this->x + rhs.x;
        this->y = this->y + rhs.y;
        return *this;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D &rhs)
    {
        return Vector2D((lhs.x + rhs.x), (lhs.y + rhs.y));
    }

    Vector2D &Vector2D::operator-=(const Vector2D &rhs)
    {
        this->x = this->x - rhs.x;
        this->y = this->y - rhs.y;
        return *this;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D &rhs)
    {
        return Vector2D((lhs.x - rhs.x), (lhs.y - rhs.y));
    }

    Vector2D &Vector2D::operator*=(const double &rhs)
    {
        this->x = this->x * rhs;
        this->y = this->y * rhs;
        return *this;
    }

    Vector2D operator*(Vector2D lhs, const double &rhs)
    {
        return Vector2D((lhs.x * rhs), (lhs.y * rhs));
    }

    Vector2D operator*(const double &rhs, Vector2D lhs)
    {
        return Vector2D((lhs.x * rhs), (lhs.y * rhs));
    }

    Vector2D normalize_vec(Vector2D vec_in)
    {
        Vector2D vec_out;
        double normalization = std::sqrt(vec_in.x * vec_in.x + vec_in.y * vec_in.y);
        vec_out.x = vec_in.x / normalization;
        vec_out.y = vec_in.y / normalization;
        return vec_out;
    }

    double Vector2D::dot(Vector2D v){

        return (double)(this->x * v.x + this->y * v.y);
    }

    Transform2D::Transform2D() : m_translation(0.0, 0.0), m_radian(0.0){};

    Transform2D::Transform2D(Vector2D trans) : m_translation(trans), m_radian(0.0){};

    Transform2D::Transform2D(double radians) : m_translation(0.0, 0.0), m_radian(radians){};

    Transform2D::Transform2D(Vector2D trans, double radians) : m_translation(trans), m_radian(radians){};

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        Vector2D res;
        res.x = std::cos(this->m_radian) * v.x - sin(this->m_radian) * v.y + this->m_translation.x;
        res.y = std::sin(this->m_radian) * v.x + cos(this->m_radian) * v.y + this->m_translation.y;
        return res;
    }

    Twist2D Transform2D::operator()(Twist2D t) const
    {
        Twist2D res;
        res.m_angular_v = t.m_angular_v;
        res.m_translation_vx = t.m_angular_v * this->m_translation.y + t.m_translation_vx * std::cos(this->m_radian) - t.m_translation_vy * std::sin(this->m_radian);
        res.m_translation_vy = t.m_angular_v * (-this->m_translation.x) + t.m_translation_vx * std::sin(this->m_radian) + t.m_translation_vy * std::cos(this->m_radian);
        return res;
    }

    Transform2D Transform2D::inv() const
    {
        double inv_radian = -this->m_radian;
        double inv_trans_x = -std::cos(this->m_radian) * this->m_translation.x - std::sin(this->m_radian) * this->m_translation.y;
        double inv_trans_y = std::sin(this->m_radian) * this->m_translation.x - std::cos(this->m_radian) * this->m_translation.y;
        Vector2D inv_translation(inv_trans_x, inv_trans_y);
        Transform2D inv_res(inv_translation, inv_radian);
        return inv_res;
    }

    Transform2D &Transform2D::operator*=(const Transform2D &rhs)
    {
        this->m_translation.x = this->m_translation.x + (std::cos(this->m_radian) * rhs.m_translation.x - std::sin(this->m_radian) * rhs.m_translation.y);
        this->m_translation.y = this->m_translation.y + (std::sin(this->m_radian) * rhs.m_translation.x + std::cos(this->m_radian) * rhs.m_translation.y);
        this->m_radian = normalize_angle(this->m_radian + rhs.m_radian);
        return *this;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D &rhs)
    {
        Transform2D tf_res;
        tf_res.m_translation.x = lhs.m_translation.x + (std::cos(lhs.m_radian) * rhs.m_translation.x - std::sin(lhs.m_radian) * rhs.m_translation.y);
        tf_res.m_translation.y = lhs.m_translation.y + (std::sin(lhs.m_radian) * rhs.m_translation.x + std::cos(lhs.m_radian) * rhs.m_translation.y);
        tf_res.m_radian = normalize_angle(lhs.m_radian + rhs.m_radian);
        return tf_res;
    }

    Vector2D Transform2D::translation() const
    {
        return this->m_translation;
    }

    double Transform2D::rotation() const
    {
        return this->m_radian;
    }

    std::ostream &operator<<(std::ostream &os, const Transform2D &tf)
    {
        os << "deg: " << rad2deg(tf.m_radian) << " "
           << "x: " << tf.m_translation.x << " "
           << "y: " << tf.m_translation.y << std::endl;
        return os;
    }

    std::istream &operator>>(std::istream &is, Transform2D &tf)
    {
        std::string str1;
        is >> str1;
        if (str1 == "deg:")
        {
            std::vector<std::string> v_str(5);
            for (int i = 0; i < 5; i++)
            {
                is >> v_str[i];
            }
            std::stringstream str_deg(v_str[0]), str_x(v_str[2]), str_y(v_str[4]);
            double double_deg;
            str_deg >> double_deg;
            tf.m_radian = deg2rad(double_deg);
            str_x >> tf.m_translation.x;
            str_y >> tf.m_translation.y;
        }
        else
        {
            std::vector<std::string> v_str(2);
            for (int i = 0; i < 2; i++)
            {
                is >> v_str[i];
            }
            std::stringstream str_deg(str1), str_x(v_str[0]), str_y(v_str[1]);
            double double_deg;
            str_deg >> double_deg;
            tf.m_radian = deg2rad(double_deg);
            str_x >> tf.m_translation.x;
            str_y >> tf.m_translation.y;
        }
        return is;
    }

    Twist2D::Twist2D() : m_angular_v(0.0), m_translation_vx(0.0), m_translation_vy(0.0){};

    Twist2D::Twist2D(double angular_v, double translation_vx, double translation_vy)
        : m_angular_v(angular_v), m_translation_vx(translation_vx), m_translation_vy(translation_vy){};

    std::ostream &operator<<(std::ostream &os, const Twist2D &t)
    {
        os << "[" << t.m_angular_v << " " << t.m_translation_vx << " " << t.m_translation_vy << "]" << std::endl;
        return os;
    }

    std::istream &operator>>(std::istream &is, Twist2D &t)
    {
        std::string str_angular_v, str_trans_vx, str_trans_vy;
        is >> str_angular_v >> str_trans_vx >> str_trans_vy;

        if (str_angular_v.at(0) == '[')
        {
            str_angular_v.erase(str_angular_v.begin());
            str_trans_vy.pop_back();
        }

        std::stringstream s_angular_v(str_angular_v), s_trans_vx(str_trans_vx), s_trans_vy(str_trans_vy);
        s_angular_v >> t.m_angular_v;
        s_trans_vx >> t.m_translation_vx;
        s_trans_vy >> t.m_translation_vy;
        return is;
    }

    double normalize_angle(double rad)
    {
        double res_rad = rad;
        while (res_rad > 1.0 * PI)
        {
            res_rad -= 2.0 * PI;
        }
        while (res_rad <= -1.0 * PI)
        {
            res_rad += 2.0 * PI;
        }
        return res_rad;
    }

    Transform2D integrate_twist(Twist2D t){
        if(almost_equal(t.m_angular_v,0.0)){
            return Transform2D(Vector2D(t.m_translation_vx,t.m_translation_vy),0.0);
        }
        else{
            
            double dx_dtheta = t.m_translation_vx/t.m_angular_v;
            double dy_dtheta = t.m_translation_vy/t.m_angular_v;
            double x = dx_dtheta * std::sin(t.m_angular_v) + dy_dtheta * (std::cos(t.m_angular_v)-1);
            double y = dx_dtheta * (1-std::cos(t.m_angular_v)) + dy_dtheta * std::sin(t.m_angular_v);
            return Transform2D(Vector2D(x,y),t.m_angular_v);
        }
    }
}
