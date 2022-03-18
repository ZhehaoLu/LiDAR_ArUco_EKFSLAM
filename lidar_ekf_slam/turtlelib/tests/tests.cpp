#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "catch_ros/catch.hpp"
#include <iostream>
#include <sstream>
using namespace turtlelib;
using namespace std;


TEST_CASE("TEST: almost_equal()","[almost_equal]")
{
    REQUIRE(almost_equal(deg2rad(45.0),0.785375,1.0e-2)==true);
}

TEST_CASE("TEST: rad2deg()","[rad2deg]")
{
    REQUIRE(almost_equal(rad2deg(1.0),57.29578,1.0e-2)==true);
}

TEST_CASE("TEST: normalize_vec()","[normalize_vec]")
{
    Vector2D v(2.0,2.0);
    Vector2D res(1.0/sqrt(2),1.0/sqrt(2));
    REQUIRE(almost_equal(normalize_vec(v).x,res.x,10e-2)==true);
    REQUIRE(almost_equal(normalize_vec(v).y,res.y,10e-2)==true);

    Approx target_x = Approx(res.x).epsilon(10e-2);
    Approx target_y = Approx(res.y).epsilon(10e-2);
    REQUIRE(normalize_vec(v).x==target_x);
    REQUIRE(normalize_vec(v).y==target_y);
}

TEST_CASE("TEST: std::istream &operator>>(std::istream &is, const Vector2D &v)","[Vector2D::operator>>]")
{
    Vector2D v;
    string str="1.0 1.0";
    stringstream ss(str);
    ss>>v;

    Vector2D res(1.0,1.0);
    Approx target_x = Approx(res.x).epsilon(10e-2);
    Approx target_y = Approx(res.y).epsilon(10e-2);
    REQUIRE(v.x==target_x);
    REQUIRE(v.y==target_y);
}

TEST_CASE("TEST: std::ostream &operator<<(std::ostream &os, const Vector2D &v)","[Vector2D::operator<<]")
{
    Vector2D v(1.0,1.0);
    double target_x=1.0,target_y=1.0;
    stringstream ss;
    ss<<v;
    string str1,str2;
    ss>>str1;
    ss>>str2;
    if(*str1.begin()=='['){
        str1=str1.substr(1,str1.size()-1);
        str2=str2.substr(0,str2.size()-1);
    }
    stringstream s1(str1),s2(str2);
    double res1,res2;
    s1>>res1;
    s2>>res2;
    REQUIRE(res1==target_x);
    REQUIRE(res2==target_y);
}

TEST_CASE("TEST: std::istream& operator>>(std::istream& is, Twist2D& t)","[Twist2D::operator>>]")
{
    Twist2D t;
    string str="1.0 2.0 3.0";
    stringstream ss(str);
    ss>>t;

    Twist2D res(1.0,2.0,3.0);
    Approx target_v = Approx(res.m_angular_v).epsilon(10e-2);
    Approx target_vx = Approx(res.m_translation_vx).epsilon(10e-2);
    Approx target_vy = Approx(res.m_translation_vy).epsilon(10e-2);

    REQUIRE(t.m_angular_v==target_v);
    REQUIRE(t.m_translation_vx==target_vx);
    REQUIRE(t.m_translation_vy==target_vy);
}

TEST_CASE("TEST: std::ostream& operator<<(std::ostream& os, const Twist2D& t)","[Twist2D::operator<<]")
{
    Twist2D t(1.0,2.0,3.0);
    double target_v=1.0,target_vx=2.0,target_vy=3.0;
    stringstream ss;
    ss<<t;
    string str1,str2,str3;
    ss>>str1;
    ss>>str2;
    ss>>str3;
    if(*str1.begin()=='['){
        str1=str1.substr(1,str1.size()-1);
        str3=str3.substr(0,str3.size()-1);
    }
    stringstream s1(str1),s2(str2),s3(str3);
    double res1,res2,res3;
    s1>>res1;
    s2>>res2;
    s3>>res3;
    REQUIRE(res1==target_v);
    REQUIRE(res2==target_vx);
    REQUIRE(res3==target_vy);
}

TEST_CASE("TEST: std::istream &operator>>(std::istream &is, Transform2D &tf)","[Transform2D::operator>>]")
{
    Transform2D tf;
    string str="deg: 90 x: 3 y: 5";
    stringstream ss(str);
    ss>>tf;
    Transform2D tf_res(Vector2D(3.0,5.0),deg2rad(90.0));
    REQUIRE(tf_res.translation().x==tf.translation().x);
    REQUIRE(tf_res.translation().y==tf.translation().y);
    REQUIRE(tf_res.rotation()==tf.rotation());
}

TEST_CASE("TEST: //std::ostream &operator<<(std::ostream &os, const Transform2D &tf)","[Transform2D::operator<<]")
{
    Transform2D tf(Vector2D(3.0,5.0),deg2rad(90.0));
    double target_angle =90,target_x=3,target_y=5;
    stringstream ss;
    ss<<tf;
    vector<string> str(6);
    for(int i=0;i<6;i++) ss>>str[i];
    stringstream s1(str[1]),s2(str[3]),s3(str[5]);
    double res1,res2,res3;
    s1>>res1;
    s2>>res2;
    s3>>res3;
    REQUIRE(res1==target_angle);
    REQUIRE(res2==target_x);
    REQUIRE(res3==target_y);
}

TEST_CASE("TEST: Vector2D operator()(Vector2D v) const","[Transform2D::operator()]")
{
    Transform2D tf(Vector2D(0.0,1.0),0.5*PI);
    Vector2D vb(1,1);
    Vector2D va(-1,2);
    Vector2D vres = tf(vb);
    Approx target_x = Approx(vres.x).epsilon(10e-2);
    Approx target_y = Approx(vres.y).epsilon(10e-2);
    REQUIRE(va.x==target_x);
    REQUIRE(va.y==target_y);

}

TEST_CASE("TEST: Twist2D operator()(Twist2D v) const","[Twist2D::operator()]")
{
    Transform2D tf(Vector2D(0.0,1.0),0.5*PI);
    Twist2D tb(1,1,1);
    Twist2D ta(1,0,1);
    Twist2D tres = tf(tb);
    Approx target_a = Approx(tres.m_angular_v).epsilon(10e-2);
    Approx target_b = Approx(tres.m_translation_vx).epsilon(10e-2);
    Approx target_c = Approx(tres.m_translation_vy).epsilon(10e-2);
    REQUIRE(ta.m_angular_v==target_a);
    REQUIRE(ta.m_translation_vx==target_b);
    REQUIRE(ta.m_translation_vy==target_c);

}


TEST_CASE("TEST: Transform2D inv() const","[Transform2D::inv()]")
{
    Transform2D tf1(Vector2D(0.0,1.0),0.5*PI);
    Transform2D tf2(Vector2D(-1.0,-6.12323e-17),-0.5*PI);
    Transform2D tfres = tf1.inv();
    Approx target_a = Approx(tfres.translation().x).epsilon(10e-2);
    Approx target_b = Approx(tfres.translation().y).epsilon(10e-2);
    Approx target_c = Approx(tfres.rotation()).epsilon(10e-2);
    REQUIRE(tf2.translation().x==target_a);
    REQUIRE(tf2.translation().y==target_b);
    REQUIRE(tf2.rotation()==target_c);

}

TEST_CASE("TEST: Transform2D &operator*=(const Transform2D &rhs)","[Transform2D::&operator*=]")
{
    Transform2D tf1(Vector2D(0.0,1.0),0.5*PI);
    Transform2D tf2(Vector2D(1.0,0.0),0.5*PI);
    tf1*=tf2;
    Transform2D tf3(Vector2D(6.12323e-17,2),PI);
    Approx target = Approx(tf3.rotation()).epsilon(10e-2);
    REQUIRE(tf1.rotation()==target);
}

TEST_CASE("TEST: Transform2D operator*(Transform2D lhs, const Transform2D &rhs)","[Transform2D::&operator*]")
{
    Transform2D tf1(Vector2D(0.0,1.0),2.5*PI);
    Transform2D tf2(Vector2D(1.0,0.0),2.5*PI);
    Transform2D tf_res = tf1*tf2;
    Transform2D tf3(Vector2D(6.12323e-17,2),PI);
    Approx target_c = Approx(tf3.rotation()).epsilon(10e-1);
    REQUIRE(tf_res.rotation()==target_c);
}


TEST_CASE("TEST: double normalize_angle(double rad)","[normalize_angle()]")
{
    double rad = 1.0 * PI;
    double res_rad = normalize_angle(rad);
    Approx target = Approx(1.0*PI).epsilon(10e-2);
    REQUIRE(res_rad==target);

    rad = -1.0 * PI;
    res_rad = normalize_angle(rad);
    target = Approx(1.0*PI).epsilon(10e-2);
    REQUIRE(res_rad==target);

    rad = 0.0 * PI;
    res_rad = normalize_angle(rad);
    target = Approx(0.0*PI).epsilon(10e-2);
    REQUIRE(res_rad==target);

    rad = -0.25 * PI;
    res_rad = normalize_angle(rad);
    target = Approx(-0.25*PI).epsilon(10e-2);
    REQUIRE(res_rad==target);

    rad = 1.5 * PI;
    res_rad = normalize_angle(rad);
    target = Approx(-0.5*PI).epsilon(10e-2);
    REQUIRE(res_rad==target);

    rad = -2.5 * PI;
    res_rad = normalize_angle(rad);
    target = Approx(-0.5*PI).epsilon(10e-2);
    REQUIRE(res_rad==target);

}

TEST_CASE("TEST: Vector2D& Vector2D::operator+=(const Vector2D &rhs)","[Vector2D::operator+=()]"){
    Vector2D v1(1,2);
    Vector2D v2(3,4);
    Vector2D v_res(4,6);
    v1+=v2;
    REQUIRE(v1.x==v_res.x);
    REQUIRE(v1.y==v_res.y);
}

TEST_CASE("TEST: Vector2D operator+(Vector2D lhs, const Vector2D &rhs)","[Vector2D::operator+()]"){
    Vector2D v1(1,2);
    Vector2D v2(3,4);
    Vector2D v_res(4,6);
    Vector2D v3=v1+v2;
    REQUIRE(v3.x==v_res.x);
    REQUIRE(v3.y==v_res.y);
}

TEST_CASE("TEST: Vector2D& Vector2D::operator-=(const Vector2D &rhs)","[Vector2D::operator-=()]"){
    Vector2D v1(1,2);
    Vector2D v2(3,4);
    Vector2D v_res(-2,-2);
    v1-=v2;
    REQUIRE(v1.x==v_res.x);
    REQUIRE(v1.y==v_res.y);
}

TEST_CASE("TEST: Vector2D operator-(Vector2D lhs, const Vector2D &rhs)","[Vector2D::operator-()]"){
    Vector2D v1(1,2);
    Vector2D v2(3,4);
    Vector2D v_res(-2,-2);
    Vector2D v3=v1-v2;
    REQUIRE(v3.x==v_res.x);
    REQUIRE(v3.y==v_res.y);
}

TEST_CASE("TEST: Vector2D &Vector2D::operator*=(const double &rhs)","[Vector2D::operator*=()]"){
    Vector2D v1(1,2);
    Vector2D v_res(1.5,3);
    double rhs = 1.5;
    v1*=rhs;
    REQUIRE(v1.x==v_res.x);
    REQUIRE(v1.y==v_res.y);
}

TEST_CASE("TEST: Vector2D operator*(Vector2D lhs, const double& rhs)","[Vector2D operator*()]"){
    Vector2D v1(1,2);
    Vector2D v_res(1.5,3);
    double rhs = 1.5;
    Vector2D v2=v1 * rhs;
    REQUIRE(v2.x==v_res.x);
    REQUIRE(v2.y==v_res.y);
}

TEST_CASE("TEST: Vector2D operator*(const double& rhs,Vector2D lhs)","[Vector2D operator*()]"){
    Vector2D v1(1,2);
    Vector2D v_res(1.5,3);
    double rhs = 1.5;
    Vector2D v2=rhs * v1;
    REQUIRE(v2.x==v_res.x);
    REQUIRE(v2.y==v_res.y);
}

TEST_CASE("TEST: double angle(const Vector2D &v1, const Vector2D &v2)","[Vector2D::angle()]"){
    Vector2D v1(1.732,1);
    Vector2D v2(3,3);
    double res_angle=angle(v1,v2);
    Approx target = Approx(15).epsilon(10e-2);
    REQUIRE(res_angle==target);

    v1=Vector2D(1.732,-1);
    v2=Vector2D(3,3);
    res_angle=angle(v1,v2);
    target = Approx(75).epsilon(10e-2);
    REQUIRE(res_angle==target);

    v1=Vector2D(1.732,-1);
    v2=Vector2D(-3,3);
    res_angle=angle(v1,v2);
    target = Approx(165).epsilon(10e-2);
    REQUIRE(res_angle==target);

    v1=Vector2D(-1.732,-1);
    v2=Vector2D(-3,-3);
    res_angle=angle(v2,v1);
    target = Approx(-15).epsilon(10e-2);
    REQUIRE(res_angle==target);
}

TEST_CASE("TEST: Transform2D integrate_twist(Twist2D t)","[integrate_twist()]"){

    Twist2D t(0,1,1);
    Transform2D tf = integrate_twist(t);
    REQUIRE(tf.rotation()==0.0);
    REQUIRE(tf.translation().x==1.0);
    REQUIRE(tf.translation().y==1.0);

    t = Twist2D(0.25*PI,0,0);
    tf = Transform2D(integrate_twist(t));
    REQUIRE(tf.rotation()==0.25*PI);
    REQUIRE(tf.translation().x==0.0);
    REQUIRE(tf.translation().y==0.0);

    t = Twist2D(0.25*PI,1,1);
    tf = Transform2D(integrate_twist(t));
    REQUIRE(tf.rotation()==0.25*PI);
    Approx target1 = Approx(0.5276).epsilon(10e-2);
    Approx target2 = Approx(1.2739).epsilon(10e-2);
    REQUIRE(tf.translation().x==target1);
    REQUIRE(tf.translation().y==target2);

}

TEST_CASE("TEST: WheelVel DiffDrive::IK_Calculate(Twist2D t)","[IK_Calculate()]"){

    Twist2D t(1,0,0);
    DiffDrive drive;
    WheelVel w_vel=drive.IK_Calculate(t);
    Approx target1 = Approx(-12.12).epsilon(10e-2);
    Approx target2 = Approx(12.12).epsilon(10e-2);
    REQUIRE(w_vel.m_ul==target1);
    REQUIRE(w_vel.m_ur==target2);

    t=Twist2D(0,1,0);
    w_vel=drive.IK_Calculate(t);
    target1 = Approx(30.30).epsilon(10e-2);
    target2 = Approx(30.30).epsilon(10e-2);
    REQUIRE(w_vel.m_ul==target1);
    REQUIRE(w_vel.m_ur==target2);

    t=Twist2D(1,1,0);
    w_vel=drive.IK_Calculate(t);
    target1 = Approx(18.18).epsilon(10e-2);
    target2 = Approx(42.42).epsilon(10e-2);
    REQUIRE(w_vel.m_ul==target1);
    REQUIRE(w_vel.m_ur==target2);

}

TEST_CASE("TEST: Twist2D DiffDrive::wheelvelTotwist(WheelVel v)","[wheelvelTotwist()]"){

    WheelVel w_vel(-12.12,12.12);
    DiffDrive drive;
    Twist2D t=drive.wheelvelTotwist(w_vel);
    Approx target1 = Approx(1.0).epsilon(10e-2);
    Approx target2 = Approx(0.0).epsilon(10e-2);
    REQUIRE(t.m_angular_v==target1);
    REQUIRE(t.m_translation_vx==target2);
    REQUIRE(t.m_translation_vy==0.0);

    w_vel=WheelVel(33.33,33.33);
    t=drive.wheelvelTotwist(w_vel);
    target1 = Approx(0.0).epsilon(10e-2);
    target2 = Approx(1.0).epsilon(10e-2);
    REQUIRE(t.m_angular_v==target1);
    REQUIRE(t.m_translation_vx==target2);
    REQUIRE(t.m_translation_vy==0.0);

    w_vel=WheelVel(18.18,42.42);
    t=drive.wheelvelTotwist(w_vel);
    target1 = Approx(1.0).epsilon(10e-2);
    target2 = Approx(1.0).epsilon(10e-2);
    REQUIRE(t.m_angular_v==target1);
    REQUIRE(t.m_translation_vx==target2);
    REQUIRE(t.m_translation_vy==0.0);

}

TEST_CASE("TEST: WheelVel FK_Calculate(double leftwheel_cur, double rightwheel_cur)","[FK_Calculate()]"){

    turtlelib::DiffDrive diff_drive;
    diff_drive.FK_Calculate(1.0,1.0);
    Transform2D tf = diff_drive.getpose();
    Approx target = Approx(0.033).epsilon(10e-2);
    REQUIRE(tf.translation().x==target);
    REQUIRE(tf.translation().y==0.0);
    REQUIRE(tf.rotation()==0.0);
}

TEST_CASE("TEST: double dot(Vector2D v1, Vector2D v2)","[Vector2D::dot()]"){

    turtlelib::Vector2D v1 (1,2);
    turtlelib::Vector2D v2 (2,1);

    double res = v1.dot(v2);
    REQUIRE(res ==4.0);

}









