#include "turtlelib/rigid2d.hpp"
#include <iostream>
using namespace turtlelib;
using namespace std;

int main(){
    cout<<"Enter transform T_{a,b}:"<<endl;
    Transform2D T_ab;
    cin>>T_ab;
    cout<<"Enter transform T_{b,c}:"<<endl;
    Transform2D T_bc;
    cin>>T_bc;
    cout<<"T_{a,b}: "<<T_ab;
    Transform2D T_ba = T_ab.inv();
    cout<<"T_{b,a}: "<<T_ba;
    cout<<"T_{b,c}: "<<T_bc;
    Transform2D T_cb = T_bc.inv();
    cout<<"T_{c,b}: "<<T_cb;
    Transform2D T_ac = T_ab;
    T_ac*=T_bc;
    cout<<"T_{a,c}: "<<T_ac;
    Transform2D T_ca = T_ac.inv();
    cout<<"T_{c,a}: "<<T_ca;
    cout<<"Enter vector v_b:"<<endl;
    Vector2D v_b;
    
    cin>>v_b;
    Vector2D v_bhat = normalize_vec(v_b);
    cout<<"v_bhat: "<<v_bhat;
    Vector2D v_a,v_c;
    v_a = T_ab(v_b);
    v_c = T_cb(v_b);
    cout<<"v_a: "<<v_a;
    cout<<"v_b: "<<v_b;
    cout<<"v_c: "<<v_c;
    cout<<"Enter twist V_b:"<<endl;
    Twist2D V_b;
    cin>>V_b;
    Twist2D V_a,V_c;
    V_a = T_ab(V_b);
    V_c = T_cb(V_b);
    cout<<"V_a "<<V_a;
    cout<<"V_b "<<V_b;
    cout<<"V_c "<<V_c;
    
    return 0;
}
