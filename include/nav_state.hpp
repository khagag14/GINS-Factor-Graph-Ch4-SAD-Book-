#ifndef NAV_STATE_H
#define NAV_STATE_H

#include<memory>
#include<eigen3/Eigen/Core>
#include<sophus/so3.hpp>
#include<sophus/se3.hpp>


struct NavStated
{

    NavStated() = default;

    explicit NavStated(double time, const Sophus::SO3d& R = Sophus::SO3d(), 
                        const Eigen::Vector3d& t = Eigen::Vector3d::Zero(), 
                        const Eigen::Vector3d& v = Eigen::Vector3d::Zero(),
                        const Eigen::Vector3d& bg = Eigen::Vector3d::Zero(),
                        const Eigen::Vector3d& ba = Eigen::Vector3d::Zero()) 
                        :  timestamp_(time), R_(R), p_(t), v_(v), bg_(bg), ba_(ba){}

    NavStated(double time, const Sophus::SE3d& pose, const Eigen::Vector3d& vel = Eigen::Vector3d::Zero()) 
            : timestamp_(time), p_(pose.translation()), R_(pose.so3()), v_(vel) {}
    
    friend std::ostream& operator << (std::ostream& os, const NavStated& s){
        os << "time : " << s.timestamp_ << " ,p: " << s.p_.transpose() << " ,v: " << s.v_.transpose() << 
        " ,q: " <<  s.R_.unit_quaternion().coeffs().transpose() << 
        " , bg: " << s.bg_.transpose() << " , ba: " << s.ba_.transpose(); 

        return os; 

    }
    
    Sophus::SE3d GetSE3 () const { return Sophus::SE3<double>(R_, p_);}

    double timestamp_ = 0.0;
    Sophus::SO3d R_;
    Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba_ = Eigen::Vector3d::Zero();

};

#endif