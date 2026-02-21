#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include<iostream>
#include<vector>

#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include<sophus/so3.hpp>
#include<sophus/common.hpp>

#include"odom.hpp"
#include"imu.hpp"
#include"gnss.hpp"
#include"nav_state.hpp"

// DEFINE OPTION STRUCT 
struct PreIntOptions{
    PreIntOptions(){}
    Eigen::Vector3d init_ba_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d init_bg_ = Eigen::Vector3d::Zero();
    double noise_gyro_ = 1e-2; 
    double noise_acce_ = 1e-1; 
};

class IMUPreintegration{

    public:    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IMUPreintegration(PreIntOptions options= PreIntOptions());
    void Integrate(const IMU& imu, double dt);
    Sophus::SO3d GetDeltaRotation(const Eigen::Vector3d & bg); 
    Eigen::Vector3d GetDeltaVelocity(const Eigen::Vector3d & bg, const Eigen::Vector3d& ba); 
    Eigen::Vector3d GetDeltaPosition(const Eigen::Vector3d & bg, const Eigen::Vector3d& ba); 
    double dt_ = 0.0;  // Total preintegration time 

    // Noise 
    Eigen::Matrix<double, 9, 9> cov_ = Eigen::Matrix<double, 9, 9>::Zero(); // Accumlate noise matrix 
    Eigen::Matrix<double, 6, 6> noise_gyro_acce_ = Eigen::Matrix<double, 6, 6>::Zero();  // IMU measurements matrix 
    
    // Bias 
    Eigen::Vector3d ba_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg_ = Eigen::Vector3d::Zero();

    // Pre-Integration State 
    Sophus::SO3d dR_; // rotation 
    Eigen::Vector3d dv_ = Eigen::Vector3d::Zero(); 
    Eigen::Vector3d dp_ = Eigen::Vector3d::Zero(); 

    // Jacobian With Respect to Bias Term 
    Eigen::Matrix<double, 3, 3> dR_dbg_ = Eigen::Matrix<double, 3, 3>::Zero(); 
    Eigen::Matrix<double, 3, 3> dV_dba_ = Eigen::Matrix<double, 3, 3>::Zero(); 
    Eigen::Matrix<double, 3, 3> dV_dbg_ = Eigen::Matrix<double, 3, 3>::Zero(); 
    Eigen::Matrix<double, 3, 3> dP_dbg_ = Eigen::Matrix<double, 3, 3>::Zero(); 
    Eigen::Matrix<double, 3, 3> dP_dba_ = Eigen::Matrix<double, 3, 3>::Zero(); 

    // PREDICT 
    NavStated Predict(const NavStated& start, const Eigen::Vector3d& grav = Eigen::Vector3d(0, 0, -9.81)) const;
    
    //Optional Store IMU msgs 
    std::vector<IMU> imu_msgs_; 
};

#endif