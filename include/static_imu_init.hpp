
#ifndef SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H
#define SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H

// GEN
#include<iostream>
#include<deque>

// LOCAL 
// #include<odom.hpp>
// #include<imu.hpp>
#include"imu_preintegration.hpp"

/*
* IMU Initializer in Horizontal Stationary State
* Usage: Call AddIMU and AddOdom to add data, and use InitSuccess to check if initialization was successful.
* After success, use the various Get functions to retrieve internal parameters.
* The initializer attempts to initialize the system each time AddIMU is called. 
    In the case of an Odom, initialization requires the Odom wheel speed reading to be close to zero;
         otherwise, it assumes the vehicle is initially stationary.
* The initializer collects IMU readings over a period of time, 
    estimates the initial zero bias and noise parameters according to Section 3.5.4 of the textbook,
     and provides this information to the ESKF or other filters.
*/

class StaticIMUInit{
    public: 
    struct Options{
        Options() {}
        double init_time_seconds_ = 10.0; 
        int max_imu_queque_max_size_ = 2000; 

        int static_odom_pluse_ = 5;

        double max_static_gyro_var_ = 0.5; 
        double max_static_acce_var_ = 0.05; 

        double gravity_norm_ = 9.81; 
        bool use_speed_for_static_checking_ = true; 
    };

    StaticIMUInit(Options options=Options()) : options_(options) {}

    bool AddIMU(const IMU& imu);
    bool AddOdom(const Odom& odom);
    bool InitSuccess() const{ return init_success_; }

    Eigen::Vector3d GetCovGyro() const {return cov_gyro_; }
    Eigen::Vector3d GetCovAcce() const {return cov_acce_; }
    Eigen::Vector3d GetInitBa()  const {return init_ba_; }
    Eigen::Vector3d GetInitBg()  const {return init_bg_; }
    Eigen::Vector3d GetGravity() const {return gravity_; }

    private: 
    bool TryInit();

    std::deque<IMU> init_imu_deque_; 

    Options options_; 
    bool init_success_ = false; 
    bool is_static_ = false; 
    Eigen::Vector3d cov_gyro_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d cov_acce_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d init_ba_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d init_bg_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();

    double current_time_ = 0.0; 
    double init_start_time_ = 0.0;
    
};

#endif  // SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H