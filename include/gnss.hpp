#ifndef SLAM_IN_AUTO_DRIVING_GNSS_H
#define SLAM_IN_AUTO_DRIVING_GNSS_H

#include<memory>

#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include<sophus/se3.hpp>

enum class GpsStatusType{
    GNSS_FLOAT_SOLUTION = 5,
    GNSS_FIXED_SOLUTION = 4,
    GNSS_PSEUDO_SOLUTION = 2,
    GNSS_SINGLE_POINT_SOLUTION = 1,
    GNSS_NOT_EXIST = 0, 
    GNSS_OTHER = -1, 
};


struct UTMCoordinate
{
    UTMCoordinate() = default;
    UTMCoordinate( int zone, const Eigen::Vector2d& xy = Eigen::Vector2d::Zero(), bool north = true) 
    : zone_(zone), xy_(xy), north_(north) {}

    int zone_ = 0;
    Eigen::Vector2d xy_ = Eigen::Vector2d::Zero();
    double z_ = 0;
    bool north_;

};


struct GNSS
{
    GNSS() = default;
    GNSS(double unix_time, int status, const Eigen::Vector3d& lat_lon_alt, double heading, bool heading_valid) 
    : unix_time_(unix_time), lat_lon_alt_(lat_lon_alt), heading_(heading),  heading_valid_(heading_valid) {}

    // GNSS(sensor_msgs::NavSatFix::Ptr msg) {
    //     unix_time_ = msg->header.stamp.toSec();
    //     // 状态位
    //     if (int(msg->status.status) >= int(sensor_msgs::NavSatStatus::STATUS_FIX)) {
    //         status_ = GpsStatusType::GNSS_FIXED_SOLUTION;
    //     } else {
    //         status_ = GpsStatusType::GNSS_OTHER;
    //     }
    //     // 经纬度
    //     lat_lon_alt_ << msg->latitude, msg->longitude, msg->altitude;
    // }

    double unix_time_ = 0;
    GpsStatusType status_ = GpsStatusType::GNSS_NOT_EXIST;
    Eigen::Vector3d lat_lon_alt_ = Eigen::Vector3d::Zero();
    double heading_ = 0.0;
    bool heading_valid_ = false;

    UTMCoordinate utm;
    bool utm_valid = false;
    Sophus::SE3d utm_pose_;

};


using GNSSPtr = std::shared_ptr<GNSS>;

#endif