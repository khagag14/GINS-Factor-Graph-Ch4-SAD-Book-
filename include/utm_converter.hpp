#ifndef SLAM_IN_AUTO_DRIVING_UTM_CONVERT_H
#define SLAM_IN_AUTO_DRIVING_UTM_CONVERT_H

#include"gnss.hpp"
#include"utm_convert/utm.h"

bool ConvertGps2UTM(GNSS& gnss_reading, const Eigen::Vector2d& antenna_pos, const double& antenna_angle,
                    const Eigen::Vector3d& map_origin = Eigen::Vector3d::Zero());

bool ConvertGps2UTMOnlyTrans(GNSS& gnss_reading);

bool LatLon2UTM(const Eigen::Vector2d& latlon, UTMCoordinate& utm_coor);

bool UTM2LatLon(const UTMCoordinate& utm_coor, Eigen::Vector2d& latlon);

#endif //SLAM_IN_AUTO_DRIVING_UTM_CONVERT_H