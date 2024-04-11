

#ifndef ODOMGEOMETRY_H
#define ODOMGEOMETRY_H

#include "mavsdk/plugins/mocap/mocap.h"
#include "OdomGeometry.h"
#include <nav_msgs/msg/detail/odometry__struct.hpp>

struct MocapMessages
{
    mavsdk::Mocap::VisionPositionEstimate vision_position_estimate;
    mavsdk::Mocap::Odometry odometry;
};

MocapMessages RtabOdom2MocapMessage(nav_msgs::msg::Odometry msg, const double& gam);

#endif //ODOMGEOMETRY_H
