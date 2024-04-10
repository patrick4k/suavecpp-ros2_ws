

#ifndef ODOMGEOMETRY_H
#define ODOMGEOMETRY_H

#include <optional>

#include "mavsdk/plugins/mocap/mocap.h"
#include "OdomGeometry.h"
#include <nav_msgs/msg/detail/odometry__struct.hpp>

struct MocapMessages
{
    mavsdk::Mocap::VisionPositionEstimate vision_position_estimate;
    mavsdk::Mocap::Odometry odometry;
};

void SetInitialHeading(double heading);

MocapMessages RtabOdom2MocapMessage(nav_msgs::msg::Odometry msg);


#endif //ODOMGEOMETRY_H
