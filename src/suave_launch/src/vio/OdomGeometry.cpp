#include "OdomGeometry.h"

#include <iostream>
#include <math.h>

#include "../util/Util.h"

using namespace mavsdk;

MocapMessages RtabOdom2MocapMessage(nav_msgs::msg::Odometry msg, const double& gam)
{
    const auto q = msg.pose.pose.orientation;
    auto yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    auto pitch = asin(-2.0*(q.x*q.z - q.w*q.y));
    auto roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);

    auto initial_pitch = -26*M_PI/180;
    auto xcam = -msg.pose.pose.position.y;
    auto ycam = -msg.pose.pose.position.x;
    auto zcam = -msg.pose.pose.position.z;
    auto xned = xcam*cos(gam) - ycam*sin(gam);
    auto yned = xcam*sin(gam) + ycam*cos(gam);
    auto zned = zcam;
    auto rollbody = yaw;
    auto pitchbody = -pitch-initial_pitch;
    auto yawbody = -roll-M_PI/2+gam;

    while (yawbody > 2*M_PI) yawbody -= 2*M_PI;
    while (yawbody < 0) yawbody += 2*M_PI;

    const auto timestamp = static_cast<uint64_t>(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3);

    const auto nan_covariance = std::vector { NAN };
    const auto position_body = Mocap::PositionBody{static_cast<float>(xned), static_cast<float>(yned), static_cast<float>(zned)};
    const auto angle_body = Mocap::AngleBody{static_cast<float>(rollbody), static_cast<float>(pitchbody), static_cast<float>(yawbody)};
    const Mocap::VisionPositionEstimate vision_position_estimate {
        .time_usec = timestamp,
        .position_body = position_body,
        .angle_body = angle_body,
        .pose_covariance = nan_covariance
    };

    auto qx = sin(rollbody/2) * cos(pitchbody/2) * cos(yawbody/2) - cos(rollbody/2) * sin(pitchbody/2) * sin(yawbody/2);
    auto qy = cos(rollbody/2) * sin(pitchbody/2) * cos(yawbody/2) + sin(rollbody/2) * cos(pitchbody/2) * sin(yawbody/2);
    auto qz = cos(rollbody/2) * cos(pitchbody/2) * sin(yawbody/2) - sin(rollbody/2) * sin(pitchbody/2) * cos(yawbody/2);
    auto qw = cos(rollbody/2) * cos(pitchbody/2) * cos(yawbody/2) + sin(rollbody/2) * sin(pitchbody/2) * sin(yawbody/2);
    auto q_odom = Mocap::Quaternion{static_cast<float>(qw), static_cast<float>(qx), static_cast<float>(qy), static_cast<float>(qz)};

    auto speed_body = Mocap::SpeedBody {
        static_cast<float>(msg.twist.twist.linear.x),
        static_cast<float>(-msg.twist.twist.linear.y),
        static_cast<float>(-msg.twist.twist.linear.z)
    };

    auto angular_velocity_body = Mocap::AngularVelocityBody {
        static_cast<float>(msg.twist.twist.angular.x),
        static_cast<float>(-msg.twist.twist.angular.y),
        static_cast<float>(-msg.twist.twist.angular.z)
    };

    const Mocap::Odometry odometry {
        .time_usec = timestamp,
        .frame_id = Mocap::Odometry::MavFrame::MocapNed,
        .position_body = position_body,
        .q = q_odom,
        .speed_body = speed_body,
        .angular_velocity_body = angular_velocity_body,
        .pose_covariance = nan_covariance,
        .velocity_covariance = nan_covariance
    };

    return {
        .vision_position_estimate = vision_position_estimate,
        .odometry = odometry
    };
}
