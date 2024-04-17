//
// Created by suave on 4/12/24.
//

#include "VIOBridge.h"


TaskResult VIOBridge::start()
{
    m_executor->add_node(this->get_node_base_interface());
    m_executor->spin();
    return TaskResult::SUCCESS;
}

VIOBridge::MocapMessages VIOBridge::RtabOdom2MocapMessage(const OdomMsg::SharedPtr msg, const double& gam)
{
    const auto q = msg->pose.pose.orientation;
    auto yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    auto pitch = asin(-2.0*(q.x*q.z - q.w*q.y));
    auto roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);

    auto initial_pitch = -26*M_PI/180;
    auto xcam = -msg->pose.pose.position.y;
    auto ycam = -msg->pose.pose.position.x;
    auto zcam = -msg->pose.pose.position.z;
    auto xned = xcam*cos(gam) - ycam*sin(gam);
    auto yned = xcam*sin(gam) + ycam*cos(gam);
    auto zned = zcam;
    auto rollbody = yaw;
    auto pitchbody = -pitch-initial_pitch;
    auto yawbody = -roll-M_PI/2+gam;

    while (yawbody > 2*M_PI) yawbody -= 2*M_PI;
    while (yawbody < 0) yawbody += 2*M_PI;

    const auto timestamp = static_cast<uint64_t>(msg->header.stamp.sec * 1e6 + msg->header.stamp.nanosec / 1e3);

    auto covariance = Mocap::Covariance{ {NAN} };
    const auto position_body = Mocap::PositionBody{static_cast<float>(xned), static_cast<float>(yned), static_cast<float>(zned)};
    const auto angle_body = Mocap::AngleBody{static_cast<float>(rollbody), static_cast<float>(pitchbody), static_cast<float>(yawbody)};
    Mocap::VisionPositionEstimate vision_position_estimate {};
    vision_position_estimate.time_usec = timestamp;
    vision_position_estimate.position_body = position_body;
    vision_position_estimate.angle_body = angle_body;
    vision_position_estimate.pose_covariance = covariance;

    auto qx = sin(rollbody/2) * cos(pitchbody/2) * cos(yawbody/2) - cos(rollbody/2) * sin(pitchbody/2) * sin(yawbody/2);
    auto qy = cos(rollbody/2) * sin(pitchbody/2) * cos(yawbody/2) + sin(rollbody/2) * cos(pitchbody/2) * sin(yawbody/2);
    auto qz = cos(rollbody/2) * cos(pitchbody/2) * sin(yawbody/2) - sin(rollbody/2) * sin(pitchbody/2) * cos(yawbody/2);
    auto qw = cos(rollbody/2) * cos(pitchbody/2) * cos(yawbody/2) + sin(rollbody/2) * sin(pitchbody/2) * sin(yawbody/2);
    auto q_odom = Mocap::Quaternion{static_cast<float>(qw), static_cast<float>(qx), static_cast<float>(qy), static_cast<float>(qz)};

    auto speed_body = Mocap::SpeedBody {
        static_cast<float>(msg->twist.twist.linear.x),
        static_cast<float>(-msg->twist.twist.linear.y),
        static_cast<float>(-msg->twist.twist.linear.z)
    };

    auto angular_velocity_body = Mocap::AngularVelocityBody {
        static_cast<float>(msg->twist.twist.angular.x),
        static_cast<float>(-msg->twist.twist.angular.y),
        static_cast<float>(-msg->twist.twist.angular.z)
    };

    Mocap::Odometry odometry {};
    odometry.time_usec = timestamp;
    odometry.frame_id = Mocap::Odometry::MavFrame::MocapNed;
    odometry.position_body = position_body;
    odometry.q = q_odom;
    odometry.speed_body = speed_body;
    odometry.angular_velocity_body = angular_velocity_body;
    odometry.pose_covariance = covariance;
    odometry.velocity_covariance = covariance;

    return { vision_position_estimate, odometry };
}

bool HandleMocapResult(const std::string& name, const Mocap::Result& result)
{
    switch (result)
    {
    case Mocap::Result::Success:
        return true;
    case Mocap::Result::Unknown:
        suave_err << name << ": " << "Unknown error" << std::endl;
        break;
    case Mocap::Result::NoSystem:
        suave_err << name << ": " << "No System error" << std::endl;
        break;
    case Mocap::Result::ConnectionError:
        suave_err << name << ": " << "Connection error" << std::endl;
        break;
    case Mocap::Result::InvalidRequestData:
        suave_err << name << ": " << "Invalid Request Data error" << std::endl;
        break;
    case Mocap::Result::Unsupported:
        suave_err << name << ": " << "Unsupported error" << std::endl;
        break;
    }
    return false;
}

void PrintMocapOdometry(const Mocap::Odometry& odometry) {
    // Convert quaternion to Euler angles
    auto toEuler = [](const Mocap::Quaternion& q) {
        float roll = std::atan2(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y));
        float pitch = std::asin(2.0f * (q.w * q.y - q.z * q.x));
        float yaw = std::atan2(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
        return std::make_tuple(roll, pitch, yaw);
    };

    auto [roll, pitch, yaw] = toEuler(odometry.q);

    // Convert radians to degrees
    auto rad_to_deg = [](float rad) { return rad * 180 / M_PI; };

    std::cout << "Mocap Odometry:\n";

    std::cout << "Position (NED): [" << odometry.position_body.x_m << ", " << odometry.position_body.y_m << ", " << odometry.position_body.z_m << "] m\n";

    std::cout << "Orientation (RPY): [" << rad_to_deg(roll) << ", " << rad_to_deg(pitch) << ", " << rad_to_deg(yaw) << "] deg\n";

    std::cout << "Velocity (Body): [" << odometry.speed_body.x_m_s << ", " << odometry.speed_body.y_m_s << ", " << odometry.speed_body.z_m_s << "] m/s\n";

    std::cout << "Angular Velocity (Body): [" << rad_to_deg(odometry.angular_velocity_body.roll_rad_s) << ", " << rad_to_deg(odometry.angular_velocity_body.pitch_rad_s) << ", " << rad_to_deg(odometry.angular_velocity_body.yaw_rad_s) << "] deg/s\n";
}

void VIOBridge::callback(const OdomMsg::SharedPtr msg)
{
    suave_log << "VIOBridge::callback()" << std::endl;
    const auto [vision_position_estimate, odometry] = RtabOdom2MocapMessage(msg, m_heading_rad);
    const Mocap mocap{*this->get_system()};
    const auto vpe_result = mocap.set_vision_position_estimate(vision_position_estimate);
    const auto odom_result = mocap.set_odometry(odometry);

    PrintMocapOdometry(odometry);

    if (!HandleMocapResult("Mocap::VisionPositionEstimate", vpe_result)
        || !HandleMocapResult("Mocap::Odometry", odom_result))
    {
        suave_err << "Error sending VIO data to Mocap" << std::endl;
    }
}
