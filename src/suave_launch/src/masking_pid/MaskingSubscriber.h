//
// Created by patrick on 9/5/24.
//

#ifndef MASKINGSUBSCRIBER_H
#define MASKINGSUBSCRIBER_H

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>

#include "../mavutil/Drone.h"
#include "../pid/YawPID.h"
#include "../vio/VIOBridge.h"
#include <optional>

using Vector3Msg = geometry_msgs::msg::Vector3;

class MaskingSubscriber final : public rclcpp::Node {
public:
    explicit MaskingSubscriber(Drone* drone, VIOBridge* vio_bridge) : Node("suave_masking_subscriber"), 
        m_drone{ drone }, 
        m_vio_bridge{ vio_bridge } 
    {
        m_subscription = this->create_subscription<Vector3Msg>(
            "/masking_pid_publisher", 10, std::bind(&MaskingSubscriber::callback, this, std::placeholders::_1));
        m_drone->set_heading_callback(std::bind(&MaskingSubscriber::heading_callback, this, std::placeholders::_1));
    }

    void enable(double yawsetpoint_deg);
    void disable();
    void export_yaw()
    {
        if (m_headingPid)
        {
            m_headingPid->export_csv();
        }
    }

private:

    void callback(const Vector3Msg::SharedPtr msg);
    void shutdown();

    void heading_callback(mavsdk::Telemetry::Heading heading);

    using Subscription = rclcpp::Subscription<Vector3Msg>;
    Subscription::SharedPtr m_subscription{ nullptr };

    using Velocity = Offboard::VelocityBodyYawspeed;
    using VelocityNed = Offboard::VelocityNedYaw;

    Drone* m_drone{ nullptr };
    VIOBridge* m_vio_bridge{ nullptr };
    std::atomic_bool m_enable{ false };
    std::atomic_bool m_end_controller{ false };
    std::optional<VelocityNed> m_prevVelocity;

    std::optional<YawPID> m_headingPid{};
    std::atomic<double> m_currHeadingPidValue{};
};

#endif //MASKINGSUBSCRIBER_H
