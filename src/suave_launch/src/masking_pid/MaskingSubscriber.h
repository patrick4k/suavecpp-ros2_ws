//
// Created by patrick on 9/5/24.
//

#ifndef MASKINGSUBSCRIBER_H
#define MASKINGSUBSCRIBER_H

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>

#include "../mavutil/Drone.h"
#include "../pid/YawPID.h"
#include "../vio/VIOBridge.h"
#include <optional>

using Vector3Msg = geometry_msgs::msg::Vector3;
using QuaternionMsg = geometry_msgs::msg::Quaternion;

class MaskingSubscriber final : public rclcpp::Node {
public:
    explicit MaskingSubscriber(Drone* drone) : Node("suave_masking_subscriber"), 
        m_drone{ drone }
    {
        // m_subscription = this->create_subscription<Vector3Msg>(
        m_subscription = this->create_subscription<QuaternionMsg>(
            "/masking_pid_publisher", 10, std::bind(&MaskingSubscriber::callback, this, std::placeholders::_1));
        m_drone->set_heading_callback(std::bind(&MaskingSubscriber::heading_callback, this, std::placeholders::_1));
    }

    void enable();
    void disable();

private:

    // void callback(const Vector3Msg::SharedPtr msg);
    void callback(const QuaternionMsg::SharedPtr msg);
    void shutdown();

    void heading_callback(mavsdk::Telemetry::Heading heading);

    // using Subscription = rclcpp::Subscription<Vector3Msg>;
    using Subscription = rclcpp::Subscription<QuaternionMsg>;
    Subscription::SharedPtr m_subscription{ nullptr };

    using Velocity = Offboard::VelocityBodyYawspeed;

    Drone* m_drone{ nullptr };
    std::atomic_bool m_enable{ false };
    std::atomic_bool m_end_controller{ false };
    std::optional<Velocity> m_prevVelocity;
};

#endif //MASKINGSUBSCRIBER_H
