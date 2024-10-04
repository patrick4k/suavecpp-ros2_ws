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

using Vector3Msg = geometry_msgs::msg::Vector3;

class MaskingSubscriber final : public rclcpp::Node {
public:
    explicit MaskingSubscriber(Drone* drone) : Node("suave_masking_subscriber"), m_drone{ drone } {
        m_subscription = this->create_subscription<Vector3Msg>(
            "/masking_pid_publisher", 10, std::bind(&MaskingSubscriber::callback, this, std::placeholders::_1));
        m_drone->set_heading_callback(std::bind(&MaskingSubscriber::heading_callback, this, std::placeholders::_1));
    }

    void set_enable(bool enable);

private:

    void callback(const Vector3Msg::SharedPtr msg);
    void shutdown();

    void heading_callback(mavsdk::Telemetry::Heading heading);

    using Subscription = rclcpp::Subscription<Vector3Msg>;
    Subscription::SharedPtr m_subscription{ nullptr };

    using Velocity = Offboard::VelocityBodyYawspeed;

    Drone* m_drone{ nullptr };
    std::atomic_bool m_enable{ false };
    std::atomic_bool m_end_controller{ false };
    std::optional<Velocity> m_prevVelocity;

    YawPID m_headingPid{ 1.0, 0.0, 0.0, 180 / 3.14 * m_drone->initial_heading_rad() }; // TODO: tune me
    std::atomic<double> m_currHeadingPidValue{};
};

#endif //MASKINGSUBSCRIBER_H
