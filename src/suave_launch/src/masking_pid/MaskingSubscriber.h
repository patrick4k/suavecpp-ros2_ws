//
// Created by patrick on 9/5/24.
//

#ifndef MASKINGSUBSCRIBER_H
#define MASKINGSUBSCRIBER_H

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>

using Vector3Msg = geometry_msgs::msg::Vector3;

class MaskingSubscriber final : public rclcpp::Node {
public:
    explicit MaskingSubscriber() : Node("suave_masking_subscriber") {
        m_subscription = this->create_subscription<Vector3Msg>(
            "/masking_pid_publisher", 10, std::bind(&MaskingSubscriber::callback, this, std::placeholders::_1));
    }

private:

    void callback(const Vector3Msg::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received x: %f, y: %f, z: %f", msg->x, msg->y, msg->z);
    }

    using Subscription = rclcpp::Subscription<Vector3Msg>;
    Subscription::SharedPtr m_subscription{ nullptr };
};

#endif //MASKINGSUBSCRIBER_H
