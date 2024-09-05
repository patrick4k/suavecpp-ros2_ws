//
// Created by suave on 4/12/24.
//

#ifndef VIOBRIDGE_H
#define VIOBRIDGE_H
#include <nav_msgs/msg/odometry.hpp>
#include <plugins/mocap/mocap.h>
#include <plugins/telemetry/telemetry.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using namespace mavsdk;
using OdomMsg = nav_msgs::msg::Odometry;

class VIOBridge: public rclcpp::Node {
public:
    explicit VIOBridge(std::shared_ptr<System> system, const double& initial_heading_rad) :
        Node("suave_vio_bridge"),
        m_system(std::move(system)),
        m_heading_rad(initial_heading_rad)
    {
        m_subscription = this->create_subscription<OdomMsg>("/odom", 10, std::bind(&VIOBridge::callback, this, std::placeholders::_1));
    }

private:
    std::shared_ptr<System> m_system;
    double m_heading_rad{NAN};

    using Subscription = rclcpp::Subscription<OdomMsg>;
    std::optional<Subscription::SharedPtr> m_subscription{};

    struct MocapMessages
    {
        Mocap::VisionPositionEstimate vision_position_estimate;
        Mocap::Odometry odometry;
    };

    static MocapMessages RtabOdom2MocapMessage(const OdomMsg::SharedPtr msg, const double& gam);

    void callback(const OdomMsg::SharedPtr msg);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Print out the received odometry message
        RCLCPP_INFO(this->get_logger(), "Position: [%f, %f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Orientation: [%f, %f, %f, %f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "Linear Velocity: [%f, %f, %f]", msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        RCLCPP_INFO(this->get_logger(), "Angular Velocity: [%f, %f, %f]", msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    }

};

#endif //VIOBRIDGE_H
