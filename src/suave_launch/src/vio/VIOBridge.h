//
// Created by suave on 4/12/24.
//

#ifndef VIOBRIDGE_H
#define VIOBRIDGE_H
#include <nav_msgs/msg/odometry.hpp>
#include <plugins/mocap/mocap.h>
#include <plugins/telemetry/telemetry.h>

#include "../ros/RosSubscriber.h"

using namespace mavsdk;
using OdomMsg = nav_msgs::msg::Odometry;

class VIOBridge: public IMavController, public rclcpp::Node {
public:
    explicit VIOBridge(std::shared_ptr<System> system) :
    IMavController(std::move(system)),
    Node("suave_vio_bridge")
    {
        const Telemetry telem{*this->get_system()};
        m_heading_rad = telem.heading().heading_deg * M_PI / 180.0;

        m_subscription = this->create_subscription<OdomMsg>("/odom", 10, std::bind(&VIOBridge::odomCallback, this, std::placeholders::_1));
    }

    MavControllerResult start() override;

private:
    double m_heading_rad;
    using Subscription = rclcpp::Subscription<OdomMsg>;
    std::optional<Subscription::SharedPtr> m_subscription{};
    using Executor = rclcpp::executors::SingleThreadedExecutor;
    std::shared_ptr<Executor> m_executor = std::make_shared<Executor>();

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
        RCLCPP_INFO(this->get_logger(), "Received odom: [%f, %f, %f]",
                     msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }

};

#endif //VIOBRIDGE_H
