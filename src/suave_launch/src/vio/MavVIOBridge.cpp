#include "MavVIOBridge.h"

#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <plugins/telemetry/telemetry.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>


MavControllerResult MavVIOBridge::start()
{
    mavsdk::Telemetry telem{*this->get_system()};
    const auto heading = telem.heading();

    const auto node = std::make_shared<rclcpp::Node>("rtabmap_listener");
    node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, [](nav_msgs::msg::Odometry::SharedPtr msg) {
        // Do something with the message
    });
    spin(node);


    return IMavController::start();
}
