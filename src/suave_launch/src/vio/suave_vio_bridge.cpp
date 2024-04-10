#include <system.h>
#include <plugins/telemetry/telemetry.h>

#include "OdomGeometry.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{

}

void launch(int argc = 0, char *argv[] = nullptr)
{
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<rclcpp::Node>("rtabmap_listener");
    node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, odom_callback);
    spin(node);
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    launch(argc, argv);
    return 0;
}

void external_launch(const std::shared_ptr<mavsdk::System>& system)
{
    mavsdk::Telemetry telem{system};
    const auto heading = telem.heading();
    SetInitialHeading(heading.heading_deg * M_PI / 180.0);
    launch();
}
