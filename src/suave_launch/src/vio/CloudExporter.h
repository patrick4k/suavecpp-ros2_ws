//
// Created by suave on 4/18/24.
//

#ifndef CLOUDEXPORTER_H
#define CLOUDEXPORTER_H
#include "../ros/RosNodeSpinner.h"

#include <sensor_msgs/msg/point_cloud2.hpp>

class CloudExporter final : public rclcpp::Node {
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
public:
    CloudExporter() : Node("suave_cloud_exporter")
    {
        m_subscription = this->create_subscription<PointCloudMsg>("/cloud_map", 10, std::bind(&CloudExporter::callback, this, std::placeholders::_1));
    }

private:
    void callback(const PointCloudMsg::SharedPtr msg)
    {
        // TODO: export the point cloud to a file
        RCLCPP_INFO(this->get_logger(), "Received point cloud message with %d points", msg->width * msg->height);
    }

    using Subscription = rclcpp::Subscription<PointCloudMsg>;
    std::optional<Subscription::SharedPtr> m_subscription{};
};



#endif //CLOUDEXPORTER_H
