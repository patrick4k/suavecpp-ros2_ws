//
// Created by suave on 4/18/24.
//

#ifndef CLOUDEXPORTER_H
#define CLOUDEXPORTER_H
#include "../ros/RosNodeSpinner.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.12/pcl/io/pcd_io.h>

class CloudExporter final : public rclcpp::Node {
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;

public:
    CloudExporter() : Node("suave_cloud_exporter")
    {
        m_subscription = this->create_subscription<PointCloudMsg>("/cloud_map", 10, std::bind(&CloudExporter::callback, this, std::placeholders::_1));
    }

    void export_cloud() const;

private:
    using Subscription = rclcpp::Subscription<PointCloudMsg>;
    Subscription::SharedPtr m_subscription{nullptr};
    PointCloudMsg::SharedPtr m_msg{nullptr};

    void callback(const PointCloudMsg::SharedPtr msg);
};

#endif //CLOUDEXPORTER_H
