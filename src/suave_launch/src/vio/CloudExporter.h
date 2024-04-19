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

    static void SavePCD()
    {
        suave_log << "Saving cloud" << std::endl;
        CloudExporter exporter{};
        rclcpp::executors::SingleThreadedExecutor executor{};
        executor.add_node(exporter.get_node_base_interface());
        while (!exporter.m_saved)
        {
            executor.spin_once();
        }
        suave_log << "Cloud saved" << std::endl;
    }

private:
    using Subscription = rclcpp::Subscription<PointCloudMsg>;
    std::optional<Subscription::SharedPtr> m_subscription{};
    bool m_saved{false};

    void callback(const PointCloudMsg::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received point cloud message with %d points", msg->width * msg->height);
        pcl::PointCloud<pcl::PointXYZ> cloud{};
        fromROSMsg(*msg, cloud);
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
        std::string filename = "/home/suave/Data/cloud_" + ss.str() + ".pcd";
        pcl::io::savePCDFileASCII(filename, cloud);
        m_saved = true;
    }
};



#endif //CLOUDEXPORTER_H
