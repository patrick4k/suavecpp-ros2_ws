//
// Created by suave on 4/18/24.
//

#include "CloudExporter.h"

void CloudExporter::SavePCD()
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

void CloudExporter::callback(const PointCloudMsg::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received point cloud message with %d points", msg->width * msg->height);
    try
    {
        pcl::PointCloud<pcl::PointXYZ> cloud{};
        fromROSMsg(*msg, cloud);
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
        std::string filename = "/home/suave/Data/cloud_" + ss.str() + ".pcd";
        pcl::io::savePCDFileASCII(filename, cloud);
    }
    catch (const std::exception& e)
    {
        suave_err << "Error saving cloud: " << e.what() << std::endl;
    }
    m_saved = true;
}
