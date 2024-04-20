//
// Created by suave on 4/18/24.
//

#include "CloudExporter.h"

void CloudExporter::export_cloud() const
{
    suave_log << "Exporting cloud" << std::endl;
    if (m_msg == nullptr)
    {
        suave_err << "Cannot export cloud: m_msg is null" << std::endl;
        return;
    }

    try
    {
        pcl::PointCloud<pcl::PointXYZ> cloud{};
        fromROSMsg(*m_msg, cloud);
        const auto now = std::chrono::system_clock::now();
        const auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d_%H-%M-%S");
        const std::string filename = "/home/suave/Data/cloud_" + ss.str() + ".pcd";
        pcl::io::savePCDFileASCII(filename, cloud);
    }
    catch (const std::exception& e)
    {
        suave_err << "Error saving cloud: " << e.what() << std::endl;
    }
}

void CloudExporter::callback(const PointCloudMsg::SharedPtr msg)
{
    m_msg = msg;
}
