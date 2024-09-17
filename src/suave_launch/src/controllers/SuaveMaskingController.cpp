//
// Created by patrick on 9/5/24.
//

#include "SuaveMaskingController.h"

#include "../common/SystemTask.h"
#include "../ros/RosNodeSpinner.h"
#include "../vio/CloudExporter.h"
#include "../vio/VIOBridge.h"
#include "../masking_pid/MaskingSubscriber.h"
#include "ControllerMacros.h"

void SuaveMaskingController::start() {
    // Create realsense and rtabmap nodes

    auto realsense_task = std::make_shared<SystemTask>(
        std::vector<std::string>{
            "source /opt/ros/humble/setup.bash",
            "ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true"
        }
    );
    m_task.push_back(realsense_task);

    auto rtabmap_task = std::make_shared<SystemTask>(
        std::vector<std::string>{
            "source /opt/ros/humble/setup.bash",
            "ros2 param set /camera/camera depth_module.emitter_enabled 0",
            "ros2 launch ~/Dev/suavecpp-ros2_ws/launch/suave_slam.py"
        }
    );
    m_task.push_back(rtabmap_task);

    // Create ROS spinner and add vio bridge and cloud exporter node
    auto vio_spinner = std::make_shared<RosNodeSpinner>();
    m_task.push_back(vio_spinner);

    const auto vio_bridge_node = std::make_shared<VIOBridge>(m_drone->system(), m_drone->initial_heading_rad());
    vio_spinner->add_node(vio_bridge_node);

    const auto cloud_exporter_node = std::make_shared<CloudExporter>();
    vio_spinner->add_node(cloud_exporter_node);

    auto masking_spinner = std::make_shared<RosNodeSpinner>();
    m_task.push_back(masking_spinner);

    auto masking_subscriber = std::make_shared<MaskingSubscriber>(m_drone.get());
    masking_spinner->add_node(masking_subscriber);

    suave_log << "Starting SLAM" << std::endl;

    realsense_task->start_in_thread();
    sleep(3);
    rtabmap_task->start_in_thread();

    suave_log << "Waiting for rtabmap to initialize..." << std::endl;

    sleep(5);

    suave_log << "Move the drone around to create initial map" << std::endl;
    std::cin.clear(); // need to clear cin before calling await_confirmation, maybe fix this later...
    await_confirmation;
    await_confirmation;

    suave_log << "Starting VIO" << std::endl;
    vio_spinner->start_in_thread();

    // Start offboard and arm
    try_action(m_drone->action().arm())
    sleep(5)
    try_offboard(m_drone->offboard_setpoint())
    sleep(1)
    try_offboard(m_drone->offboard().start())
    sleep(3)

    suave_log << "Ready to enable masking control" << std::endl;

    await_confirmation;

    masking_spinner->start_in_thread();

    while (true) {
        suave_log << "Input action: ";
        std::string buffer{};
        std::getline(std::cin, buffer);
        if (buffer == "land")
        {
            m_drone->offboard_wait_for_land();
        }
        if (buffer == "stop")
        {
            masking_subscriber->set_enable(false);
            m_drone->offboard_hold();
        }
        if (buffer == "start")
        {
            masking_subscriber->set_enable(true);
        }
        if (buffer == "takeoff")
        {
            try_offboard(m_drone->set_relative_position_ned(0, 0, -3))
        }
        if (buffer == "exit")
        {
            break;
        }
    }

    m_drone->offboard_wait_for_land();

    suave_log << "Stopping task" << std::endl;

    for (auto& task: m_task) {
        task->stop();
    }

    suave_log << "Exiting..." << std::endl;

}

void SuaveMaskingController::shutdown() {
    suave_log << "SuaveMaskingController::shutdown()" << std::endl;
    m_drone->offboard_wait_for_land();
    
    for (auto& task: m_task) {
        task->stop();
    }
}
