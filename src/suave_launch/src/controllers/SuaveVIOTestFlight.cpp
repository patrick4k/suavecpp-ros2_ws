//
// Created by suave on 4/17/24.
//

#include "SuaveVIOTestFlight.h"

#include "../common/SystemTask.h"
#include "../ros/RosNodeSpinner.h"
#include "../vio/CloudExporter.h"
#include "../vio/VIOBridge.h"

#include "ControllerMacros.h"

std::vector<ITask*> s_tasks{};

void SuaveVIOTestFlight::start()
{
    // Create realsense and rtabmap nodes
    SystemTask realsense_task{
        {
            "source /opt/ros/humble/setup.bash",
            "ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true"
        }
    };
    SystemTask rtabmap_task{
        {
            "source /opt/ros/humble/setup.bash",
            "ros2 param set /camera/camera depth_module.emitter_enabled 0",
            "ros2 launch ~/Dev/suavecpp-ros2_ws/launch/suave_slam.py"
        }
    };

    // Create ROS spinner and add vio bridge and cloud exporter node
    RosNodeSpinner spinner{};
    const auto vio_bridge_node = std::make_shared<VIOBridge>(m_drone.system(), m_drone.initial_heading_rad());
    const auto cloud_exporter_node = std::make_shared<CloudExporter>();
    spinner.add_node(vio_bridge_node);
    spinner.add_node(cloud_exporter_node);

    // Add task to s_tasks
    s_tasks = {
        &realsense_task,
        &rtabmap_task,
        &spinner
    };

    try_tune(m_drone.play_waiting_tune())
    suave_log << "Ready to start?" << std::endl;
    await_confirmation;

    suave_log << "Starting realsense and rtabmap" << std::endl;
    realsense_task.start_in_thread();
    sleep(2)
    rtabmap_task.start_in_thread();

    sleep(5)
    try_tune(m_drone.play_waiting_tune())
    suave_log << "Move the drone around to create initial map" << std::endl;
    await_confirmation;

    suave_log << "Starting VIO" << std::endl;
    spinner.start_in_thread();

    sleep(5)
    try_tune(m_drone.play_ready_tune())
    suave_log << "Ready for flight?" << std::endl;
    await_confirmation;

    suave_log << "Starting flight plan" << std::endl;

    // Flight plan start ---------------------------------------------------------------

    // Start offboard and arm
    try_action(m_drone.action().arm())
    try_offboard(m_drone.offboard_setpoint())
    try_offboard(m_drone.offboard().start())

    sleep(5)

    try_offboard(m_drone.set_relative_position_ned(0,0,-2))
    sleep(5)
    try_offboard(m_drone.offboard_land())

    // Wait for drone to land
    int elapse_sec = 0;
    while (m_drone.in_air())
    {
        suave_log << "Waiting for drone to land, elapse_sec: " << elapse_sec << std::endl;
        sleep(1)
        elapse_sec++;
    }
    sleep(3)
    try_action(m_drone.action().disarm())

    // Flight plan end ----------------------------------------------------------------
    cloud_exporter_node->export_cloud();
    endtask();
}

void SuaveVIOTestFlight::shutdown()
{
    bool is_first_shutdown = false;
    if (m_end_controller) is_first_shutdown = true;
    m_end_controller = true;
    suave_warn << "Suave VIO Test Shutdown" << std::endl;
    if (m_drone.in_air())
    {
        if (m_drone.action().land() != Action::Result::Success)
        {
            suave_err << "Landing failed, attempting to kill drone" << std::endl;
            auto kill_result = m_drone.action().kill();
            if (kill_result != Action::Result::Success)
            {
                suave_err << "Failed to kill drone" << std::endl;
            }
        }
    }

    if (is_first_shutdown) endtask();
}

void SuaveVIOTestFlight::endtask()
{
    for (auto task : s_tasks)
    {
        if (task)
        {
            task->stop();
        }
    }
}
