//
// Created by suave on 4/17/24.
//

#include "SuaveVIOTestFlight.h"

#include "../common/SystemTask.h"
#include "../ros/RosNodeSpinner.h"
#include "../vio/VIOBridge.h"

#define sleep(sec) std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(sec * 1e6)));

#define try_mav(act, success) \
{\
    suave_log << "Starting " << #act << std::endl; \
    const auto act_result = act; \
    if (act_result != success) \
    { \
        suave_err << #act << " failed: " << act_result << std::endl; \
        this->shutdown(); \
        return; \
    } \
    suave_log << #act << ": " << act_result << std::endl;\
}

#define try_action(act) try_mav(act, Action::Result::Success)
#define try_offboard(act) try_mav(act, Offboard::Result::Success)

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
            "ros2 launch rtabmap_examples realsense_d435i_infra.launch.py"
        }
    };

    // Create ROS spinner and add vio bridge node
    RosNodeSpinner spinner{};
    std::shared_ptr<VIOBridge> vio_bridge_node = std::make_shared<VIOBridge>(m_drone.system(), m_drone.initial_heading_rad());
    spinner.add_node(vio_bridge_node);

    // Add task to s_tasks
    s_tasks = {
        &realsense_task,
        &rtabmap_task,
        &spinner
    };

    // Start tasks
    for (const auto task : s_tasks)
    {
        // task->start_in_thread();
    }

    suave_log << "Ready for flight?" << std::endl;
    await_confirmation;

    suave_log << "Starting flight plan" << std::endl;

    // Flight plan start ---------------------------------------------------------------

    // Start offboard and arm
    try_offboard(m_drone.set_relative_position_ned(0, 0, 0))
    try_offboard(m_drone.offboard().start())
    try_action(m_drone.action().arm())

    try_offboard(m_drone.set_relative_position_ned(0, 0, -2))
    sleep(10)
    try_offboard(m_drone.offboard().set_velocity_body({1, 0, 0, 360.0/5}));
    sleep(5)
    try_offboard(m_drone.offboard().set_velocity_body({0, 0, 0, 0}));
    sleep(1)
    try_action(m_drone.action().land())

    // Wait for drone to land
    while (m_drone.in_air()) sleep(1)
    try_action(m_drone.action().disarm())

    // Flight plan end ----------------------------------------------------------------

    endtask();
}

void SuaveVIOTestFlight::shutdown()
{
    suave_warn << "Failsafe activated" << std::endl;
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

    endtask();
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
    s_tasks.clear();
}
