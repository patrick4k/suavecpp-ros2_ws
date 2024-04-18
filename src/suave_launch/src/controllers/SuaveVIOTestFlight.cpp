//
// Created by suave on 4/17/24.
//

#include "SuaveVIOTestFlight.h"

#include "../common/SystemTask.h"
#include "../ros/RosNodeSpinner.h"
#include "../vio/VIOBridge.h"

#define sleep(sec) std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(sec * 1e6)))

#define try_mav(act, success) if (act != success) { suave_err << #act << " failed" << std::endl; this->failsafe(); return; }
#define try_action(act) try_mav(act, Action::Result::Success)
#define try_offboard(act) try_mav(act, Offboard::Result::Success)

void SuaveVIOTestFlight::start()
{
    // Create realsense and rtabmap nodes
    SystemTask realsense_task{
        {
            "ros",
            "ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true"
        }
    };
    SystemTask rtabmap_task{
        {
            "ros",
            "ros2 param set /camera/camera depth_module.emitter_enabled 0",
            "ros2 launch rtabmap_examples realsense_d435i_infra.launch.py"
        }
    };

    // Create ROS spinner and add vio bridge node
    RosNodeSpinner spinner{};
    std::shared_ptr<VIOBridge> vio_bridge_node = std::make_shared<VIOBridge>(m_drone.system(), m_drone.initial_heading_rad());
    spinner.add_node(vio_bridge_node);

    // Start realsense and rtabmap nodes
    realsense_task.start_in_thread();
    rtabmap_task.start_in_thread();

    // Start ROS spinner
    auto spinner_result = spinner.start_in_thread();


    // Flight plan start ---------------------------------------------------------------
    try_action(m_drone.action().arm())
    sleep(2);
    try_action(m_drone.action().disarm())
    // Flight plan end ----------------------------------------------------------------


    spinner.stop();
    if (spinner_result->get() != TaskResult::SUCCESS)
    {
        suave_err << "RosNodeSpinner failed" << std::endl;
    }

    spinner.stop();

}
void SuaveVIOTestFlight::failsafe()
{
    suave_warn << "Failsafe activated" << std::endl;
    auto land_result = m_drone.action().land();
    if (land_result != Action::Result::Success)
    {
        suave_err << "Landing failed, attempting to kill drone" << std::endl;
        auto kill_result = m_drone.action().kill();
        if (kill_result != Action::Result::Success)
        {
            suave_err << "Failed to kill drone" << std::endl;
        }
    }
}
