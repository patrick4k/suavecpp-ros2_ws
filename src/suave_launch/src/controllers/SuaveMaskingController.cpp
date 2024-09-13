//
// Created by patrick on 9/5/24.
//

#include "SuaveMaskingController.h"

#include "../common/SystemTask.h"
#include "../ros/RosNodeSpinner.h"
#include "../vio/CloudExporter.h"
#include "../vio/VIOBridge.h"
#include "../masking_pid/MaskingSubscriber.h"

#define sleep(sec) std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(sec * 1e6)));

#define try_mav(act, success) \
{\
    suave_log << "Starting " << #act << std::endl; \
    if (m_end_controller)     \
    {\
        suave_err << "m_end_controller = true" << std::endl; \
        this->shutdown();\
        return;\
    }\
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
#define try_tune(act) try_mav(act, Tune::Result::Success)

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
            "ros2 launch rtabmap_examples realsense_d435i_infra.launch.py"
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

    suave_log << "Move the drone around to create initial map" << std::endl;
    await_confirmation;

    suave_log << "Starting VIO" << std::endl;
    vio_spinner->start_in_thread();

    sleep(30);

    suave_log << "Stopping task" << std::endl;

    for (auto& task: m_task) {
        task->stop();
    }

    suave_log << "Exiting..." << std::endl;

}

void SuaveMaskingController::shutdown() {
    for (auto& task: m_task) {
        task->stop();
    }
}
