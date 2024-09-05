//
// Created by patrick on 5/11/24.
//

#include "GazeboSwarmTest.h"
#include "../common/ITask.h"
#include "../common/SystemTask.h"

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

std::vector<std::shared_ptr<SystemTask>> s_gazebo_tasks{};
std::vector<Drone> s_drones;

void GazeboSwarmTest::start() {
    constexpr uint32_t NUMBER_OF_DRONES = 10;

    for (uint32_t i = 0; i < NUMBER_OF_DRONES; i++)
    {
        constexpr uint32_t PORT0 = Mavsdk::DEFAULT_UDP_PORT;

        auto new_drone = std::make_shared<SystemTask>(
            std::vector<std::string>{
                "cd ~/Dev/PX4-Autopilot",
                "PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE=\"" + std::to_string(i) + "\" ./build/px4_sitl_default/bin/px4 -i " + std::to_string(i)
            }
        );

        new_drone->start_in_thread();

        s_gazebo_tasks.push_back(new_drone);

        s_drones.emplace_back(Mavlink::connectToPX4SITL(PORT0 + i), i);
    }

    suave_log << "Sleeping for 5" << std::endl;
    sleep(5)

    for (auto& drone: s_drones) {
        try_tune(drone.play_waiting_tune())
        suave_log << "Ready to start?" << std::endl;
        await_confirmation;

        suave_log << "Starting flight plan" << std::endl;

        // Start offboard and arm
        try_offboard(drone.set_relative_position_ned(0, 0, 0))
        try_offboard(drone.offboard().start())
        try_action(drone.action().arm())
        try_offboard(drone.set_relative_position_ned(0,0,-2))
        sleep(10)
        try_offboard(drone.offboard_land())

        // Wait for drone to land
        int elapse_sec = 0;
        while (drone.in_air())
        {
            suave_log << "Waiting for drone to land, elapse_sec: " << elapse_sec << std::endl;
            sleep(1)
            elapse_sec++;
        }
        try_action(drone.action().disarm())
    }

    endtask();
}

void GazeboSwarmTest::shutdown() {
    bool is_first_shutdown = false;
    if (m_end_controller) is_first_shutdown = true;
    m_end_controller = true;
    suave_warn << "Suave VIO Test Shutdown" << std::endl;

    for (auto& drone: s_drones) {
        if (drone.in_air())
        {
            if (drone.action().land() != Action::Result::Success)
            {
                suave_err << "Landing failed, attempting to kill drone" << std::endl;
                auto kill_result = drone.action().kill();
                if (kill_result != Action::Result::Success)
                {
                    suave_err << "Failed to kill drone" << std::endl;
                }
            }
        }
    }

    if (is_first_shutdown) endtask();
}

void GazeboSwarmTest::endtask()
{
    for (auto& task : s_gazebo_tasks)
    {
        task->stop();
    }
}
