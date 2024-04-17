//
// Created by suave on 4/12/24.
//

#include "SuaveTaskManager.h"

#include "../nav/TakeoffLandFlightPlan.h"
#include "../ros/RosNodeSpinner.h"
#include "../vio/VIOBridge.h"

void SuaveTaskManager::start()
{
    suave_log << "SuaveTaskManager::start()" << std::endl;

    auto takeoff_land_flight_plan = TakeoffLandFlightPlan{m_system};
    m_controllers.push_back(&takeoff_land_flight_plan);

    auto vio_bridge = VIOBridge{m_system};
    RosNodeSpinner spinner{};
    spinner.add(vio_bridge.get_node_base_interface());
    m_controllers.push_back(&spinner);

    auto vio_result = spinner.start_in_thread();

    auto flight_plan_result = takeoff_land_flight_plan.start();

    spinner.stop();

    if (flight_plan_result != TaskResult::SUCCESS)
    {
        std::cerr << "TakeoffLandFlightPlan failed" << std::endl;
    }

    if (vio_result->get() != TaskResult::SUCCESS)
    {
        std::cerr << "VIOBridge failed" << std::endl;
    }

    suave_log << "SuaveTaskManager::start() done" << std::endl;
}
