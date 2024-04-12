//
// Created by suave on 4/12/24.
//

#include "SuaveTaskManager.h"

#include <mavsdk.h>
#include <thread>

#include "../nav/TakeoffLandFlightPlan.h"
#include "../vio/VIOBridge.h"

void SuaveTaskManager::start()
{
    suave_log << "SuaveTaskManager::start()" << std::endl;

    // Create a TakeoffLandFlightPlan controller
    auto takeoff_land_flight_plan = TakeoffLandFlightPlan{m_system};

    // Add the flight plan to controllers for failsafe
    m_controllers.push_back(&takeoff_land_flight_plan);

    // Create VIO bridge
    auto vio_bridge = VIOBridge{m_system};

    // Add the VIO bridge to controllers for failsafe
    m_controllers.push_back(&vio_bridge);

    // Start VIO bridge in new thread
//    auto vio_future = vio_bridge.start_in_thread();
    auto vio_result = vio_bridge.start();

    // Start the flight plan
    auto flight_plan_result = takeoff_land_flight_plan.start();

    // Kill VIO bridge
    vio_bridge.stop();
//    auto vio_result = vio_future->get();

    if (flight_plan_result != MavControllerResult::SUCCESS)
    {
        std::cerr << "TakeoffLandFlightPlan failed" << std::endl;
    }

    if (vio_result != MavControllerResult::SUCCESS)
    {
        std::cerr << "VIOBridge failed" << std::endl;
    }

    suave_log << "SuaveTaskManager::start() done" << std::endl;
}
