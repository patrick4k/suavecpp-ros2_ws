//
// Created by suave on 4/12/24.
//

#include "SuaveTaskManager.h"

#include "../nav/TakeoffLandFlightPlan.h"
#include "../vio/VIOBridge.h"

void SuaveTaskManager::start()
{
    suave_log << "SuaveTaskManager::start()" << std::endl;

    auto takeoff_land_flight_plan = TakeoffLandFlightPlan{m_system};
    m_controllers.push_back(&takeoff_land_flight_plan);

    auto vio_bridge = VIOBridge{m_system};
    m_controllers.push_back(&vio_bridge);

    auto vio_result = vio_bridge.start();

    auto flight_plan_result = takeoff_land_flight_plan.start();

    vio_bridge.stop();

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
