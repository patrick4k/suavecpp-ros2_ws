//
// Created by suave on 4/12/24.
//

#include "SuaveTaskManager.h"

#include <mavsdk.h>
#include <thread>

#include "../nav/TakeoffLandFlightPlan.h"
#include "../vio/VIOBridge.h"

std::optional<SuaveTaskManager> SuaveTaskManager::create()
{
    mavsdk::Mavsdk mavsdk{};

    // Connect to the PX4 SITL.
    mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14540");
    if (connection_result != mavsdk::ConnectionResult::Success)
    {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return {};
    }

    // Wait until the vehicle is ready.
    while (mavsdk.systems().empty())
    {
        std::cout << "Waiting for system to connect...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    auto system = *mavsdk.systems().begin();
    std::cout << "System connected\n";

    return SuaveTaskManager{std::move(system)};
}

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
    auto vio_future = vio_bridge.start_in_thread();

    // Start the flight plan
    auto flight_plan_result = takeoff_land_flight_plan.start();

    // Kill VIO bridge
    vio_bridge.stop();
    auto vio_result = vio_future->get();

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
