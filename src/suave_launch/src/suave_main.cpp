#include <iostream>
#include <mavsdk.h>
#include <thread>

#include "nav/ISystemController.h"
#include "nav/TakeoffLandFlightPlan.h"

int main(int argc, char **argv)
{
    mavsdk::Mavsdk mavsdk{};

    // Connect to the PX4 SITL.
    mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14540");
    if (connection_result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    // Wait until the vehicle is ready.
    while (mavsdk.systems().empty()) {
        std::cout << "Waiting for system to connect...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    auto system = *mavsdk.systems().begin();
    std::cout << "System connected\n";

    // TODO: Get ROS nodes spinning

    TakeoffLandFlightPlan takeoff_land_flight_plan{};
    takeoff_land_flight_plan.set_system(system);
    takeoff_land_flight_plan.start();
}
