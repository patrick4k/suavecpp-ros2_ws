#include <iostream>
#include <mavsdk.h>
#include <thread>
#include <rclcpp/utilities.hpp>

#include "common/SuaveTaskManager.h"
#include "nav/TakeoffLandFlightPlan.h"
#include "vio/MavVIOBridge.h"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto controller_result = SuaveTaskManager::create();
    if (!controller_result.has_value())
    {
        std::cerr << "Failed to create SuaveTaskManager\n";
        return 1;
    }
    auto controller = controller_result.value();
    controller.start();
    rclcpp::shutdown();
}

int main1(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    mavsdk::Mavsdk mavsdk{};

    // Connect to the PX4 SITL.
    mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14540");
    if (connection_result != mavsdk::ConnectionResult::Success)
    {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    // Wait until the vehicle is ready.
    while (mavsdk.systems().empty())
    {
        std::cout << "Waiting for system to connect...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    auto system = *mavsdk.systems().begin();
    std::cout << "System connected\n";



    // TODO: Get ROS nodes spinning

    // MavVIOBridge vio_bridge{system};
    // auto vio_future = vio_bridge.start_in_thread();

    TakeoffLandFlightPlan takeoff_land_flight_plan{system};
    takeoff_land_flight_plan.start_in_thread();

    std::this_thread::sleep_for(std::chrono::seconds(30));

    takeoff_land_flight_plan.stop();

    rclcpp::shutdown();
}
