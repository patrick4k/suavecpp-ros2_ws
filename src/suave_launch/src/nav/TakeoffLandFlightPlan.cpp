#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <iostream>

#include "TakeoffLandFlightPlan.h"

#include <chrono>
#include <thread>
#include <plugins/mocap/mocap.h>

using namespace mavsdk;
using std::this_thread::sleep_for;
using std::chrono::seconds;

#define check if (m_should_cancel) return TaskResult::CANCELLED;

TaskResult TakeoffLandFlightPlan::start()
{
    // Make sure we have a telemetry plugin
    auto telemetry = Telemetry{*this->get_system()};

    // We want to listen to the altitude of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = telemetry.set_rate_position(1.0);

    if (set_rate_result != Telemetry::Result::Success) {
        std::cerr << "Setting rate failed:" << set_rate_result << '\n';
        return TaskResult::FAILURE;
    }

    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
    });

    // Make sure we have an action plugin
    auto action = Action{*this->get_system()};

    // Arm the drone
    check
    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();

    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed:" << arm_result << '\n';
        return TaskResult::FAILURE;
    }

    // Take off
    check
    std::cout << "Taking off...\n";
    const Action::Result takeoff_result = action.takeoff();

    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed:" << takeoff_result << '\n';
        return TaskResult::FAILURE;
    }

    sleep_for(seconds(10));

    auto offboard = Offboard{*get_system()};
    offboard.set_velocity_body(Offboard::VelocityBodyYawspeed{1, 0, 0, 0});
    auto offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed:" << offboard_result << '\n';
        return TaskResult::FAILURE;
    }
    sleep_for(seconds(1));
    offboard.set_velocity_body(Offboard::VelocityBodyYawspeed{0, 0, 0, 0});

    // Let it hover for a bit before landing again.
    sleep_for(seconds(5));

    // Land the drone
    check
    std::cout << "Landing...\n";
    const Action::Result land_result = action.land();

    if (land_result != Action::Result::Success) {
        std::cerr << "Land failed:" << land_result << '\n';
        return TaskResult::FAILURE;
    }

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    check
    sleep_for(seconds(5));
    std::cout << "Finished...\n";

    return TaskResult::SUCCESS;
}
