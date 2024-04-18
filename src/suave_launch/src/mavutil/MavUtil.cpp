//
// Created by suave on 4/17/24.
//

#include "MavUtil.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "../util/Util.h"

std::shared_ptr<mavsdk::System> connectToPX4SITL(mavsdk::Mavsdk& m_mavsdk)
{
    // Connect to the PX4 SITL.
    mavsdk::ConnectionResult connection_result = m_mavsdk.add_any_connection("udp://:14540");
    if (connection_result != mavsdk::ConnectionResult::Success)
    {
        throw std::runtime_error((std::stringstream() << "Connection failed: " << connection_result << '\n').str());
    }

    // Wait until the vehicle is ready.
    while (m_mavsdk.systems().empty())
    {
        suave_log << "Waiting for system to connect...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return *m_mavsdk.systems().begin();
}
