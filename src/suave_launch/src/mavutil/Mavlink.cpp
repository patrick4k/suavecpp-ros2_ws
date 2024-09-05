//
// Created by patrick on 5/11/24.
//

#include "Mavlink.h"

#include <thread>
#include "../util/Util.h"

std::shared_ptr<mavsdk::System> Mavlink::connectToPX4SITL(uint32_t port) {
    static std::vector<std::shared_ptr<mavsdk::System>> s_connected_systems{};

    const mavsdk::ConnectionResult connection_result = s_mavsdk.add_any_connection("udp://:" + std::to_string(port));
    if (connection_result != mavsdk::ConnectionResult::Success)
    {
        throw std::runtime_error((std::stringstream() << "Connection failed: " << connection_result << '\n').str());
    }

    // Wait until the vehicle is ready.
    while (true)
    {
        suave_log << "Waiting for system to connect...\n";

        for (const auto& system : s_mavsdk.systems())
        {
            if (std::find(s_connected_systems.begin(), s_connected_systems.end(), system) == s_connected_systems.end()) {
                s_connected_systems.push_back(system);
                return system;
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    throw std::runtime_error("No new system found");
}

std::shared_ptr<mavsdk::System> Mavlink::connectToSerial() {
    // TODO: Scan serial ports and connect to pixhawk
    return nullptr;
}
