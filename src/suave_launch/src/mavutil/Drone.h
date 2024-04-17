//
// Created by suave on 4/17/24.
//

#ifndef DRONE_H
#define DRONE_H
#include <system.h>

#include <plugins/offboard/offboard.h>
#include <plugins/telemetry/telemetry.h>

using namespace mavsdk;

class Drone {
public:
    explicit Drone(std::shared_ptr<System> system) : m_system(std::move(system))
    {
    }

    std::shared_ptr<System> get_system()
    {
        return m_system;
    }

private:
    std::shared_ptr<System> m_system{};
    Offboard m_offboard{*m_system};
    Telemetry m_telemetry{*m_system};
};

#endif //DRONE_H
