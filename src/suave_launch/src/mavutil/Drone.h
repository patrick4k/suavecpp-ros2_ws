//
// Created by suave on 4/17/24.
//

#ifndef DRONE_H
#define DRONE_H
#include <system.h>
#include <thread>
#include <plugins/action/action.h>

#include <plugins/offboard/offboard.h>
#include <plugins/telemetry/telemetry.h>

#include "../util/Util.h"

using namespace mavsdk;

class Drone {
public:
    explicit Drone(std::shared_ptr<System> system);

    Action& action() { return m_action; }
    Offboard& offboard() { return m_offboard; }
    Telemetry& telemetry() { return m_telemetry; }
    [[nodiscard]] double initial_heading_rad() const { return m_initial_heading_rad; }
    std::shared_ptr<System> system() { return m_system; }

    Offboard::Result set_position_frd(const double& f, const double& r, const double& d, const double& yaw_deg);

private:
    std::shared_ptr<System> m_system{};
    Action m_action{*m_system};
    Offboard m_offboard{*m_system};
    Telemetry m_telemetry{*m_system};
    double m_initial_heading_rad{ NAN };
};

#endif //DRONE_H
