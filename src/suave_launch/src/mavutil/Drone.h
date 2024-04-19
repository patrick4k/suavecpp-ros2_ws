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

#include "TelemetryProperty.h"
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

    // Telemetry Properties
    [[nodiscard]] double heading_rad() const { return m_heading.get().unwrap().heading_deg * M_PI / 180; }
    [[nodiscard]] bool in_air() const { return m_in_air.get().unwrap(); }

    Offboard::Result set_position_frd(const double& f, const double& r, const double& d, const double& yaw_deg);

private:
    // Mavsdk
    std::shared_ptr<System> m_system{};
    Action m_action{*m_system};
    Offboard m_offboard{*m_system};
    Telemetry m_telemetry{*m_system};

    // Telemetry Properties
    TelemetryProperty<Telemetry::Heading> m_heading {m_telemetry, &Telemetry::subscribe_heading};
    TelemetryProperty<bool> m_in_air {m_telemetry, &Telemetry::subscribe_in_air};

    // Misc
    double m_initial_heading_rad{ NAN };
};

#endif //DRONE_H
