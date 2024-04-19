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
#include <plugins/param/param.h>
#include <plugins/tune/tune.h>

#include "TelemetryProperty.h"
#include "../util/Util.h"

using namespace mavsdk;

class Drone {
public:
    explicit Drone(std::shared_ptr<System> system);

    std::shared_ptr<System> system() { return m_system; }
    Action& action() { return m_action; }
    Offboard& offboard() { return m_offboard; }
    Telemetry& telemetry() { return m_telemetry; }
    Param& param() { return m_param; }
    Tune& tune() { return m_tune; }

    [[nodiscard]]
    double initial_heading_rad() const { return m_initial_heading_rad; }

    // Telemetry Properties
    [[nodiscard]]
    double heading_rad() const
    {
        return m_heading.get().unwrap().heading_deg * M_PI / 180;
    }
    [[nodiscard]]
    bool in_air() const
    {
        return m_in_air.get().unwrap();
    }

    // Blocking until next position update
    Offboard::Result set_relative_position_ned(const float& n, const float& e, const float& d);

    Offboard::Result set_position_frd(const float& f, const float& r, const float& d, const float& yaw_deg);

private:
    // Mavsdk
    std::shared_ptr<System> m_system{};
    Action m_action{*m_system};
    Offboard m_offboard{*m_system};
    Telemetry m_telemetry{*m_system};
    Param m_param{*m_system};
    Tune m_tune{*m_system};

    // Telemetry Properties
    TelemetryProperty<Telemetry::Heading> m_heading {m_telemetry, &Telemetry::subscribe_heading};
    TelemetryProperty<bool> m_in_air {m_telemetry, &Telemetry::subscribe_in_air};
    TelemetryProperty<Telemetry::PositionVelocityNed> m_position_velocity_ned {m_telemetry, &Telemetry::subscribe_position_velocity_ned};
    TelemetryProperty<Telemetry::EulerAngle> m_attitude_euler {m_telemetry, &Telemetry::subscribe_attitude_euler};

    // Misc
    double m_initial_heading_rad{ NAN };
};

#endif //DRONE_H
