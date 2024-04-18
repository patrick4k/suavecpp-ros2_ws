//
// Created by suave on 4/17/24.
//

#include "Drone.h"

Drone::Drone(std::shared_ptr<System> system): m_system(std::move(system))
{
    while (std::isnan(m_initial_heading_rad)) {
        m_initial_heading_rad = m_telemetry.heading().heading_deg * M_PI / 180.0;
        suave_log << "Waiting for initial heading...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    suave_log << "Initial heading: " << m_initial_heading_rad << " rad\n";
}

Offboard::Result Drone::set_position_frd(const double& f, const double& r, const double& d, const double& yaw_deg)
{
    // TODO: Implement transformation using initial heading
    return Offboard::Result::Unknown;
}
