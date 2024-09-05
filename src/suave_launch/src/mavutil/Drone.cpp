//
// Created by suave on 4/17/24.
//

#include "Drone.h"
#include <cmath>

Drone::Drone(std::shared_ptr<System> system): m_system(std::move(system))
{
    Timer timer{};
    timer.start();
    while (std::isnan(m_initial_heading_rad)) {
        m_initial_heading_rad = m_telemetry.heading().heading_deg * M_PI / 180.0;
        suave_log << "Waiting for initial heading...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (timer.has_been(3)) 
        {
            suave_log << "Input initial heading in degrees: ";
            double initial_heading;
            std::cin >> initial_heading;
            m_initial_heading_rad = initial_heading * M_PI / 180.0;
            break;
        }
    }
    suave_log << "Initial heading: " << m_initial_heading_rad << " rad\n";
}

Offboard::Result Drone::offboard_setpoint()
{
    return offboard().set_attitude({0, 0, 0, 0});
}

Offboard::Result Drone::set_relative_position_ned(const float& n, const float& e, const float& d)
{
    const auto [north_m, east_m, down_m] = m_position_velocity_ned.wait_for_next().unwrap().position;
    const auto yaw = m_attitude_euler.wait_for_next(1).unwrap().yaw_deg;
    return offboard().set_position_ned({north_m + n, east_m + e, down_m + d, yaw});
}

Offboard::Result Drone::offboard_land()
{
    const auto yaw = m_attitude_euler.wait_for_next().unwrap().yaw_deg;
    return offboard().set_velocity_ned({0, 0, 0.5, yaw});
}

Offboard::Result Drone::set_position_frd(const float& f, const float& r, const float& d, const float& yaw_deg)
{
    // TODO: Implement transformation using initial heading
    return Offboard::Result::Unknown;
}

Tune::Result Drone::play_waiting_tune()
{
    Tune::SongElement note = Tune::SongElement::NoteC;
    Tune::TuneDescription waiting_tune{{note, note, note}, 90};
    return tune().play_tune(waiting_tune);
}

Tune::Result Drone::play_ready_tune()
{
    Tune::SongElement c = Tune::SongElement::NoteC;
    Tune::SongElement d = Tune::SongElement::NoteD;
    Tune::SongElement e = Tune::SongElement::NoteE;
    Tune::SongElement b = Tune::SongElement::NoteB;
    Tune::TuneDescription ready_tune{{c, d, e, b}, 90};
    return tune().play_tune(ready_tune);
}
