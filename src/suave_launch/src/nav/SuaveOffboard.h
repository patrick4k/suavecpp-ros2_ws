//
// Created by suave on 4/15/24.
//

#ifndef SUAVEOFFBOARD_H
#define SUAVEOFFBOARD_H
#include <plugins/offboard/offboard.h>
#include <plugins/telemetry/telemetry.h>

#include "../common/IMavController.h"
#include "../util/Result.h"

using namespace mavsdk;

class SuaveOffboard : public IMavController {
public:
    explicit SuaveOffboard(std::shared_ptr<System> system) :
    IMavController(std::move(system)),
    m_offboard(*get_system())
    {
        const auto start_time = std::chrono::system_clock::now();
        Telemetry telemetry{*get_system()};
        telemetry.subscribe_position_velocity_ned(
            [this, start_time](const Telemetry::PositionVelocityNed& pos_vel)
            {
                this->m_position_velocity_ned = pos_vel;
                m_posvel_elapse_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - start_time).count();
            }
        );
    }

    Result<Telemetry::PositionVelocityNed> get_position_velocity_ned(const double& min_elapse_s = 1.0)
    {
        const auto min_elase_us =static_cast<u_int32_t>(min_elapse_s * 1e6);
        if (min_elase_us < m_posvel_elapse_us)
        {
            return "Position not received in " + std::to_string(min_elapse_s) + " seconds";
        }
        return m_position_velocity_ned;
    }

private:
    u_int32_t m_posvel_elapse_us{};
    Result<Telemetry::PositionVelocityNed> m_position_velocity_ned{"Position has not yet been received"};
    Offboard m_offboard;
};

#endif //SUAVEOFFBOARD_H
