//
// Created by suave on 4/17/24.
//

#ifndef SUAVEVIOTESTFLIGHT_H
#define SUAVEVIOTESTFLIGHT_H
#include <connection_result.h>
#include <iostream>
#include <thread>

#include "../common/ISuaveController.h"
#include "../mavutil/Drone.h"
#include "../mavutil/MavUtil.h"


namespace mavsdk
{
    enum class ConnectionResult;
}

class SuaveVIOTestFlight: public ISuaveController {
public:
    SuaveVIOTestFlight() :
        ISuaveController(),
        m_drone(connectToPX4SITL(m_mavsdk))
    {
    }

    void start() override;
    void failsafe();

private:
    Drone m_drone;
};

#endif //SUAVEVIOTESTFLIGHT_H
