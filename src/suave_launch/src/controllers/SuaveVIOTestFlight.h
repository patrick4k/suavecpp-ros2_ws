//
// Created by suave on 4/17/24.
//

#ifndef SUAVEVIOTESTFLIGHT_H
#define SUAVEVIOTESTFLIGHT_H

#include "../common/ISuaveController.h"
#include "../mavutil/Drone.h"
#include "../mavutil/Mavlink.h"

class SuaveVIOTestFlight: public ISuaveController {
public:
    SuaveVIOTestFlight() :
        ISuaveController(),
        m_drone(Mavlink::connectToSerial())
    {
    }

    void start() override;
    void shutdown() override;

private:
    static void endtask();
    bool m_end_controller{false};
    Drone m_drone;
};

#endif //SUAVEVIOTESTFLIGHT_H
