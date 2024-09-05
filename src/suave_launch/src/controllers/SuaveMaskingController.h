//
// Created by patrick on 9/5/24.
//

#ifndef SUAVEMASKINGCONTROLLER_H
#define SUAVEMASKINGCONTROLLER_H

#include "../common/ISuaveController.h"
#include "../mavutil/Drone.h"
#include "../mavutil/MavUtil.h"

class SuaveMaskingController: public ISuaveController {
public:
    SuaveMaskingController() :
        ISuaveController()
    {
        try {
            // TODO: Correct for test case
            //m_drone = std::make_unique<Drone>(connectToPX4SITL(m_mavsdk));
        }
        catch (...) { /*ignore*/ }
    }

    void start() override;
    void shutdown() override;

private:
    std::unique_ptr<Drone> m_drone{ nullptr };
};

#endif //SUAVEMASKINGCONTROLLER_H
