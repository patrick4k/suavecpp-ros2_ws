//
// Created by suave on 4/10/24.
//

#ifndef TAKEOFFLANDFLIGHTPLAN_H
#define TAKEOFFLANDFLIGHTPLAN_H
#include "../common/IMavController.h"

class TakeoffLandFlightPlan : public IMavController {
public:
    explicit TakeoffLandFlightPlan(const std::shared_ptr<mavsdk::System>& system)
        : IMavController(system)
    {
    }

    MavControllerResult start() override;

    void stop() override
    {
        suave_log << "TakeoffLandFlightPlan::stop()" << std::endl;
        m_should_cancel = true;
    }
private:
    std::atomic_bool m_should_cancel { false };
};

#endif //TAKEOFFLANDFLIGHTPLAN_H
