//
// Created by suave on 4/10/24.
//

#ifndef TAKEOFFLANDFLIGHTPLAN_H
#define TAKEOFFLANDFLIGHTPLAN_H
#include "IFlightPlan.h"


class TakeoffLandFlightPlan: public IFlightplan {
public:
    FlightPlanResult start(std::shared_ptr<mavsdk::System>& system) override;
};


#endif //TAKEOFFLANDFLIGHTPLAN_H
