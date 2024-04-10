//
// Created by suave on 4/10/24.
//

#ifndef TAKEOFFLANDFLIGHTPLAN_H
#define TAKEOFFLANDFLIGHTPLAN_H
#include "ISystemController.h"


class TakeoffLandFlightPlan: public ISystemController {
public:
    SystemControllerResult start() override;
};

#endif //TAKEOFFLANDFLIGHTPLAN_H
