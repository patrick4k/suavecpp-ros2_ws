#ifndef IFLIGHTPLAN_H
#define IFLIGHTPLAN_H
#include <system.h>

enum class FlightPlanResult
{
    SUCCESS,
    FAILURE,
    CANCELED,
    INVALID,
    NOT_IMPLMENTED
};

class IFlightplan
{
public:
    virtual ~IFlightplan() = default;

    virtual FlightPlanResult start(std::shared_ptr<mavsdk::System>& system)
    {
        return FlightPlanResult::NOT_IMPLMENTED;
    }

};

#endif //IFLIGHTPLAN_H
