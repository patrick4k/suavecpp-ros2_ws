#ifndef IFLIGHTPLAN_H
#define IFLIGHTPLAN_H
#include <system.h>

#include "SystemControllerResult.h"


class ISystemController
{
public:
    virtual ~ISystemController() = default;

    void set_system(const std::shared_ptr<mavsdk::System>& system) { m_system = system; }

    virtual SystemControllerResult start()
    {
        return SystemControllerResult::NOT_IMPLMENTED;
    }

protected:
    std::shared_ptr<mavsdk::System> m_system;
};

#endif //IFLIGHTPLAN_H
