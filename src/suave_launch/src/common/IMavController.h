#ifndef IFLIGHTPLAN_H
#define IFLIGHTPLAN_H
#include <future>
#include <system.h>

#include <utility>

#include "ITask.h"

class IMavController : public ITask
{
public:
    explicit IMavController(std::shared_ptr<mavsdk::System> system) : m_system(std::move(system))
    {
    }

    [[nodiscard]]
    mavsdk::System* get_system() const { return m_system.get(); }

private:
    std::shared_ptr<mavsdk::System> m_system;
};

#endif //IFLIGHTPLAN_H
