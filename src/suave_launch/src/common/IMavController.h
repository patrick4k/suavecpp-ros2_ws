#ifndef IFLIGHTPLAN_H
#define IFLIGHTPLAN_H
#include <future>
#include <optional>
#include <system.h>
#include <thread>

#include <utility>

#include "MavControllerResult.h"
#include "../util/Util.h"

class IMavController
{
public:
    explicit IMavController(std::shared_ptr<mavsdk::System> system) : m_system(std::move(system))
    {
    }

    virtual ~IMavController() = default;

    virtual MavControllerResult start()
    {
        suave_warn << "IMavController::start() not implemented" << std::endl;
        return MavControllerResult::NOT_IMPLMENTED;
    }

    std::future<MavControllerResult>* start_in_thread()
    {
        m_async_future = std::async(std::launch::async, &IMavController::start, this);
        return &*m_async_future;
    }

    virtual void stop()
    {
        suave_warn << "IMavController::stop() not implemented" << std::endl;
    }

    [[nodiscard]]
    mavsdk::System* get_system() const { return m_system.get(); }

private:
    std::optional<std::future<MavControllerResult>> m_async_future{};
    std::shared_ptr<mavsdk::System> m_system;
};

#endif //IFLIGHTPLAN_H
