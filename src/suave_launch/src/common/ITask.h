//
// Created by patrick on 4/17/24.
//

#ifndef SUAVE_LAUNCH_ITASK_H
#define SUAVE_LAUNCH_ITASK_H

#include <future>
#include <optional>

#include "TaskResult.h"
#include "../util/Util.h"

class ITask
{
public:
    virtual ~ITask() = default;

    virtual TaskResult start()
    {
        suave_warn << "ITask::start() not implemented" << std::endl;
        return TaskResult::NOT_IMPLMENTED;
    }

    virtual std::future<TaskResult>* start_in_thread()
    {
        m_async_future = std::async(std::launch::async, &ITask::start, this);
        return &*m_async_future;
    }

    virtual void stop()
    {
        suave_warn << "ITask::stop() not implemented" << std::endl;
    }

private:
    std::optional<std::future<TaskResult>> m_async_future{};
};

#endif //SUAVE_LAUNCH_ITASK_H
