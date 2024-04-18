//
// Created by suave on 4/18/24.
//

#ifndef SYSTEMTASK_H
#define SYSTEMTASK_H

#include <sys/wait.h>
#include <csignal>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <utility>
#include <vector>

#include "ITask.h"

class SystemTask : public ITask
{
public:
    explicit SystemTask(std::vector<std::string> commands) : m_commands(std::move(commands)) {}

    TaskResult start() override;

    void stop() override;

    // ALWAYS RETURNS NULLPTR
    std::future<TaskResult>* start_in_thread() override;

private:
    TaskResult m_result = TaskResult::INVALID;
    std::vector<std::string> m_commands;
    pid_t m_pid = -1;
};

#endif //SYSTEMTASK_H
