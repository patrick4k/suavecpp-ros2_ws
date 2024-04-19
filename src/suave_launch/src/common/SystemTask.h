//
// Created by suave on 4/18/24.
//

#ifndef SYSTEMTASK_H
#define SYSTEMTASK_H

#include <sys/wait.h>
#include <utility>
#include <vector>

#include "ITask.h"

class SystemTask : public ITask
{
public:
    explicit SystemTask(std::vector<std::string> commands, bool should_print_output = false) :
        m_commands(std::move(commands)),
        m_should_print_output(should_print_output)
    {
    }

    TaskResult start() override;

    void stop() override;

    // Warning: This function will always return nullptr
    std::future<TaskResult>* start_in_thread() override;

private:
    pid_t m_pid = -1;
    std::vector<std::string> m_commands;
    bool m_should_print_output;
};

#endif //SYSTEMTASK_H
