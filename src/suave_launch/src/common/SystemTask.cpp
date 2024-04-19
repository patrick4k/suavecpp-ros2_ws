//
// Created by suave on 4/18/24.
//

#include "SystemTask.h"

#include <cstring>

TaskResult SystemTask::start()
{
    std::string command = "bash -c '";
    for (const auto& subcommand: m_commands)
    {
        if (subcommand == m_commands.back()) command += subcommand + (m_should_print_output? "'" : " > /dev/null 2>&1'");
        else command += subcommand + "&&";
    }
    suave_log << "Executing command: " << command << std::endl;
    execl("/bin/bash", "bash", "-c", command.c_str(), static_cast<char*>(nullptr));
    return TaskResult::SUCCESS;
}

void SystemTask::stop()
{
    if (m_pid > 0) {
        suave_log << "Stopping process: " << m_pid << std::endl;

        // Send SIGKILL signal to the child process
        int killResult = kill(m_pid, SIGTERM);
        if (killResult == -1) {
            suave_err << "Failed to send SIGKILL signal to process " << m_pid << ": " << strerror(errno) << std::endl;
            return;
        }

        suave_log << "Waiting for process: " << m_pid << " to terminate" << std::endl;

        // Wait for the child process to terminate
        int status;
        int waitResult = waitpid(m_pid, &status, 0);
        if (waitResult == -1) {
            suave_err << "Failed to wait for process termination: " << strerror(errno) << std::endl;
            return;
        }

        if (WIFSIGNALED(status)) {
            suave_log << "Process: " << m_pid << " terminated by signal " << WTERMSIG(status) << std::endl;
        } else {
            suave_log << "Process: " << m_pid << " terminated" << std::endl;
        }

        // Reset PID after process termination
        m_pid = -1;
    }
}

std::future<TaskResult>* SystemTask::start_in_thread()
{
    if (m_pid > 0)
    {
        suave_err << "Process already running" << std::endl;
        return nullptr;
    }

    pid_t pid = fork();

    if (pid == 0) {
        auto result = this->start();
        exit(result == TaskResult::SUCCESS ? 0 : 1);
    }
    if (pid < 0) {
        suave_err << "Failed to fork process" << std::endl;
        return nullptr;
    }

    suave_log << "Setting PID to: " << pid << std::endl;
    m_pid = pid;
    return nullptr;
}
