//
// Created by suave on 4/17/24.
//

#ifndef TIMER_H
#define TIMER_H
#include <chrono>
#include <optional>
#include <stdexcept>

class Timer
{
public:
    Timer() = default;

    void start()
    {
        m_start = std::chrono::system_clock::now();
    }

    void reset()
    {
        m_start.reset();
    }

    [[nodiscard]]
    bool is_running() const
    {
        return m_start.has_value();
    }

    [[nodiscard]]
    uint32_t elapse_us() const
    {
        if (m_start)
        {
            return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - m_start.value()).count();
        }
        throw std::runtime_error("Timer not started yet!");
    }

    [[nodiscard]]
    double elapse_sec() const
    {
        return elapse_us() * 1e-6;
    }

    [[nodiscard]]
    bool has_been(double elapse_sec) const
    {
        return this->elapse_sec() >= elapse_sec;
    }


private:
    std::optional<std::chrono::time_point<std::chrono::system_clock>> m_start{};

};

#endif //TIMER_H
