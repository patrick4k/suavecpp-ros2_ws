#pragma once

#include <optional>
#include <chrono>
#include <algorithm>

class PID
{
public:
    PID(double Kp, double Ki, double Kd, double setpoint = 0) :
        m_Kp{ Kp },
        m_Ki{ Ki },
        m_Kd{ Kd },
        m_setpoint{ setpoint }
    {
    }

    virtual double compute_error(double setpoint, double input)
    {
        return setpoint - input;
    }

    double operator()(double input)
    {
        double error = compute_error(m_setpoint, input);
        
        double P = m_Kp * error;
        double I = 0;
        double D = 0;

        const auto now = SteadyClock::now();        
        if (m_prevTime)
        {
            const auto dt = std::chrono::duration<double>(now - *m_prevTime).count();
            m_integral += error * dt;

            I = m_Ki * m_integral;

            double derivative = (error - m_prevError) / dt;
            D = m_Kd * derivative;
        }

        m_prevTime = now;
        m_prevError = error;

        auto output = P + I + D;
        return std::clamp(output, -100.0, 100.0);
    }

private:
    using SteadyClock = std::chrono::steady_clock;
    using Time = SteadyClock::time_point;

    double m_Kp;
    double m_Ki;
    double m_Kd;
    double m_setpoint;

    double m_integral{ 0 };
    std::optional<Time> m_prevTime{};
    double m_prevError{ 0 };
};
