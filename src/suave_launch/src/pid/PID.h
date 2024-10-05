#pragma once

#include <optional>
#include <chrono>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <fstream>

class PID
{
public:
    PID(double Kp, double Ki, double Kd, double setpoint = 0, bool enableLogging = false):
        m_Kp{ Kp },
        m_Ki{ Ki },
        m_Kd{ Kd },
        m_setpoint{ setpoint },
        m_enableLogging{ enableLogging }
    {
    }

    virtual double compute_error(double setpoint, double input)
    {
        return setpoint - input;
    }

    double call(double input)
    {
        return this->operator()(input);
    }

    double operator()(double input)
    {
        double error = compute_error(m_setpoint, input);
        
        double P = m_Kp * error;
        double I = 0;
        double D = 0;

        const auto now = SteadyClock::now();        
        
        if (!m_startTime)
        {
            m_startTime = now;
            m_csv_ss << "time,setpoint,input,error,output\n";
        }

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
        output = std::clamp(output, -100.0, 100.0);

        if (m_enableLogging)
        {
            log(now, input, error, output);
        }

        return output;
    }

    void export_csv()
    {
        std::ofstream csvFile{};
        csvFile.open("/home/suave/Data/SuaveMaskingPid/yaw_latest.csv", std::ofstream::out | std::ofstream::trunc);
        if (csvFile.is_open()) 
        {
            csvFile << m_csv_ss.str();
            csvFile.close();
            std::cout << "CSV file written successfully.\n";
        } else 
        {
            std::cerr << "Error opening file.\n";
        }
    }

private:
    using SteadyClock = std::chrono::steady_clock;
    using Time = SteadyClock::time_point;

    double m_Kp;
    double m_Ki;
    double m_Kd;
    double m_setpoint;

    double m_integral{ 0 };
    std::optional<Time> m_startTime{};
    std::optional<Time> m_prevTime{};
    double m_prevError{ 0 };

    bool m_enableLogging{ false };
    std::stringstream m_csv_ss{};

    void log(Time now, double input, double error, double output)
    {
        auto t = std::chrono::duration<double>(now - *m_startTime).count();
        m_csv_ss << t << "," << m_setpoint << "," << input << "," << error << "," << output << "\n";
    }
};
