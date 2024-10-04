#pragma once

#include "PID.h"
#include <iostream>

class YawPID : public PID
{
public:

    YawPID(double Kp, double Ki, double Kd, double setpoint = 0, bool enableLogging = false) : PID(Kp, Ki, Kd, setpoint, enableLogging)
    {
    }

    double compute_error(double setpoint, double input) override
    {
        auto error = setpoint - input;
        if (error > 180)
        {
            error = error - 360;
        }
        else if (error < -180)
        {
            error = error + 360;
        }

        std::cout << "Setpoint = " << setpoint << "\nInput = " << input << "\nError = " << error << std::endl;

        return error;
    }
};
