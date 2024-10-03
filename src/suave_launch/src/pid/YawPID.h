#pragma once

#include "PID.h"

class YawPID : public PID
{
public:

    YawPID(double Kp, double Ki, double Kd, double setpoint = 0) : PID(Kp, Ki, Kd, setpoint)
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

        return error;

    }
};
