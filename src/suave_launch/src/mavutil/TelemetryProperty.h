//
// Created by suave on 4/18/24.
//

#ifndef TELEMETRYPROPERTY_H
#define TELEMETRYPROPERTY_H
#include <optional>
#include <thread>
#include <plugins/telemetry/telemetry.h>

#include "../util/Timer.h"
#include "../util/Result.h"

template<typename T>
class TelemetryProperty {
public:
    using TCallback = std::function<void(T)>;
    using SubscribeFn = void(mavsdk::Telemetry::*)(TCallback);
    TelemetryProperty(mavsdk::Telemetry& telemetry, SubscribeFn subscribe_fn, std::optional<TCallback> callback = std::nullopt):
        m_callback{callback}
    {
        (telemetry.*subscribe_fn)(std::bind(&TelemetryProperty::set, this, std::placeholders::_1));
    }

    void set_callback(TCallback callback)
    {
        if (m_callback)
        {
            suave_err << "Overriding TelemetryProperty::m_callback!";
        }
        m_callback = callback;
    }

    [[nodiscard]]
    Result<T> get() const
    {
        if (!m_value)
        {
            return Result<T>("TelemetryProperty::get(), does not have the value yet!");
        }
        return *m_value;
    }

    Result<T> get_within(double elapse_sec = 1.0) const
    {
        if (m_timer_since_last_update.has_been(elapse_sec))
        {
            return Result<T>("TelemetryProperty::get_within(), does not have the value within " + std::to_string(elapse_sec) + " seconds! Returning the last value.");
        }
        return get();
    }

    Result<T> wait_for_next(const double timeout_sec = 60, const double refresh_rate_hz = 1) const
    {
        Timer timer{};
        timer.start();
        while (m_timer_since_last_update.has_been(1/refresh_rate_hz))
        {
            if (timer.has_been(timeout_sec))
            {
                return Result<T>("TelemetryProperty::wait_for_next(), timeout after " + std::to_string(timeout_sec) + " seconds!");
            }
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(1/refresh_rate_hz * 1e6 / 10)));
        }
        return get();
    }

private:

    void set(T value)
    {
        m_value = value;
        m_timer_since_last_update.start();
        if (m_callback.has_value())
        {
            m_callback.value()(m_value.value());
        }
    }

private:
    Timer m_timer_since_last_update{};
    std::optional<T> m_value{};
    std::optional<TCallback> m_callback{};
};

#endif //TELEMETRYPROPERTY_H
