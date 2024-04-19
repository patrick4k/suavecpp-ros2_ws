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

    [[nodiscard]]
    Result<T> get() const
    {
        if (!m_value)
        {
            return Result<T>("TelemetryProperty::get(), does not have the value yet!");
        }
        return *m_value;
    }

    Result<T> get_within(double elapse_sec = 1.0)
    {
        if (m_timer.has_been(elapse_sec))
        {
            return "TelemetryProperty::get_within(), does not have the value within " + std::to_string(elapse_sec) + " seconds! Returning the last value.";
        }
        return get();
    }

    Result<T> wait_for_next(double refresh_rate_hz = 1.0)
    {
        while (m_timer.has_been(refresh_rate_hz))
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(1/refresh_rate_hz * 1e6 / 10)));
        }
        return get();
    }

    void set(T value)
    {
        m_value = value;
        m_timer.start();
        if (m_callback.has_value())
        {
            m_callback.value()(m_value.value());
        }
    }

private:
    Timer m_timer{};
    std::optional<T> m_value{};
    std::optional<TCallback> m_callback{};
};

#endif //TELEMETRYPROPERTY_H
