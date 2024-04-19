//
// Created by suave on 4/15/24.
//

#ifndef RESULT_H
#define RESULT_H
#include <optional>
#include <string>
#include "Util.h"

template<typename TOk, typename TErr = std::string>
class Result
{
public:
    Result(TOk ok) : m_ok(ok) {}
    Result(TErr err) : m_err(err) {}

    [[nodiscard]]
    bool is_ok() const { return m_ok.has_value(); }

    [[nodiscard]]
    bool is_err() const { return m_err.has_value(); }

    explicit operator bool() const { return is_ok(); }

    [[nodiscard]]
    TOk unwrap() const
    {
        if (!m_ok)
        {
            if (m_err)
                suave_err << m_err.value() << std::endl;
            else
                suave_err << "Unknown error on Result::unwrap()" << std::endl;
        }
        return m_ok.value();
    }

    [[nodiscard]]
    TErr unwrap_err() const { return m_err.value(); }

    void ok(TOk ok)
    {
        m_err.reset();
        m_ok = ok;
    }
    void err(TErr err)
    {
        m_ok.reset();
        m_err = err;
    }

private:
    std::optional<TOk> m_ok{};
    std::optional<TErr> m_err{};
};

#endif //RESULT_H
