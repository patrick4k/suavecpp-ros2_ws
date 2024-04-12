#ifndef ROSSUBSCRIBER_H
#define ROSSUBSCRIBER_H
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "IRosController.h"

template<typename Msg>
class RosSubscriber : public IRosController {
public:
    RosSubscriber(std::shared_ptr<mavsdk::System> system, const std::string& topic) : IRosController(std::move(system)),
    m_executor{std::make_shared<rclcpp::executors::SingleThreadedExecutor>()},
    m_node{std::make_shared<rclcpp::Node>(topic)}
    {
        m_node->create_subscription<Msg>(
            topic,
            10,
            [this](typename Msg::SharedPtr msg)->void
            {
                this->callback(msg);
            }
        );
    }

    virtual void callback(typename Msg::SharedPtr msg) = 0;

    MavControllerResult start() override
    {
        m_spin_future = std::async(
            std::launch::async,
            [this]() -> void
            {
                m_executor->add_node(m_node);
                m_executor->spin();
            }
        );

        return MavControllerResult::SUCCESS;
    }

    void stop() override
    {
        m_executor->cancel();
        if (m_spin_future->valid())
        {
            m_spin_future->get();
        }
    }

protected:
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    std::shared_ptr<rclcpp::Node> m_node;
    std::optional<std::future<void>> m_spin_future{};
};

#endif //ROSSUBSCRIBER_H
