#ifndef ROSSUBSCRIBER_H
#define ROSSUBSCRIBER_H
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/executors.hpp>

#include "IRosController.h"
using std::placeholders::_1;

template<typename Msg>
class RosSubscriber : public IRosController, public rclcpp::Node {
public:
    using MsgSharedPtr = typename Msg::SharedPtr;
    RosSubscriber(std::shared_ptr<mavsdk::System> system, const std::string& node_name, const std::string& topic) :
    IRosController(std::move(system)),
    rclcpp::Node(node_name),
    m_executor{std::make_shared<rclcpp::executors::SingleThreadedExecutor>()}
    {
        m_subscription = this->create_subscription<Msg>(
            topic,
            10,
            std::bind(&RosSubscriber::callback, this, _1)
        );
    }


    virtual void callback(const MsgSharedPtr msg) const = 0;

    MavControllerResult start() override
    {
        suave_log << "Starting ROS subscriber" << std::endl;
        rclcpp::spin(this->get_node_base_interface());
        return MavControllerResult::SUCCESS;
    }

    void stop() override
    {
        suave_log << "Stopping ROS subscriber" << std::endl;
        m_executor->cancel();
        if (m_spin_future->valid())
        {
            m_spin_future->get();
        }
    }

protected:
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    std::shared_ptr<rclcpp::Node> m_node;
    typename rclcpp::Subscription<Msg>::SharedPtr m_subscription;
    std::optional<std::future<void>> m_spin_future{};
};

#endif //ROSSUBSCRIBER_H
