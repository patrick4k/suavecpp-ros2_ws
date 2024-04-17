//
// Created by suave on 4/17/24.
//

#ifndef ROSNODEEXECUTOR_H
#define ROSNODEEXECUTOR_H
#include <rclcpp/node.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "../common/ITask.h"


class RosNodeSpinner: public ITask {
public:
    TaskResult start() override;
    void stop() override;

    void add_node(rclcpp::Node::SharedPtr node) const
    {
        m_executor->add_node(std::move(node));
    }

private:
    using Executor = rclcpp::executors::MultiThreadedExecutor;
    std::shared_ptr<Executor> m_executor =  std::make_shared<Executor>();
};

#endif //ROSNODEEXECUTOR_H
