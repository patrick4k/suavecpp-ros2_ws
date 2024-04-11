#ifndef MAVVIOBRIDGE_H
#define MAVVIOBRIDGE_H

#include <rclcpp/node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "../common/IMavController.h"

namespace mavsdk
{
    class System;
}

class MavVIOBridge : public IMavController
{
public:
    explicit MavVIOBridge(std::shared_ptr<mavsdk::System> system);

    MavControllerResult start() override;
    void stop() override;

private:
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    std::shared_ptr<rclcpp::Node> m_node;
    std::optional<std::future<void>> m_spin_future{};
};

#endif //MAVVIOBRIDGE_H
