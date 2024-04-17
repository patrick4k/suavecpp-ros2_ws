#include <rclcpp/utilities.hpp>

#include "controllers/SuaveTaskManager.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    SuaveTaskManager controller{};
    controller.start();
    rclcpp::shutdown();
}
