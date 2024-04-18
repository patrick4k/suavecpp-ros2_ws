#include <rclcpp/utilities.hpp>

#include "controllers/SuaveVIOTestFlight.h"

int main(int argc, char **argv)
{
    suave_log << "Starting suave_main" << std::endl;
    rclcpp::init(argc, argv);
    SuaveVIOTestFlight controller{};
    controller.start();
    rclcpp::shutdown();
    suave_log << "Exiting suave_main" << std::endl;
}
