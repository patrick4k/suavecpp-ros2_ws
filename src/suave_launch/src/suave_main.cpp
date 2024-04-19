#include <csignal>
#include <rclcpp/utilities.hpp>

#include "controllers/SuaveVIOTestFlight.h"

SuaveVIOTestFlight s_controller{};

void signal_handler(int sig)
{
    suave_warn << "Signal received: " << sig << std::endl;
    s_controller.shutdown();
    rclcpp::shutdown();
    suave_log << "Exiting suave_main" << std::endl;
    exit(0);
}

int main(int argc, char **argv)
{
    suave_log << "Starting suave_main" << std::endl;
    rclcpp::init(argc, argv);
    std::vector signals = {SIGINT, SIGTERM, SIGQUIT};
    for (const auto sig : signals)
    {
        std::signal(sig, signal_handler);
    }
    s_controller.start();
    rclcpp::shutdown();
    suave_log << "Exiting suave_main" << std::endl;
}
