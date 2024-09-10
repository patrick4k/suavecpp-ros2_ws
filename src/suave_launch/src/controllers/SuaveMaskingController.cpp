//
// Created by patrick on 9/5/24.
//

#include "SuaveMaskingController.h"

#include "../masking_pid/MaskingSubscriber.h"
#include "../ros/RosNodeSpinner.h"

#define sleep(sec) std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(sec * 1e6)));

void SuaveMaskingController::start() {
    RosNodeSpinner spinner{};
    auto masking_subscriber = std::make_shared<MaskingSubscriber>(m_drone.get());
    spinner.add_node(masking_subscriber);

    suave_log << "Starting spinner" << std::endl;

    spinner.start_in_thread();

    sleep(10)

    suave_log << "Stopping spinner" << std::endl;

    spinner.stop();

    suave_log << "Exiting..." << std::endl;

}

void SuaveMaskingController::shutdown() {
}
