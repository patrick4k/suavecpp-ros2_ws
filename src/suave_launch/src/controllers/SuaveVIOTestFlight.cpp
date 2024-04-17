//
// Created by suave on 4/17/24.
//

#include "SuaveVIOTestFlight.h"

#include "../ros/RosNodeSpinner.h"
#include "../vio/VIOBridge.h"

void SuaveVIOTestFlight::start()
{
    RosNodeSpinner spinner{};
    std::shared_ptr<VIOBridge> vio_bridge = std::make_shared<VIOBridge>(m_drone.get_system());
    spinner.add_node(vio_bridge);
    auto spinner_result = spinner.start_in_thread();

    // Do offboard stuff here

    if (spinner_result->get() != TaskResult::SUCCESS)
    {
        suave_err << "RosNodeSpinner failed" << std::endl;
    }

    spinner.stop();

}
