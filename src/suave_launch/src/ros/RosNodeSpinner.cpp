//
// Created by suave on 4/17/24.
//

#include "RosNodeSpinner.h"

TaskResult RosNodeSpinner::start()
{
    m_executor->spin();
    return TaskResult::SUCCESS;
}

void RosNodeSpinner::stop()
{
    m_executor->cancel();
}
