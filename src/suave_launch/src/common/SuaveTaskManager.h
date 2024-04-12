//
// Created by suave on 4/12/24.
//

#ifndef SUAVETASKMANAGER_H
#define SUAVETASKMANAGER_H
#include <connection_result.h>

#include "IMavController.h"
#include "ISuaveController.h"


class SuaveTaskManager : public ISuaveController {
    explicit SuaveTaskManager(std::shared_ptr<mavsdk::System> system) : m_system(std::move(system))
    {
    }

public:
    static std::optional<SuaveTaskManager> create();


    void start() override;

private:

    std::shared_ptr<mavsdk::System> m_system;
    std::vector<IMavController*> m_controllers{};
};

#endif //SUAVETASKMANAGER_H
