//
// Created by suave on 4/12/24.
//

#ifndef SUAVETASKMANAGER_H
#define SUAVETASKMANAGER_H
#include <connection_result.h>
#include <mavsdk.h>
#include <thread>

#include "../common/IMavController.h"
#include "../common/ISuaveController.h"


class SuaveTaskManager : public ISuaveController {
public:
    SuaveTaskManager() :
    m_mavsdk(mavsdk::Mavsdk{}),
    m_system(nullptr)
    {
        // Connect to the PX4 SITL.
        mavsdk::ConnectionResult connection_result = m_mavsdk.add_any_connection("udp://:14540");
        if (connection_result != mavsdk::ConnectionResult::Success)
        {
            throw std::runtime_error((std::stringstream() << "Connection failed: " << connection_result << '\n').str());
        }

        // Wait until the vehicle is ready.
        while (m_mavsdk.systems().empty())
        {
            std::cout << "Waiting for system to connect...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        m_system = *m_mavsdk.systems().begin();
        std::cout << "System connected\n";
    }

    void start() override;

private:
    mavsdk::Mavsdk m_mavsdk;
    std::shared_ptr<mavsdk::System> m_system;
    std::vector<IMavController*> m_controllers{};
};

#endif //SUAVETASKMANAGER_H
