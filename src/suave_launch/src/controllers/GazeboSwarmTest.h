//
// Created by patrick on 5/11/24.
//

#ifndef GAZEBOSWARMTEST_H
#define GAZEBOSWARMTEST_H
#include "../common/ISuaveController.h"
#include "../mavutil/Drone.h"
#include "../mavutil/Mavlink.h"
#include "../mavutil/MavUtil.h"

class GazeboSwarmTest: public ISuaveController {
public:
    GazeboSwarmTest() :
        ISuaveController()
    {
    }

    void start() override;
    void shutdown() override;

private:
    static void endtask();
    bool m_end_controller{false};
};


#endif //GAZEBOSWARMTEST_H
