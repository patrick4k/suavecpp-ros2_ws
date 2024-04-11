#ifndef MAVVIOBRIDGE_H
#define MAVVIOBRIDGE_H

#include "../common/IMavController.h"

namespace mavsdk
{
    class System;
}

class MavVIOBridge : public IMavController
{
public:
    explicit MavVIOBridge(std::shared_ptr<mavsdk::System>& system) : IMavController(system)
    {
    }

    MavControllerResult start() override;
};

#endif //MAVVIOBRIDGE_H
