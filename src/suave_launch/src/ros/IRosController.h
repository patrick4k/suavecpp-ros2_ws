//
// Created by suave on 4/11/24.
//

#ifndef IROSSUBSCRIBER_H
#define IROSSUBSCRIBER_H
#include "../common/IMavController.h"

class IRosController: public IMavController
{
public:
    explicit IRosController(std::shared_ptr<mavsdk::System> system) : IMavController(std::move(system))
    {
    }

};

#endif //IROSSUBSCRIBER_H
