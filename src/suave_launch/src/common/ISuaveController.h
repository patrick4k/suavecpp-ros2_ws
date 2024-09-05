#ifndef ISUAVECONTROLLER_H
#define ISUAVECONTROLLER_H

#include <mavsdk/mavsdk.h>

class ISuaveController
{
public:
    ISuaveController() = default;

    virtual ~ISuaveController() = default;

    virtual void start() = 0;
    virtual void shutdown() = 0;
};

#endif //ISUAVECONTROLLER_H
