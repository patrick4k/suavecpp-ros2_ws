//
// Created by patrick on 5/11/24.
//

#ifndef MAVLINK_H
#define MAVLINK_H
#include <mavsdk.h>

static mavsdk::Mavsdk s_mavsdk;

class Mavlink {
public:
    Mavlink() = default;

    static std::shared_ptr<mavsdk::System> connectToPX4SITL(uint32_t port = mavsdk::Mavsdk::DEFAULT_UDP_PORT);

    static std::shared_ptr<mavsdk::System> connectToSerial();
};

#endif //MAVLINK_H
