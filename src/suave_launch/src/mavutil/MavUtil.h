//
// Created by suave on 4/17/24.
//

#ifndef MAVUTIL_H
#define MAVUTIL_H
#include <mavsdk.h>
#include <memory>

std::shared_ptr<mavsdk::System> connectToPX4SITL(mavsdk::Mavsdk& m_mavsdk);

#endif //MAVUTIL_H
