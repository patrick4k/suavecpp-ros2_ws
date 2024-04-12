//
// Created by suave on 4/12/24.
//

#ifndef VIOBRIDGE_H
#define VIOBRIDGE_H
#include <nav_msgs/msg/odometry.hpp>
#include <plugins/mocap/mocap.h>
#include <plugins/telemetry/telemetry.h>

#include "../ros/RosSubscriber.h"

using namespace mavsdk;
using OdomMsg = nav_msgs::msg::Odometry;

class VIOBridge: public RosSubscriber<OdomMsg> {
public:
    explicit VIOBridge(std::shared_ptr<System> system) :
    RosSubscriber(std::move(system), "rtabmaps_listener", "/odom")
    {
        const Telemetry telem{*this->get_system()};
        m_heading = telem.heading().heading_deg * M_PI / 180.0;
    }

private:
    double m_heading;

    struct MocapMessages
    {
        Mocap::VisionPositionEstimate vision_position_estimate;
        Mocap::Odometry odometry;
    };

    static MocapMessages RtabOdom2MocapMessage(const OdomMsg::SharedPtr msg, const double& gam);

public:
    void callback(const OdomMsg::SharedPtr msg) const override;

};

#endif //VIOBRIDGE_H
