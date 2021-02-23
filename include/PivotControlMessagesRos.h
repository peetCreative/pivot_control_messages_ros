//
// Created by peetcreative on 21.02.21.
//

#ifndef SRC_PIVOTCONTROLMESSAGESROS_H
#define SRC_PIVOTCONTROLMESSAGESROS_H

#include "pivot_control_messages_ros/LaparoscopeDOFPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFBoundaries.h"

#include "PivotControlMessages.h"

namespace pivot_control_messages_ros
{
    pivot_control_messages::DOFPose toDOFPose(
            pivot_control_messages_ros::LaparoscopeDOFPose);
    pivot_control_messages_ros::LaparoscopeDOFPose toROSDOFPose(
            pivot_control_messages::DOFPose, std::string, int);
    pivot_control_messages::DOFBoundaries toDOFBoundaries(
            pivot_control_messages_ros::LaparoscopeDOFBoundaries);
    pivot_control_messages_ros::LaparoscopeDOFBoundaries toROSDOFBoundaries(
            pivot_control_messages::DOFBoundaries, std::string, int);
}

#endif //SRC_PIVOTCONTROLMESSAGESROS_H
