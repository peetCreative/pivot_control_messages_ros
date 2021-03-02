//
// Created by peetcreative on 21.02.21.
//

#ifndef SRC_PIVOTCONTROLMESSAGESROS_H
#define SRC_PIVOTCONTROLMESSAGESROS_H

#include "pivot_control_messages_ros/SetPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFBoundaries.h"

#include "PivotControlMessages.h"

namespace pivot_control_messages_ros
{
    pivot_control_messages::DOFPose toDOFPose(LaparoscopeDOFPose);
    pivot_control_messages::DOFPose toDOFPose(SetPose::Request);
    LaparoscopeDOFPose toROSDOFPose(
            pivot_control_messages::DOFPose, std::string, int);
    LaparoscopeDOFPose toROSDOFPose(SetPose::Request, std::string, int);
    pivot_control_messages::DOFBoundaries toDOFBoundaries(
            LaparoscopeDOFBoundaries);
    LaparoscopeDOFBoundaries toROSDOFBoundaries(
            pivot_control_messages::DOFBoundaries, std::string, int);
}

#endif //SRC_PIVOTCONTROLMESSAGESROS_H
