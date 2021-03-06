//
// Created by peetcreative on 21.02.21.
//
#include "PivotControlMessagesRos.h"

#include "PivotControlMessages.h"
#include "ros/time.h"

namespace pivot_control_messages_ros
{
    pivot_control_messages::DOFPose toDOFPose(LaparoscopeDOFPose poseROS)
    {
        pivot_control_messages::DOFPose pose;
        pose.pitch = poseROS.pitch;
        pose.yaw = poseROS.yaw;
        pose.roll = poseROS.roll;
        pose.transZ = poseROS.trans_z;
        return pose;
    }

    pivot_control_messages::DOFPose toDOFPose(SetPose::Request poseReqROS)
    {
        pivot_control_messages::DOFPose pose;
        pose.pitch = poseReqROS.pitch;
        pose.yaw = poseReqROS.yaw;
        pose.roll = poseReqROS.roll;
        pose.transZ = poseReqROS.trans_z;
        return pose;
    }

    LaparoscopeDOFPose toROSDOFPose(
            pivot_control_messages::DOFPose pose,
            std::string frameId = "", int sequence = 0)
    {
        LaparoscopeDOFPose poseROS;
        poseROS.header.stamp = ros::Time::now();
        poseROS.header.seq = sequence;
        poseROS.header.frame_id = frameId;
        poseROS.pitch = pose.pitch;
        poseROS.yaw = pose.yaw;
        poseROS.roll = pose.roll;
        poseROS.trans_z = pose.transZ;
        return poseROS;
    }

    LaparoscopeDOFPose toROSDOFPose(
            SetPose::Request pose,
            std::string frameId = "", int sequence = 0)
    {
        LaparoscopeDOFPose poseROS;
        poseROS.header.stamp = ros::Time::now();
        poseROS.header.seq = sequence;
        poseROS.header.frame_id = frameId;
        poseROS.pitch = pose.pitch;
        poseROS.yaw = pose.yaw;
        poseROS.roll = pose.roll;
        poseROS.trans_z = pose.trans_z;
        return poseROS;

    }

    pivot_control_messages::DOFBoundaries toDOFBoundaries(
            LaparoscopeDOFBoundaries boundariesROS)
    {
        pivot_control_messages::DOFBoundaries boundaries;
        boundaries.pitchMax = boundariesROS.pitch_max;
        boundaries.pitchMin = boundariesROS.pitch_min;
        boundaries.yawMax = boundariesROS.yaw_max;
        boundaries.yawMin = boundariesROS.yaw_min;
        boundaries.rollMax = boundariesROS.roll_max;
        boundaries.rollMin = boundariesROS.roll_min;
        boundaries.transZMax = boundariesROS.trans_z_max;
        boundaries.transZMin = boundariesROS.trans_z_min;
        return boundaries;
    }

    LaparoscopeDOFBoundaries toROSDOFBoundaries(
            pivot_control_messages::DOFBoundaries boundaries,
            std::string frameId, int sequence)
    {
        LaparoscopeDOFBoundaries boundariesROS;
        boundariesROS.header.stamp = ros::Time::now();
        boundariesROS.header.seq = sequence;
        boundariesROS.header.frame_id = frameId;
        boundariesROS.pitch_max = boundaries.pitchMax;
        boundariesROS.pitch_min = boundaries.pitchMin;
        boundariesROS.yaw_max = boundaries.yawMax;
        boundariesROS.yaw_min = boundaries.yawMin;
        boundariesROS.roll_max = boundaries.rollMax;
        boundariesROS.roll_min = boundaries.rollMin;
        boundariesROS.trans_z_max = boundaries.transZMax;
        boundariesROS.trans_z_min = boundaries.transZMin;
        return boundariesROS;
    }

}
