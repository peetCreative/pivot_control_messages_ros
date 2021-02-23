//
// Created by peetcreative on 21.02.21.
//
#include "ros/time.h"
#include "PivotControlMessagesRos.h"

namespace pivot_control_messages_ros
{
    pivot_control_messages::DOFPose toDOFPose(
            pivot_control_messages_ros::LaparoscopeDOFPose poseROS)
    {
        pivot_control_messages::DOFPose pose;
        pose.pitch = poseROS.pitch;
        pose.yaw = poseROS.yaw;
        pose.roll = poseROS.roll;
        pose.transZ = poseROS.trans_z;
        return pose;
    }

    pivot_control_messages_ros::LaparoscopeDOFPose toROSDOFPose(
            pivot_control_messages::DOFPose pose,
            std::string frameId = "", int sequence = 0)
    {
        pivot_control_messages_ros::LaparoscopeDOFPose poseROS;
        poseROS.header.stamp = ros::Time::now();
        poseROS.header.seq = sequence;
        poseROS.header.frame_id = frameId;
        poseROS.pitch = pose.pitch;
        poseROS.yaw = pose.yaw;
        poseROS.roll = pose.roll;
        poseROS.trans_z = pose.transZ;
        return poseROS;
    }

    pivot_control_messages::DOFBoundaries toDOFBoundaries(
            pivot_control_messages_ros::LaparoscopeDOFBoundaries boundariesROS)
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

    pivot_control_messages_ros::LaparoscopeDOFBoundaries toROSDOFBoundaries(
            pivot_control_messages::DOFBoundaries boundaries,
            std::string frameId, int sequence)
    {
        pivot_control_messages_ros::LaparoscopeDOFBoundaries boundariesROS;
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
