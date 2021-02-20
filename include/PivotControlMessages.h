#ifndef PIVOT_CONTROL_MESSAGES_H
#define PIVOT_CONTROL_MESSAGES_H

namespace pivot_control_messages
{
    struct {
        float yaw = 0;
        float pitch = 0;
        float roll = 0;
        float transZ = 0;
    } DOFPose;

    struct {
        float yawMax, yawMin;
        float pitchMax, pitchMin;
        float rollMax, rollMin;
        float transZMax, transZMin;
    } DOFBoundaries;
}

#endif //PIVOT_CONTROL_MESSAGES_H