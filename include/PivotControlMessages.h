#ifndef PIVOT_CONTROL_MESSAGES_H
#define PIVOT_CONTROL_MESSAGES_H

#include <sstream>

namespace pivot_control_messages
{
    struct DOFPose
    {
        float pitch = 0;
        float yaw = 0;
        float roll = 0;
        float transZ = 0;
        std::string toString()
        {
            std::stringstream ss;
            ss << "pitch:" << pitch
               << " yaw:" << yaw
               << " roll" << roll
               << " transZ" << transZ;
            return ss.str();
        }
    };
    struct DOFBoundaries
    {
        float pitchMax = 0;
        float pitchMin = 0;
        float yawMax = 0;
        float yawMin = 0;
        float rollMax = 0;
        float rollMin = 0;
        float transZMax = 0;
        float transZMin = 0;
    };
}

#endif //PIVOT_CONTROL_MESSAGES_H