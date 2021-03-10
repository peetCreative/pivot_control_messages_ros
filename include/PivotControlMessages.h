#ifndef PIVOT_CONTROL_MESSAGES_H
#define PIVOT_CONTROL_MESSAGES_H

#include <sstream>
#include <math.h>

namespace pivot_control_messages
{
    struct DOFPose
    {
        double pitch = 0;
        double yaw = 0;
        double roll = 0;
        double transZ = 0;
        std::string toString()
        {
            std::stringstream ss;
            ss << "pitch:" << pitch
               << " yaw:" << yaw
               << " roll" << roll
               << " transZ" << transZ;
            return ss.str();
        }
        bool operator==(const DOFPose& other)
        {
            return pitch == other.pitch &&
                   yaw == other.yaw &&
                   roll == other.roll &&
                   transZ == other.transZ;
        }
        //TODO: bring this to Eigen
        bool closeTo(DOFPose &other, double rotEpsilon, double transZEpsilon)
        {
            double diffPitch = pitch - other.pitch;
            double diffYaw = yaw - other.yaw;
            double diffRoll = roll - other.roll;
            double diffTransZ = transZ - other.transZ;
            double rotDist = std::sqrt(
                    diffPitch * diffPitch +
                    diffYaw * diffYaw +
                    diffRoll * diffRoll);
            double transZDist = std::abs(diffTransZ);
            return  rotDist < rotEpsilon && transZDist < transZEpsilon;
        }
    };
    struct DOFBoundaries
    {
        double pitchMax = 0;
        double pitchMin = 0;
        double yawMax = 0;
        double yawMin = 0;
        double rollMax = 0;
        double rollMin = 0;
        double transZMax = 0;
        double transZMin = 0;
    };

    class PivotController {
    protected:
        bool mDofPoseReady = false;
        bool mDofBoundariesReady = false;
    public:
        virtual bool setTargetDOFPose(
                DOFPose) = 0;
        virtual bool getCurrentDOFPose(
                DOFPose &laparoscopeDofPose) = 0;
        virtual bool getDOFBoundaries(
                DOFBoundaries &laparoscopeDofBoundaries) = 0;
        bool isReady() {
            return mDofBoundariesReady && mDofPoseReady;};
    };
}

#endif //PIVOT_CONTROL_MESSAGES_H