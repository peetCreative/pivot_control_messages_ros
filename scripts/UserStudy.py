#!/usr/bin/env python
import rospy
import rosbag
import yaml
from pivot_control_messages_ros.srv import GetInt, SetInt, SetPose
from pivot_control_messages_ros.msg import LaparoscopeDOFPose

delayStep = 10
delayMax = 2000

def DofPoseToStr(pose):
    return '{} Pitch:{} Yaw:{} Roll:{} TransZ:{}'.format(
        pose['id'], pose['pitch'], pose['yaw'],
        pose['roll'], pose['trans_z']
    )

class UserStudy:
    def __init__(self):
        self.participantId = "NONE"

        #init ROS
        node = rospy.init_node('user_study')

        # Set new poses
        dofPosesYamlUrl =  rospy.get_param('~dof_poses_yaml')
        print("load Poses from {}".format(dofPosesYamlUrl))
        self.startPosesDofs = None
        self.curStartPoseNum = 0
        if dofPosesYamlUrl != '':
            with open(dofPosesYamlUrl) as f:
                self.startPosesDofs = yaml.load(f, Loader=yaml.FullLoader)
        i = 0
        for dofPose in self.startPosesDofs:
            i = i + 1
            print("{}: {}".format(i, DofPoseToStr(dofPose)))
        self.forceNewPoseService = rospy.ServiceProxy('force_set_dof_pose', SetPose)
        self.curStartPoseId = "None"

        # Get/Set simulaiton-delay
        self.getSimulationDelayService = rospy.ServiceProxy(
            'get_simulation_delay', GetInt)
        self.setSimulationDelayService = rospy.ServiceProxy(
            'set_simulation_delay', SetInt)

        response = self.getSimulationDelayService()
        self.curSimulationDelay = response.data
        # self.curSimulationDelay = 0

        #Start rosbags
        #Subscribe wanted topics
        self.rosbagLocation = rospy.get_param("~rosbag_location_folder", "")
        self.rosbag = None
        self.targetDOFPoseSub = rospy.Subscriber(
            'target/DOFPose', LaparoscopeDOFPose, self.WriteROSBag)

    def SetParticipantId(self, participantId):
        self.participantId = participantId

    def SetStartPoseAbs(self, num):
        print('force_set_dof_pose')
        if not (0 <= num and num < len(self.startPosesDofs)):
            print("Out of bound")
            return False
        pose = self.startPosesDofs[num]
        rospy.wait_for_service('force_set_dof_pose')
        response = self.forceNewPoseService(
            pose['pitch'], pose['yaw'], pose['roll'], pose['trans_z'])
        if response.success:

            self.curStartPoseNum = num
            self.curStartPoseId = pose['id']
            return True
        else:
            return False

    def SetStartPoseRel(self, rel):
        return self.SetStartPoseAbs(self.curStartPoseNum + rel)

    def SetSimulationDelayAbs(self, delay):
        if delay < 0:
            delay = 0
        if delay > delayMax:
            print('simulation delay too big')
            return False
        print('set simulation delay to {}ms'.format(delay))
        rospy.wait_for_service('set_simulation_delay')
        response = self.setSimulationDelayService(delay)
        return response.success

    def SetSimulationDelayRel(self, rel):
        #getSimulaiton Delay
        print('get_simulation_delay')
        rospy.wait_for_service('get_simulation_delay')
        response = self.getSimulationDelayService()
        if response.success:
            newDelay = response.data + rel
            return self.SetSimulationDelayAbs(newDelay)
        else:
            return False

    def ToggleRosbag(self):
        if self.rosbag is None:
            bagName = '{}{}_{}{}_{}.bag'.format(
                self.rosbagLocation,
                self.participantId,
                self.curStartPoseNum,
                self.curStartPoseId,
                self.curSimulationDelay)
            #create ROS Bag
            file = open(bagName, 'w+')
            file.close()
            self.rosbag = rosbag.Bag(bagName, 'w')
            print("Start Recording to:{}".format(bagName))
            return True
        else:
            self.StopRosbag()
            return False

    def StopRosbag(self):
        if self.rosbag is not None:
            self.rosbag.close()
            self.rosbag = None
        print("Stop Recording")

    def WriteROSBag(self, DOFPose):
        if self.rosbag is None:
            return
        self.rosbag.write('target/dof_pose', DOFPose)
        try:
            pass
        except:
            print("something went wrong with the ROSBag")
            self.rosbag.close()
            self.rosbag = None



if __name__ == '__main__':
    userStudy = UserStudy()

    # From
    participantId = ""
    while len(participantId) < 2:
        print("Please Enter a valid participant id:")
        participantId = input()
    print("start userstudy for {}".format(participantId))
    userStudy.SetParticipantId(participantId)

    #TODO: explain what you can do
    print("q:  Quit")
    print("j:  Go to previous Pose")
    print("k:  Go to next Pose")
    print("pX: Go to Xth Pose")
    print("d:  Decrease simulation delay")
    print("f:  Increase simulation delay")
    print("eX: Set simulation delay to X ms")
    print("r:  Record than press enter for start and stop")
    recording = False
    while not rospy.is_shutdown():
        command = input('>')
        # set delay
        if command == 'q':
            userStudy.StopRosbag()
            print('Quit')
            break

        if recording:
            recording = userStudy.ToggleRosbag()
            continue

        # set previous Pose
        if command == 'j' and userStudy.startPosesDofs is not None:
            print ('previous start pose')
            succ = userStudy.SetStartPoseRel(-1)
            if not succ:
                print("could not set new pose")
            continue
        # set next Pose
        if command == 'k' and userStudy.startPosesDofs is not None:
            print ('Next start pose')
            succ = userStudy.SetStartPoseRel(1)
            if not succ:
                print("could not set new pose")
            continue
        # Go To specific Pose
        if command.startswith('p') and userStudy.startPosesDofs is not None:
            print ("Go to start pose")
            poseNum = None
            try:
                poseNum = int(command[1:])
            except:
                print("Could not parse num")
                continue
            succ = userStudy.SetStartPoseAbs(poseNum)
            if not succ:
                print("Could not set new pose")
            continue
        if command == 'd':
            print('Decrease simulation delay')
            succ = userStudy.SetSimulationDelayRel(-delayStep)
            if not succ:
                print("Could not decrease delay")
            continue
        if command == 'f':
            print('Increase simulation delay')
            succ = userStudy.SetSimulationDelayRel(delayStep)
            if not succ:
                print("could not decrease delay")
            continue
        if command.startswith('e'):
            print('set simulation delay')
            delay = None
            try:
                delay = int(command[1:])
            except:
                print("Could not delay")
                continue
            succ = userStudy.SetSimulationDelayAbs(delay)
            if not succ:
                print("could not set new simulation delay")
            continue
        # Record Rosbag
        if command == 'r':
            print('Press Enter for record to rosbag')
            recording = True
            continue
        print("could not parse command")
