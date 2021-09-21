#!/usr/bin/env python3
import rospy
import rosbag
import yaml
from threading import Lock
from datetime import datetime
from pivot_control_messages_ros.srv import GetInt, SetInt, SetPose
from pivot_control_messages_ros.msg import LaparoscopeDOFPose,\
    LaparoscopeDOFBoundaries, PivotError, FrankaError
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger

delayStep = 10
delayMax = 2000

screen_boxtrainer = "screen-boxtrainer"
vr_boxtrainer = "vr-boxtrainer"
vr_simulation = "vr-simulation"
vr_robotarm = "vr-robotarm"

def DofPoseToStr(pose):
    return '{} Pitch:{} Yaw:{} Roll:{} TransZ:{}'.format(
        pose['id'], pose['pitch'], pose['yaw'],
        pose['roll'], pose['trans_z']
    )

class UserStudy:
    def __init__(self):
        self.ready = True
        self.participantIdStr = ""
        self.rosbag_lock = Lock()
        #init ROS
        node = rospy.init_node('user_study')

        if rospy.has_param('~participant_id'):
            self.participantIdStr =  rospy.get_param('~participant_id')
        if rospy.has_param('~setup_id'):
            self.setupIdStr =  rospy.get_param('~setup_id')
        self.enableSetSimulationDelay = False
        self.enableForceNewPose = False

        #Start rosbags
        #Subscribe wanted topics
        self.rosbagLocation = rospy.get_param("/log_folder", "")
        self.rosbag = None
        self.targetDOFPoseSub = None
        self.DOFBoundariesSub = None
        self.currentDOFPoseSub = None
        self.tfSub = None
        self.pivotErrorSub = None
        self.frankaErrorSub = None
        self.imageSub = None
        self.cameraInfoSub = None
        topic = 'display/quit'
        rospy.wait_for_service(topic)
        self.quitService = rospy.ServiceProxy(topic, Trigger)

    def initializeParticipantId(self):
        if self.participantIdStr == "":
            print("participant ID empty")
            return False
        try:
            participantId = int(self.participantIdStr)
            print("Set participant ID to \'{}\'".format(self.participantIdStr))
            rospy.set_param("participant_id", self.participantIdStr)
        except Exception as e:
            rospy.logerr("participant ID is invalid give number! {}".format(e))
            self.participantIdStr = ""
            return False
        return True


    def initializeSetupId(self):
        valid = False
        if self.setupIdStr == screen_boxtrainer\
                or self.setupIdStr == vr_boxtrainer:
            valid = True
        if self.setupIdStr == vr_simulation:
            self.enableForceNewPose = True
            valid = True
        if self.setupIdStr == vr_robotarm:
            self.enableForceNewPose = True
            valid = True
        if valid:
            print("set setup_id")
            rospy.set_param("setup_id", self.setupIdStr)
        return valid

    def printPoses(self):
        i = 0
        for dofPose in self.startPosesDofs:
            i = i + 1
            print("{}: {}".format(i, DofPoseToStr(dofPose)))


    def initializeRest(self):
        if not self.ready:
            return
        if self.enableForceNewPose:
            if not rospy.has_param('~dof_poses_yaml'):
                rospy.logerr("no dofPoses File registered")
                self.ready = False
                # break an quit
                return False
            dofPosesYamlUrl = rospy.get_param('~dof_poses_yaml')
            rospy.loginfo("load Poses from {}".format(dofPosesYamlUrl))
            self.startPosesDofs = None
            self.curStartPoseNum = -1
            if dofPosesYamlUrl != '':
                with open(dofPosesYamlUrl) as f:
                    self.startPosesDofs = yaml.load(f, Loader=yaml.FullLoader)
            self.printPoses()
            rospy.wait_for_service('force_set_dof_pose')
            self.forceNewPoseService = rospy.ServiceProxy('force_set_dof_pose', SetPose)
            self.curStartPoseId = "NoPose"

        if self.enableSetSimulationDelay:
            # Get/Set simulaiton-delay
            self.getSimulationDelayService = rospy.ServiceProxy(
                'get_simulation_delay', GetInt)
            self.setSimulationDelayService = rospy.ServiceProxy(
                'set_simulation_delay', SetInt)

            response = self.getSimulationDelayService()
            self.curSimulationDelay = response.data
            # self.curSimulationDelay = 0
        if not self.TestTopics():
            rospy.logerr("needed topics are not publishing")
            self.ready = False


    def participantSetupDialog(self):
        if self.participantIdStr == "":
            self.participantIdStr = input("Participant ID:")
            if self.participantIdStr == "q":
                self.ready = False
                return
        if self.setupIdStr == "":
            print("1: " + screen_boxtrainer)
            print("2: " + vr_boxtrainer)
            print("3: " + vr_simulation)
            print("4: " + vr_robotarm)
            id = input("Setup ID:")
            if id == '1':
                self.setupIdStr = screen_boxtrainer
            if id == '2':
                self.setupIdStr = vr_boxtrainer
            if id == '3':
                self.setupIdStr = vr_simulation
            if id == '4':
                self.setupIdStr = vr_robotarm
            if id == "q":
                self.ready = False

    def SetStartPoseAbs(self, num):
        rospy.loginfo('force_set_dof_pose')
        if not (0 <= num and num < len(self.startPosesDofs)):
            rospy.logwarn("Out of bound")
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
            rospy.logwarn('simulation delay too big')
            return False
        rospy.loginfo('set simulation delay to {}ms'.format(delay))
        rospy.wait_for_service('set_simulation_delay')
        response = self.setSimulationDelayService(delay)
        return response.success

    def SetSimulationDelayRel(self, rel):
        #getSimulaiton Delay
        rospy.loginfo('get_simulation_delay')
        rospy.wait_for_service('get_simulation_delay')
        response = self.getSimulationDelayService()
        if response.success:
            newDelay = response.data + rel
            return self.SetSimulationDelayAbs(newDelay)
        else:
            return False

    def ToggleRosbag(self):
        if self.rosbag is None:
            poses_indicator = "_{}{}".format(
                self.curStartPoseNum + 1,
                self.curStartPoseId) if self.enableForceNewPose else ""
            delay_indicator = "_{}".format(
                self.curSimulationDelay) if self.enableSetSimulationDelay else ""
            t = datetime.now()
            timestr = t.isoformat()
            bagName = '{}/rosbag_{}_{}{}{}_{}.bag'.format(
                self.rosbagLocation,
                self.participantIdStr,
                self.setupIdStr,
                poses_indicator,
                delay_indicator,
                timestr)
            #create ROS Bag
            file = open(bagName, 'w+')
            file.close()
            self.rosbag = rosbag.Bag(bagName, 'w')
            rospy.loginfo("Start Recording to:{}".format(bagName))
            self.SubscribeTopics()
            return True
        else:
            self.UnsubscribeTopics()
            self.StopRosbag()
            return False

    def TestTopics(self):
        try:
            timeout = 1.0
            if self.setupIdStr == vr_simulation or self.setupIdStr == vr_robotarm:
                # this topic does not publish continuously
                # topic = "target/DOFPose"
                # rospy.wait_for_message(topic, LaparoscopeDOFPose, timeout)
                topic = "DOFBoundaries"
                rospy.wait_for_message(topic, LaparoscopeDOFBoundaries, timeout)
                topic = "current/DOFPose"
                rospy.wait_for_message(topic, LaparoscopeDOFPose, timeout)

            if self.setupIdStr == vr_robotarm \
                    or self.setupIdStr == vr_simulation \
                    or self.setupIdStr == vr_robotarm:
                topic = "/tf"
                rospy.wait_for_message(topic, TFMessage, 10)

            if self.setupIdStr == vr_robotarm:
                topic = "pivot_error"
                rospy.wait_for_message(topic, PivotError, timeout)

            topic = "image_raw"
            rospy.wait_for_message(topic, Image, timeout)
            topic = "camera_info"
            rospy.wait_for_message(topic, CameraInfo, timeout)
        except (rospy.ROSException) as err:
            rospy.logerr("ROSException: {0}".format(err))
            return False
        return True

    def SubscribeTopics(self):
        if self.setupIdStr == vr_simulation or self.setupIdStr == vr_robotarm:
            topic = "target/DOFPose"
            self.targetDOFPoseSub = rospy.Subscriber(
                topic, LaparoscopeDOFPose, self.WriteToROSBag, topic)
            topic = "DOFBoundaries"
            self.DOFBoundariesSub = rospy.Subscriber(
                topic, LaparoscopeDOFBoundaries, self.WriteToROSBag, topic)
            topic = "current/DOFPose"
            self.currentDOFPoseSub = rospy.Subscriber(
                topic, LaparoscopeDOFPose, self.WriteToROSBag, topic)

        if self.setupIdStr == vr_robotarm \
                or self.setupIdStr == vr_simulation \
                or self.setupIdStr == vr_robotarm:
            topic = "/tf"
            self.tfSub = rospy.Subscriber(
                topic, TFMessage, self.WriteToROSBag, topic)

        if self.setupIdStr == vr_robotarm:
            topic = "pivot_error"
            self.pivotErrorSub = rospy.Subscriber(
                topic, PivotError, self.WriteToROSBag, topic)
            topic = "franka_error"
            self.frankaErrorSub = rospy.Subscriber(
                topic, FrankaError, self.WriteToROSBag, topic)

        topic = "image_raw"
        self.imageSub = rospy.Subscriber(
            topic, Image, self.WriteToROSBag, topic)
        topic = "camera_info"
        self.cameraInfoSub = rospy.Subscriber(
            topic, CameraInfo, self.WriteToROSBag, topic)

    def UnsubscribeTopics(self):
        if self.targetDOFPoseSub is not None:
            self.targetDOFPoseSub.unregister()
        if self.DOFBoundariesSub is not None:
            self.DOFBoundariesSub.unregister()
        if self.currentDOFPoseSub is not None:
            self.currentDOFPoseSub.unregister()
        if self.tfSub is not None:
            self.tfSub.unregister()
        if self.pivotErrorSub is not None:
            self.pivotErrorSub.unregister()
        if self.frankaErrorSub is not None:
            self.frankaErrorSub.unregister()
        if self.imageSub is not None:
            self.imageSub.unregister()
        if self.cameraInfoSub is not None:
            self.cameraInfoSub.unregister()

    def StopRosbag(self):
        self.rosbag_lock.acquire()
        rospy.loginfo("StartStop Recording")
        if self.rosbag is not None:
            self.rosbag.close()
            self.rosbag = None
        rospy.loginfo("Stop Recording")
        self.rosbag_lock.release()

    def WriteToROSBag(self, message, topic):
        self.rosbag_lock.acquire()
        if self.rosbag is None:
            return
        try:
            self.rosbag.write(topic, message)
        except Exception as err:
            rospy.logerr("ROSBAGException: {0}".format(err))
            rospy.logwarn("something went wrong with the ROSBag")
            self.rosbag.close()
            self.rosbag = None
        finally:
            self.rosbag_lock.release()


def mainloop(userStudy):
    print("q:  Quit")
    if userStudy.enableForceNewPose:
        print("j:  Go to previous Pose")
        print("k:  Go to next Pose")
        print("pX: Go to Xth Pose")
        print("i:  Print all available Poses")
    if userStudy.enableSetSimulationDelay:
        print("d:  Decrease simulation delay")
        print("f:  Increase simulation delay")
        print("eX: Set simulation delay to X ms")
    print("r:  Record than press enter for start and stop")
    recording = False
    while not rospy.is_shutdown():
        command = input('>')
        # set delay
        if command == 'q':
            userStudy.UnsubscribeTopics()
            userStudy.StopRosbag()
            try:
                resp = userStudy.quitService()
                if not resp.success:
                    rospy.logwarn('Quit Display Failed')
            except Exception as e:
                print("Display allready quit")
            rospy.loginfo('Quit')
            break

        if recording:
            recording = userStudy.ToggleRosbag()
            continue

        # set previous Pose
        if command == 'j' and userStudy.startPosesDofs is not None:
            print('previous pose')
            succ = userStudy.SetStartPoseRel(-1)
            if not succ:
                rospy.logwarn("could not set new pose")
            continue
        # set next Pose
        if command == 'k' and userStudy.startPosesDofs is not None:
            print('Next pose')
            succ = userStudy.SetStartPoseRel(1)
            if not succ:
                rospy.logwarn("could not set new pose")
            continue
        # Go To specific Pose
        if command.startswith('p') and userStudy.startPosesDofs is not None:
            poseNum = None
            try:
                poseNum = int(command[1:])
            except:
                rospy.logwarn("Could not parse num")
                continue
            succ = userStudy.SetStartPoseAbs(poseNum-1)
            if not succ:
                rospy.logwarn("Could not set new pose")
            else:
                print("Go to {}".format(userStudy.curStartPoseId))
            continue
        if userStudy.enableSetSimulationDelay:
            if command == 'd':
                print('Decrease simulation delay')
                succ = userStudy.SetSimulationDelayRel(-delayStep)
                if not succ:
                    rospy.logwarn("Could not decrease delay")
                continue
            if command == 'f':
                print('Increase simulation delay')
                succ = userStudy.SetSimulationDelayRel(delayStep)
                if not succ:
                    rospy.logwarn("could not increase delay")
                continue
            if command.startswith('e'):
                print('set simulation delay')
                delay = None
                try:
                    delay = int(command[1:])
                except:
                    rospy.logwarn("Could not set delay")
                    continue
                succ = userStudy.SetSimulationDelayAbs(delay)
                if not succ:
                    rospy.logwarn("could not set new simulation delay")
                continue
            if command == 'i':
                userStudy.printPoses()
                continue
        # Record Rosbag
        if command == 'r':
            print("Press Enter to stop")
            recording = userStudy.ToggleRosbag()
            continue
        print("could not parse command")

if __name__ == '__main__':
    userStudy = UserStudy()
    succParticipantId = userStudy.initializeParticipantId()
    succSetupId = userStudy.initializeSetupId()
    while not(succParticipantId and succSetupId) and userStudy.ready:
        userStudy.participantSetupDialog()
        if not succParticipantId:
            succParticipantId = userStudy.initializeParticipantId()
        if not succSetupId:
            succSetupId = userStudy.initializeSetupId()
        print("print succ participantID: {} setupID: {}".format(succParticipantId, succSetupId))
    userStudy.initializeRest()
    if userStudy.ready:
       mainloop(userStudy)