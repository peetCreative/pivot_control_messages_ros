#!/usr/bin/env python
import rospy
import rosbag
import yaml
from threading import Lock
from datetime import datetime
from pivot_control_messages_ros.srv import GetInt, SetInt, SetPose
from pivot_control_messages_ros.msg import LaparoscopeDOFPose, LaparoscopeDOFBoundaries
from sensor_msgs.msg import Image

delayStep = 10
delayMax = 2000

screen_boxtrainer = "screen_box-trainer"
vr_boxtrainer = "vr_box-trainer"
vr_simulation = "vr_box-trainer"
vr_robotarm = "vr_box-trainer"

def DofPoseToStr(pose):
    return '{} Pitch:{} Yaw:{} Roll:{} TransZ:{}'.format(
        pose['id'], pose['pitch'], pose['yaw'],
        pose['roll'], pose['trans_z']
    )

class UserStudy:
    def __init__(self):
        self.ready = True
        self.participantId = "NONE"
        self.rosbag_lock = Lock()

        #init ROS
        node = rospy.init_node('user_study')

        if not rospy.has_param('~dof_poses_yaml')\
                or not rospy.has_param('~pid')\
                or not rospy.has_param('~setup'):
            rospy.logerr("one of the following topics missing: dof_poses_yaml, pid, setup")
            self.ready = False
            return
        dofPosesYamlUrl =  rospy.get_param('~dof_poses_yaml')
        self.participantId =  rospy.get_param('~pid')
        if type(self.participantId) is not int:
            rospy.logerr("pid is not int")
            self.ready = False
            return
        self.setupStr =  rospy.get_param('~setup')
        self.enableSetSimulationDelay = False
        self.enableForceNewPose = False
        validSetup = False
        if self.setupStr == 1 or self.setupStr == screen_boxtrainer:
            self.setupStr = screen_boxtrainer
            validSetup = True
        if self.setupStr == 2 or self.setupStr == vr_boxtrainer:
            self.setupStr = vr_boxtrainer
            validSetup = True
        if self.setupStr == 3  or self.setupStr == vr_simulation:
            self.setupStr = vr_simulation
            self.enableForceNewPose = True
            validSetup = True
        if self.setupStr == 4  or self.setupStr == vr_robotarm:
            self.setupStr = vr_robotarm
            self.enableForceNewPose = True
            validSetup = True
        if not validSetup:
            rospy.logerr("{} is no valid setup".format(self.setupStr))
            self.ready = False
            return
        if self.enableForceNewPose:
            rospy.loginfo("load Poses from {}".format(dofPosesYamlUrl))
            self.startPosesDofs = None
            self.curStartPoseNum = -1
            if dofPosesYamlUrl != '':
                with open(dofPosesYamlUrl) as f:
                    self.startPosesDofs = yaml.load(f, Loader=yaml.FullLoader)
            i = 0
            for dofPose in self.startPosesDofs:
                i = i + 1
                rospy.loginfo("{}: {}".format(i, DofPoseToStr(dofPose)))
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

        #Start rosbags
        #Subscribe wanted topics
        self.rosbagLocation = rospy.get_param("~rosbag_location_folder", "")
        self.rosbag = None
        if not self.TestTopics():
            rospy.logerr("needed topics are not publishing")
            self.ready = False
            return

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
            bagName = '{}/{}_{}{}{}_{}.bag'.format(
                self.rosbagLocation,
                self.participantId,
                self.setupStr,
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
            if self.setupStr == vr_simulation or self.setupStr == vr_robotarm:
                # this topic does not publish continuously
                # topic = "target/DOFPose"
                # rospy.wait_for_message(topic, LaparoscopeDOFPose, timeout)
                topic = "DOFBoundaries"
                rospy.wait_for_message(topic, LaparoscopeDOFBoundaries, timeout)
                topic = "current/DOFPose"
                rospy.wait_for_message(topic, LaparoscopeDOFPose, timeout)
                #TODO subscribe to Robot camera_tip tf
                self.cameraTipSub = None

            if self.setupStr == vr_robotarm \
                    or self.setupStr == vr_simulation \
                    or self.setupStr == vr_robotarm:
                #TODO subscribe to head movements
                self.headTfSub = None
                pass

            topic = "image_raw"
            rospy.wait_for_message(topic, Image, timeout)
        except (rospy.ROSException) as err:
            rospy.logerr("ROSException: {0}".format(err))
            return False
        return True

    def SubscribeTopics(self):
        if self.setupStr == vr_simulation or self.setupStr == vr_robotarm:
            topic = "target/DOFPose"
            self.targetDOFPoseSub = rospy.Subscriber(
                topic, LaparoscopeDOFPose, self.WriteToROSBag, topic)
            topic = "DOFBoundaries"
            self.DOFBoundariesSub = rospy.Subscriber(
                topic, LaparoscopeDOFBoundaries, self.WriteToROSBag, topic)
            topic = "current/DOFPose"
            self.currentDOFPoseSub = rospy.Subscriber(
                topic, LaparoscopeDOFPose, self.WriteToROSBag, topic)
            #TODO subscribe to Robot camera_tip tf
            self.cameraTipSub = None

        if self.setupStr == vr_robotarm \
                or self.setupStr == vr_simulation \
                or self.setupStr == vr_robotarm:
            #TODO subscribe to head movements
            self.headTfSub = None
            pass

        topic = "image_raw"
        self.imageSub = rospy.Subscriber(
            topic, Image, self.WriteToROSBag, topic)

    def UnsubscribeTopics(self):
        if self.setupStr == vr_simulation or self.setupStr == vr_robotarm:
            self.targetDOFPoseSub.unregister()
        if self.setupStr == vr_simulation or self.setupStr == vr_robotarm:
            self.DOFBoundariesSub.unregister()
        if self.setupStr == vr_simulation or self.setupStr == vr_robotarm:
            self.currentDOFPoseSub.unregister()
        if self.imageSub is not None:
            self.imageSub.unregister()

    def StopRosbag(self):
        if self.rosbag is not None:
            self.rosbag.close()
            self.rosbag = None
        rospy.loginfo("Stop Recording")

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
            bag = None
        finally:
            self.rosbag_lock.release()


def mainloop(userStudy):
    rospy.loginfo("q:  Quit")
    if userStudy.enableForceNewPose:
        rospy.loginfo("j:  Go to previous Pose")
        rospy.loginfo("k:  Go to next Pose")
        rospy.loginfo("pX: Go to Xth Pose")
    if userStudy.enableSetSimulationDelay:
        rospy.loginfo("d:  Decrease simulation delay")
        rospy.loginfo("f:  Increase simulation delay")
        rospy.loginfo("eX: Set simulation delay to X ms")
    rospy.loginfo("r:  Record than press enter for start and stop")
    recording = False
    while not rospy.is_shutdown():
        command = input('>')
        # set delay
        if command == 'q':
            userStudy.UnsubscribeTopics()
            userStudy.StopRosbag()
            rospy.loginfo('Quit')
            break

        if recording:
            recording = userStudy.ToggleRosbag()
            continue

        # set previous Pose
        if command == 'j' and userStudy.startPosesDofs is not None:
            rospy.loginfo('previous start pose')
            succ = userStudy.SetStartPoseRel(-1)
            if not succ:
                rospy.logwarn("could not set new pose")
            continue
        # set next Pose
        if command == 'k' and userStudy.startPosesDofs is not None:
            rospy.loginfo ('Next start pose')
            succ = userStudy.SetStartPoseRel(1)
            if not succ:
                rospy.logwarn("could not set new pose")
            continue
        # Go To specific Pose
        if command.startswith('p') and userStudy.startPosesDofs is not None:
            rospy.loginfo("Go to start pose")
            poseNum = None
            try:
                poseNum = int(command[1:])
            except:
                rospy.logwarn("Could not parse num")
                continue
            succ = userStudy.SetStartPoseAbs(poseNum-1)
            if not succ:
                rospy.logwarn("Could not set new pose")
            continue
        if userStudy.enableSetSimulationDelay:
            if command == 'd':
                rospy.loginfo('Decrease simulation delay')
                succ = userStudy.SetSimulationDelayRel(-delayStep)
                if not succ:
                    rospy.logwarn("Could not decrease delay")
                continue
            if command == 'f':
                rospy.loginfo('Increase simulation delay')
                succ = userStudy.SetSimulationDelayRel(delayStep)
                if not succ:
                    rospy.logwarn("could not increase delay")
                continue
            if command.startswith('e'):
                rospy.loginfo('set simulation delay')
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
        # Record Rosbag
        if command == 'r':
            rospy.loginfo("Press Enter to stop")
            recording = userStudy.ToggleRosbag()
            continue
        rospy.logwarn("could not parse command")

if __name__ == '__main__':
    userStudy = UserStudy()
    if userStudy.ready:
       mainloop(userStudy)