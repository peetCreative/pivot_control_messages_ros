#!/usr/bin/env python
import rospy
import pygame
import math
import numpy as np
from pygame.locals import K_ESCAPE, K_w, K_s, K_a, K_d, K_q, K_e, K_y, K_x
import argparse
from pivot_control_messages_ros.msg import LaparoscopeDOFPose
from pivot_control_messages_ros.msg import LaparoscopeDOFBoundaries


def limitToOne(a):
    if a <= -1:
        a = -1
    if a >= 1:
        a = 1
    return a


class KeyboardDOFController:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('keyboard_dof_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.pose_publisher = rospy.Publisher(
            "target/laparoscope_dof_pose",
            LaparoscopeDOFPose, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.boundaries_subscriber = rospy.Subscriber(
            "laparoscope_dof_boundaries",
            LaparoscopeDOFBoundaries, self.update_boundaries)
        self.pose_subscriber = rospy.Subscriber(
            "current/laparoscope_dof_pose",
            LaparoscopeDOFPose, self.update_pose)

        self.seq = 0
        self.pose = LaparoscopeDOFPose()
        self.boundaries = LaparoscopeDOFBoundaries()
        # rospy.Timer(rospy.Duration(0.1), self.checkKeyboard)
        self.rate = rospy.Rate(10)

        pygame.init()
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption('Python numbers')
        self.screen.fill((159, 182, 205))

        self.font = pygame.font.Font(None, 30)
        self.direction = "stop motion"

        # 5mm every step
        self.step_distance = 0.02
        # make rotation 4x faster than lateral movements
        self.rot_factor = 4
        # the laparoscope is 30° tilted
        self.camera_tilt = 0.52359
        self.add_camera_tilt = False
        # focus distance in 10 cm
        self.focus_distance = 0.10
        self.add_trans_pitch = False
        self.add_roll_correction = False

    def update_boundaries(self, boundaries):
        self.boundaries = boundaries

    def update_pose(self, pose):
        self.pose = pose

    def move(self):
        pose = LaparoscopeDOFPose()
        self.seq = self.seq + 1
        pose.header.seq = self.seq
        pose.header.stamp = rospy.Time()
        pose.header.frame_id = "laparoscope"
        pose.yaw = self.pose.yaw
        pose.pitch = self.pose.pitch
        pose.roll = self.pose.roll
        pose.trans_z = self.pose.trans_z
        if self.direction == "stop motion":
            # just publish the last pose we got
            self.pose_publisher.publish(self.pose)
            return
        # print("move further")
        # calculate the pitch allready caused by trans_z
        trans_pitch = math.asin(limitToOne(
            (math.sin(
                math.pi - self.camera_tilt) * pose.trans_z) / self.focus_distance
        ))
        axis_length = self.focus_distance
        # the point which should be equally fast moving
        if pose.trans_z != 0 and self.add_trans_pitch:
            axis_length = math.asin(limitToOne(
                (math.sin(
                    math.pi - self.camera_tilt) * pose.trans_z) / math.sin(
                    trans_pitch)
            ))
        else:
            axis_length = self.focus_distance - self.pose.trans_z
        # TODO: I don't understand this anymore
        step_pitch = self.rot_factor * self.step_distance# / axis_length
        step_yaw = self.rot_factor * self.step_distance# / axis_length
        step_roll = self.rot_factor * self.step_distance# / axis_length
        # apply step direction
        if self.direction == "up":
            pose.pitch = pose.pitch + step_pitch
        if self.direction == "down":
            pose.pitch = pose.pitch - step_pitch
        if self.direction == "left":
            pose.yaw = pose.yaw + step_yaw
        if self.direction == "right":
            pose.yaw = pose.yaw - step_yaw
        if self.direction == "rot_left":
            pose.roll = pose.roll + step_roll
        if self.direction == "rot_right":
            pose.roll = pose.roll - step_roll
        if self.direction == "in":
            # TODO: with pitching more distance will be covered
            pose.trans_z = pose.trans_z + self.step_distance
        if self.direction == "out":
            pose.trans_z = pose.trans_z - self.step_distance
        # check boundaries
        pose.pitch =  max(pose.pitch, self.boundaries.pitch_min)
        pose.pitch =  min(pose.pitch, self.boundaries.pitch_max)
        pose.yaw =  max(pose.yaw, self.boundaries.yaw_min)
        pose.yaw =  min(pose.yaw, self.boundaries.yaw_max)
        pose.roll =  max(pose.roll, self.boundaries.roll_min)
        pose.roll =  min(pose.roll, self.boundaries.roll_max)
        pose.trans_z =  max(pose.trans_z, self.boundaries.trans_z_min)
        pose.trans_z =  min(pose.trans_z, self.boundaries.trans_z_max)

        if (self.boundaries.yaw_min <= pose.yaw <= self.boundaries.yaw_max and
                self.boundaries.pitch_min <= pose.pitch <= self.boundaries.pitch_max and
                self.boundaries.roll_min <= pose.roll <= self.boundaries.roll_max and
                self.boundaries.trans_z_min <= pose.trans_z <= self.boundaries.trans_z_max):
            if self.pose.trans_z != pose.trans_z and self.add_trans_pitch:
                trans_pitch_new = math.asin(limitToOne(
                    (math.sin(math.pi - self.camera_tilt) * pose.trans_z) /
                    self.focus_distance
                ))
                pose.pitch = pose.pitch - (trans_pitch_new - trans_pitch)
            if self.add_roll_correction:
                tilt_pitch = pose.pitch - (
                    self.camera_tilt if self.add_camera_tilt else 0)
                vec1 = np.array([0, math.cos(tilt_pitch), math.sin(tilt_pitch)])
                vec2 = np.array([math.sin(pose.yaw) * math.sin(tilt_pitch),
                                 math.cos(tilt_pitch),
                                 math.cos(pose.yaw) * math.sin(tilt_pitch)])
                sign_pitch = 1 if pose.pitch > 0 else -1
                sign_yaw = 1 if pose.yaw < 0 else -1
                z_angle = np.dot(vec1, vec2)
                if z_angle < 1.0:
                    pose.roll = sign_yaw * sign_pitch * math.acos(z_angle)
            self.pose_publisher.publish(pose)

    def display(self):
        dirtext = self.font.render(self.direction, True, (255, 255, 255),
                                   (159, 182, 205))
        dirtextRect = dirtext.get_rect()
        dirtextRect.centerx = self.screen.get_rect().centerx
        dirtextRect.centery = self.screen.get_rect().centery

        str = "pitch:{:.2f} yaw:{:.2f} roll:{:.2f} trans_z:{:.2f}".format(
            self.pose.pitch, self.pose.yaw, self.pose.roll, self.pose.trans_z)
        posetext = self.font.render(str, True, (255, 255, 255), (159, 182, 205))
        posetextRect = posetext.get_rect()
        posetextRect.centerx = self.screen.get_rect().centerx
        posetextRect.centery = self.screen.get_rect().centery + 20

        str = "pitch_bound:{:.2f},{:.2f} yaw_bound:{:.2f},{:.2f} roll_bound:{:.2f},{:.2f}".format(
            self.boundaries.pitch_min, self.boundaries.pitch_max,
            self.boundaries.yaw_min, self.boundaries.yaw_max,
            self.boundaries.roll_min, self.boundaries.roll_max)
        boundtext = self.font.render(str, True, (255, 255, 255),
                                     (159, 182, 205))
        boundtextRect = boundtext.get_rect()
        boundtextRect.centerx = self.screen.get_rect().centerx
        boundtextRect.centery = self.screen.get_rect().centery + 40

        str = "trans_z_bound:{:.2f},{:.2f}".format(
            self.boundaries.trans_z_min, self.boundaries.trans_z_max)
        bound1text = self.font.render(str, True, (255, 255, 255),
                                      (159, 182, 205))
        bound1textRect = boundtext.get_rect()
        bound1textRect.centerx = self.screen.get_rect().centerx
        bound1textRect.centery = self.screen.get_rect().centery + 60

        self.screen.blit(dirtext, dirtextRect)
        self.screen.blit(posetext, posetextRect)
        self.screen.blit(boundtext, boundtextRect)
        self.screen.blit(bound1text, bound1textRect)
        pygame.display.update()

    def start(self):
        done = False
        move_old = False
        while not done:
            self.display()
            pygame.event.pump()
            keys = pygame.key.get_pressed()
            move = False
            if rospy.is_shutdown():
                done = True
            if keys[K_ESCAPE]:
                done = True
            if keys[K_w]:
                self.direction = "up"
                move = True
            if keys[K_s]:
                self.direction = "down"
                move = True
            if keys[K_a]:
                self.direction = "left"
                move = True
            if keys[K_d]:
                self.direction = "right"
                move = True
            if keys[K_q]:
                self.direction = "out"
                move = True
            if keys[K_e]:
                self.direction = "in"
                move = True
            if keys[K_y]:
                self.direction = "rot_left"
                move = True
            if keys[K_x]:
                self.direction = "rot_right"
                move = True
            # if keys[K_]:
            #     self.add_camera_tilt = True
            #     self.direction = ""
            #     move = True
            # if keys[K_n]:
            #     self.add_camera_tilt = False
            #     self.direction = ""
            #     move = True
            if move:
                self.move()
                move_old = True
            if move_old and not move:
                self.direction = "stop motion"
                self.move()
                move_old = False


if __name__ == '__main__':
    try:
        x = KeyboardDOFController()
        x.start()
    except rospy.ROSInterruptException:
        pass
