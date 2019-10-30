#!/usr/bin/env python

# Author: Christian Melendez
# Inverse Kinematics program for a ROBOTIS-OP3 robot

from enum import Enum
from time import sleep

import rosnode
import rospy
from std_msgs.msg import String

from control_modules.control_module import ControlModule
from iros_vision.msg import ObjectCoords
from keys import Keys

import roslib.packages as rospkg
import os
import json
import math

from sensor_msgs.msg import JointState
from robotis_controller_msgs.msg import SyncWriteItem
from robotis_controller_msgs.srv import SetModule

class InverseKinematics(object):
    joints = [
        'r_sho_pitch', 
        'l_sho_pitch', 
        'r_sho_roll', 
        'l_sho_roll', 
        'r_el', 
        'l_el', 
        'r_hip_yaw', 
        'l_hip_yaw', 
        'r_hip_roll', 
        'l_hip_roll', 
        'r_hip_pitch', 
        'l_hip_pitch', 
        'r_knee', 
        'l_knee', 
        'r_ank_pitch', 
        'l_ank_pitch', 
        'r_ank_roll', 
        'l_ank_roll', 
        'head_pan', 
        'head_tilt'
    ]

    def __init__(self):
        self.sync_write_pub = rospy.Publisher(
            '/robotis/sync_write_item', SyncWriteItem, queue_size=0)
        self.dxl_torque_pub = rospy.Publisher(
            '/robotis/dxl_torque', String, queue_size=0)
        self.write_joint_pub = rospy.Publisher(
            'robotis/set_joint_states', JointState, queue_size=0)
        self.write_joint_pub2 = rospy.Publisher(
            'robotis/direct_control/set_joint_states', JointState, queue_size=0)

        self.read_joint_sub = rospy.Subscriber(
            '/robotis/present_joint_states', JointState, self.get_joint_states)

        self.set_joint_module_client = rospy.ServiceProxy(
            '/robotis/set_present_ctrl_modules', SetModule)

        vals = [0.0]*20

        self.upper_arm = 89 # mm
        self.forearm = 157 # mm

        self.key_width = 20

        self.joint_positions = dict(zip(InverseKinematics.joints, vals))

        self.active_control = False


    def get_joint_states(self, msg):
        self.header = msg.header
        for joint in range(len(msg.name)):
            name = msg.name[joint]
            position = msg.position[joint]

            self.joint_positions[name] = position
    

    def set_module(self, name):
        if not self.active_control:
            set_module_srv = SetModule()
            set_module_srv.module_name = name

            if not self.set_joint_module_client.call(set_module_srv.module_name):
                rospy.logerr('Failed to set module')
            
            self.active_control = True


    def publish_joints(self):
        self.set_module('direct_control_module')
        msg = JointState()
        msg.header = self.header

        for joint in self.joint_positions:
            name = joint
            position = self.joint_positions[joint]

            msg.name.append(name)
            msg.position.append(position)

        self.write_joint_pub2.publish(msg)


    def move_r_shoulder(self): 
        self.joint_positions['r_sho_pitch'] -= 0.017 # move arm down by 1 degree
        self.publish_joints()

        return self.joint_positions['r_sho_pitch']


    def move_arm(self, arm, values):
        self.joint_positions['{}_sho_pitch'.format(arm)] = values[0]
        self.joint_positions['{}_sho_roll'.format(arm)] = values[1]
        self.joint_positions['{}_el'.format(arm)] = values[2]
        self.publish_joints()


    def move_joint_to(self, joint, angle):
        self.joint_positions[joint] = angle
        self.publish_joints()


    def get_arm_values(self):
        left = (self.joint_positions['l_sho_pitch'], self.joint_positions['l_sho_roll'], self.joint_positions['l_el'])
        right = (self.joint_positions['r_sho_pitch'], self.joint_positions['r_sho_roll'], self.joint_positions['r_el'])

        return left, right


    def calc_sho_roll(self, px, py):
        d = (px/(px*px + py*py)**0.5)
        return math.asin(d)


    def equations(self, px, py, pz):
        a1 = self.upper_arm
        a2 = self.forearm

        f1 = float(a2*a2 + a1*a1 - px*px - py*py - pz*pz)
        f2 = float(2*a2*a1)
        A = f1 / f2

        # print('f1',f1, 'f2',f2, 'A',A)

        # elbow = math.pi/2. - math.acos(A) # theta 6
        
        elbow = -math.acos(A) # theta 6
        # if elbow == -math.pi:
        #     elbow = 0
        # elbow = 0

        # sho_roll = math.asin(py/(a1 + a2*math.cos(elbow))) # theta 4
        sho_roll = math.acos(py/(px*px + py*py + pz*pz)**0.5) # theta 4

        B = -a2*math.sin(elbow)
        C = -a1*math.cos(sho_roll) - a2*math.cos(sho_roll)*math.cos(elbow)

        # sho_pitch = 2*math.atan((C + (B*B + C*C - px*px)**0.5) / (B + px)) # theta 2

        sho_pitch = 0
        if px != 0:
            sho_pitch = math.atan(pz/px) - elbow # theta 2
        else:
            print('B C',B, C)
            sho_pitch = 2*math.atan((C + (B*B + C*C - px*px)**0.5) / (B + px)) # theta 2

        return sho_pitch, sho_roll, elbow
        
        