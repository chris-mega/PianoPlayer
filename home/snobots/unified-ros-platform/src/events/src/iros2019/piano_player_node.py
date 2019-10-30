#!/usr/bin/env python

# Author: Christian Melendez
# PianoPlayer program for a ROBOTIS-OP3 robot
# Code structure adapted from 'United ROS Platform' made by Snobots at UofM

from enum import Enum
from time import sleep, time
import cv2 as cv

import rosnode
import rospy
from std_msgs.msg import String

from control_modules.control_module import ControlModule
from iros_vision.msg import ObjectCoords
from keys import Keys
from inverse_kinematics import InverseKinematics

import roslib.packages as rospkg
import os
import json
import numpy as np
import math
from udp_connect import client

# SETUP ----------------------------------------------

class RobotMode(Enum):
    READY = 0
    RUNNING = 1


def button_callack(msg):
    global robot_mode, mode_change

    if msg.data == 'start':
        # Start button pressed
        # Send robot to RUNNING mode
        if robot_mode == RobotMode.READY:
            robot_mode = RobotMode.RUNNING
            mode_change = True
        elif robot_mode == RobotMode.RUNNING:
            rospy.logerr('Robot already in running mode.')

    elif msg.data == 'mode':
        # Mode button pressed
        # Send robot to READY mode
        robot_mode = RobotMode.READY
        mode_change = True


def wait_for_node(node_name):
    # Wait for node to start
    while node_name not in rosnode.get_node_names():
        rospy.logwarn('Node is not running: {0}'.format(node_name))
        sleep(1)

    rospy.loginfo('Node is running: {0}'.format(node_name))


# CONTROL (MOTIONS) ----------------------------------------------------------

def bass_billie_jean():
    actions = ['Re right', 'La left', 'Do right', 'Re right', 'Do right', 'La left', 'Sol left', 'La left']

    avg = .0

    for action in actions:
        t1 = time()
        control_module.action().play_action(action)
        avg += (time() - t1)
    
    return avg/len(actions)


def measure_key(action, expected):
    t1 = time()
    control_module.action().play_action(action)
    # 0.438 is the average time it takes a ready position to be played
    while(time()-t1 < expected-0.438):
        sleep(0.001)
    control_module.action().play_action('{} ready'.format(action))

    return time() - t1


def knokin_on(beat_to_sec, repeat):
    print('C', measure_key('C', 2*beat_to_sec))
    print('G', measure_key('G', 2*beat_to_sec))
    if repeat:
        print('F', measure_key('F', 4*beat_to_sec))
    else:
        print('Dm', measure_key('Dm', 4*beat_to_sec))


def cantonese_song(beat_to_sec, out_part):
    for i in range(2):
        measure_key('C', 2*beat_to_sec)
        measure_key('G', 2*beat_to_sec)
        measure_key('Am', 2*beat_to_sec)
        measure_key('Em', 2*beat_to_sec)
        measure_key('F', 2*beat_to_sec)
        measure_key('G', 2*beat_to_sec)
        measure_key('C', 2*beat_to_sec)
        measure_key('G', 2*beat_to_sec)

    if out_part == 1:
        for i in range(2):
            measure_key('Am', 2*beat_to_sec)
            measure_key('G', 2*beat_to_sec)
            measure_key('F', 2*beat_to_sec)
            measure_key('G', 2*beat_to_sec)
    elif out_part == 2:
        measure_key('Am', 2*beat_to_sec)
        measure_key('Em', 2*beat_to_sec)
        measure_key('F', 2*beat_to_sec)
        measure_key('G', 2*beat_to_sec)
        measure_key('C', 2*beat_to_sec)
        measure_key('G', 2*beat_to_sec)


def calibrate_robot_pos(control=None):
    global control_module, song

    if control is not None:
        control_module = control


    if song == 'billie':
        control_module.action().play_action("key_ready")
        raw_input('Tab Enter to calibrate')
        control_module.action().play_action("Do right")
        raw_input('Tab Enter to ready')
        control_module.action().play_action("key_ready")
        rospy.loginfo('Press start button to begin.')
    elif song == 'knocking' or song == 'beyond':
        control_module.action().play_action("C ready")
        raw_input('Tab Enter to calibrate')
        control_module.action().play_action("C")
        raw_input('Tab Enter to ready')
        control_module.action().play_action("C ready")
        rospy.loginfo('Press start button to begin.')


def play_keys(mode=None, control=None, ratep=None):
    global mode_change, control_module, rate, udp, ik_config, with_ik

    if mode is not None and control is not None and ratep is not None:
        mode_change = mode
        control_module = control
        rate = ratep

    repeat = False

    bpm = 60 # 78
    t1 = 0
    vars = None
    if song == 'knocking':     
        bpm = 72 # Knokin' on heavens door beat (original is 69)
    elif song == 'beyond':
        bpm = 70 # Cantonese song beat
    else:
        with open(ik_config, 'r') as fp:
            vars = json.load(fp)

    if udp == '1':
        client('192.168.31.11', 'ready')

    beat_to_sec = 60./bpm

    while not rospy.is_shutdown() and not mode_change:
        if song == 'billie':
            if with_ik:
                for i in range(2):  
                    billie_jean_ik(vars, bpm)
                
                raw_input('Tab Enter to repeat')
            else:
                avg, t1, t2 = 0, 0, 0
                for i in range(2):
                    if i == 0:
                        t1 = bass_billie_jean() * 2
                    else:
                        t2 = bass_billie_jean() * 2
                      
                chord_ready = time()
                control_module.action().play_action("Dm billie ready")
                while(time() - chord_ready < (t1 + t2)*4. ):
                    sleep(0.001)
                
                measure_key('Dm billie', 1.5*t1)
                measure_key('Em billie', 1.*t1)
                sleep(1.5*t1- 0.219)
                measure_key('F billie', 1.5*t2)
                measure_key('Em billie', 1.*t2)
                sleep(1.5*t2- 0.219)
                control_module.action().play_action("key_ready")
                
                raw_input('Tab Enter to repeat')
                
                
        elif song == 'knocking':
            for i in range(18):
                knokin_on(beat_to_sec, repeat)
                repeat = not(repeat)
            exit()
        elif song == 'beyond':
            for i in range(6):
                print('ITERATION {} ---------------------'.format(i))
                if i == 0 or i == 4 or i == 5:
                    cantonese_song(beat_to_sec, 0)
                elif i == 1 or i == 3:
                    cantonese_song(beat_to_sec, 1)
                elif i == 2:
                    cantonese_song(beat_to_sec, 2)
            exit()



# INVERSE KINEMATICS STUFF ------------------------------------------

def billie_jean_ik(config, beat_to_sec):
    measure_ik('r', config['Re'], beat_to_sec/2)
    measure_ik('l', config['La'], beat_to_sec/2)
    measure_ik('r', config['Do'], beat_to_sec/2)
    measure_ik('r', config['Re'], beat_to_sec/2)
    measure_ik('r', config['Do'], beat_to_sec/2)
    measure_ik('l', config['La'], beat_to_sec/2)
    measure_ik('l', config['Sol'], beat_to_sec/2)
    measure_ik('l', config['La'], beat_to_sec/2)


def measure_ik(arm, key, expected):
    t1 = time()
    inv_kin.move_arm(arm, key['ready'])
    sleep(0.04)
    inv_kin.move_arm(arm, key['play'])
    sleep(0.04)

    while(time()-t1 < expected-0.04):
        sleep(0.001)
    inv_kin.move_arm(arm, key['ready'])
    sleep(0.04)

    return time() - t1


def align_hand():
    # visual servoing
    hand_center = keys.hand.y + keys.hand.height/2

    new_angle = 0

    while keys.hand.y != -1 and keys.blue.y > hand_center:        
        print('Blue Y = {0}\tHand Y = {1}'.format(keys.blue.y, hand_center))
        new_angle = inv_kin.move_r_shoulder()
        sleep(1) # important!
        print('New angle:', new_angle)
        hand_center = keys.hand.y + keys.hand.height/2
    
    print('Hand aligned!')
    return new_angle


def calculate_ik_values():
    global ik_config
    vars = None
    with open(ik_config, 'r') as fp:
        vars = json.load(fp)
    
    # original values (from ready motion)
    length_arm = inv_kin.forearm + inv_kin.upper_arm
    original_left, original_right = inv_kin.get_arm_values()
    print('original right arm values',original_right)

    actions = ['Sol left', 'La left', 'Do right', 'Re right']


    # get shoulder (servo #2) from visual servoing        
    r_shoulder_angle = align_hand()
    l_shoulder_angle = -r_shoulder_angle
    print(l_shoulder_angle)
    raw_input('Press Enter to ready')
    inv_kin.move_joint_to('r_sho_pitch', original_right[0])
    
    # because our arm length is fixed (and we can't move more in the z-axis),
    # we need to calculate a new z-coordinate, no longer (0, desired, length_arm)
    desired = inv_kin.key_width
    new_z = ((length_arm*length_arm) - (desired*desired))**0.5

    # calculate D:
    d_roll = original_right[1]
    d_roll += inv_kin.calc_sho_roll(desired, new_z)
    print('calculated D', d_roll)
    raw_input('Press Enter to calculate A')

    desired = inv_kin.key_width * 3
    new_z = ((length_arm*length_arm) - (desired*desired))**0.5

    # calculate A
    a_roll = original_left[1]
    a_roll += inv_kin.calc_sho_roll(desired, new_z)
    print('calculated A', a_roll)
    raw_input('Press Enter to calculate G')

    desired = inv_kin.key_width * 2
    new_z = ((length_arm*length_arm) - (desired*desired))**0.5

    # calculate G
    g_roll = original_left[1]
    g_roll += inv_kin.calc_sho_roll(desired, new_z)
    print('calculated G', g_roll)
    raw_input('Press Enter to test D ready')
            
    # check values1
    print('Ready for D')
    inv_kin.move_joint_to('r_sho_pitch', original_right[0])
    sleep(1)
    inv_kin.move_joint_to('r_sho_roll', d_roll)
    sleep(1)
    raw_input('Press Enter to test D play ')
    inv_kin.move_joint_to('r_sho_pitch', r_shoulder_angle)
    sleep(1)

    print('Ready for A')
    inv_kin.move_joint_to('l_sho_pitch', original_left[0])
    sleep(1)
    inv_kin.move_joint_to('l_sho_roll', a_roll)
    sleep(1)
    raw_input('Press Enter to test A play ')
    inv_kin.move_joint_to('l_sho_pitch', l_shoulder_angle)
    sleep(1)

    print('Ready for G')
    inv_kin.move_joint_to('l_sho_pitch', original_left[0])
    sleep(1)
    inv_kin.move_joint_to('l_sho_roll', g_roll)
    sleep(1)
    raw_input('Press Enter to test G play ')
    inv_kin.move_joint_to('l_sho_pitch', l_shoulder_angle)
    sleep(1)

    # organize for return

    vars['Do']['ready'] = original_right
    vars['Do']['play'] = [r_shoulder_angle, original_right[1], 0]

    vars['Re']['ready'] = [original_left[0], d_roll, 0]
    vars['Re']['play'] = [r_shoulder_angle, d_roll, 0]

    vars['La']['ready'] = [original_left[0], a_roll, 0]
    vars['La']['play'] = [l_shoulder_angle, a_roll, 0]

    vars['Sol']['ready'] = [original_left[0], g_roll, 0]
    vars['Sol']['play'] = [l_shoulder_angle, g_roll, 0]


    with open(ik_config, 'w') as fp:
        c = json.dumps(
            vars, sort_keys=True, indent=2)
        c = c.replace('"##<', '').replace('>##"', '')
        fp.write(c)


# MAIN -------------------------------------------------------------

if __name__ == '__main__':

    # Initalize global variables
    robot_mode = RobotMode.READY
    mode_change = False
    SPIN_RATE = 0.5
    event = 'piano_player'
    keys = Keys()

    robot_size = rospy.get_param('piano_player_node/size')
    song = rospy.get_param('piano_player_node/song')
    udp = rospy.get_param('piano_player_node/udp')
    with_ik = rospy.get_param('piano_player_node/ik') == '1'

    # set_variables() # Important !

    # Wait for other nodes to launch
    wait_for_node('/op3_manager')
    wait_for_node('/piano_detector_node')

    # Initialze ROS node
    rospy.init_node('piano_player_node')
    rate = rospy.Rate(SPIN_RATE)

    # Initialize OP3 ROS generic subscriber
    button_sub = rospy.Subscriber('/robotis/open_cr/button', String, button_callack)

    blue_sub = rospy.Subscriber('/iros_vision_node/blue', ObjectCoords,
                               lambda msg: keys.blue.update(msg.x, msg.y, msg.width, msg.height, msg.area, msg.angle))

    hand_sub = rospy.Subscriber('/iros_vision_node/hand', ObjectCoords,
                               lambda msg: keys.hand.update(msg.x, msg.y, msg.width, msg.height, msg.area, msg.angle))

    # Initializing modules
    control_module = ControlModule('iros2019', robot_size)

    # ik config
    ik_config = os.path.join(rospkg.get_pkg_dir('events'), 'config/iros2019/piano_values.json')

    inv_kin = InverseKinematics()
    # Enabling walking module
    sleep(10)

    calibrate_robot_pos()

    frame = np.zeros((50,50,3), np.uint8)

    if with_ik:
        cal = raw_input('Calibrate? (y/n)')
        if cal == 'y':
            calculate_ik_values()
            sleep(10)
            control_module.action().play_action("key_ready")
            sleep(1)
        rospy.loginfo('Press start button to begin.')

    while not rospy.is_shutdown():        

        if mode_change:
            mode_change = False

            if robot_mode == RobotMode.READY:
                rospy.loginfo('Resetting OP3. Press start button to begin.')

                # control_module.walking().stop()
                control_module.head().move_head(pan_position=0, tilt_position=-0.8)

            elif robot_mode == RobotMode.RUNNING:
                rospy.loginfo('Starting piano player event.')

                play_keys()
                

        rate.sleep()

