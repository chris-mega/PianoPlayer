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
from udp_connect import client, server

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


def cantonese_song(beat_to_sec, out_part, end=False):
    for i in range(2):
        measure_key('C', 2*beat_to_sec)
        measure_key('G', 2*beat_to_sec)
        measure_key('Am', 2*beat_to_sec)
        measure_key('Em', 2*beat_to_sec)
        measure_key('F', 2*beat_to_sec)
        measure_key('G', 2*beat_to_sec)
        
        if i == 0 or (i == 1 and not end):
            measure_key('C', 2*beat_to_sec)
            measure_key('G', 2*beat_to_sec)
        elif end and i == 1:
            measure_key('C', 4*beat_to_sec)
    
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
        sleep(1)
        skip = raw_input('Press Enter to wave (s for skip)')
        play_sound('Oscar_Intro1.mp3')
        
        if skip != 's':
            control_module.action().play_action("op3_wave")

        control_module.action().play_action("key_ready")
        play_sound('Oscar_Intro2.mp3')
    
        if skip != 's':
            control_module.action().play_action("Do right")
            control_module.action().play_action("key_ready")
    elif song == 'knocking' or song == 'beyond':
        control_module.action().play_action("C ready")
        if song == 'knocking':
            raw_input('Press enter')
            play_sound('Oscar_IntroPolaris3.mp3')
            sleep(1)
            play_sound('Oscar_StartSong.mp3')

        raw_input('Tab Enter to calibrate')
        control_module.action().play_action("C")
        raw_input('Tab Enter to ready')
        control_module.action().play_action("C ready")

        

        rospy.loginfo('Press start button to begin.')


def play_keys(mode=None, control=None, ratep=None):
    global mode_change, control_module, rate, udp, ik_config, with_ik, music_path

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
        client('192.168.31.9', 'ready')

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
                sleep(1) 
                raw_input('Tab Enter to repeat')
                play_sound('Oscar_IntroPolaris.mp3')
                sleep(1)
                play_sound('Oscar_IntroPolaris2.mp3')
                exit()
                
                
        elif song == 'knocking':
            for i in range(22): # 18, 22
                if i == 18 and udp == '1':
                    client('192.168.31.9', 'continue')
                # if i == 14 and udp == '1':
                #     server('192.168.31.8')
                knokin_on(beat_to_sec, repeat)
                repeat = not(repeat)
            
            exit()
        elif song == 'beyond':
            for i in range(6):
                print('ITERATION {} ---------------------'.format(i))
                if i == 0 or i == 4:
                    cantonese_song(beat_to_sec, 0)
                elif i == 5:
                    cantonese_song(beat_to_sec, 0, True)
                elif i == 1 or i == 3:
                    cantonese_song(beat_to_sec, 1)
                elif i == 2:
                    cantonese_song(beat_to_sec, 2)

            raw_input('Press enter to finish')
            play_sound('Oscar_FinalThanks.mp3')
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
    sleep(0.1)
    inv_kin.move_arm(arm, key['play'])
    sleep(0.1)
    inv_kin.move_arm(arm, key['ready'])
    sleep(0.1)
    
    return time() - t1


def align_hand():
    # visual servoing
    hand_center = keys.hand.y + keys.hand.height/2

    goal = keys.blue.y + keys.blue.height/4

    ideal_x = 38
    error = 3

    move_x = 0
    move_y = 0

    # move left or right
    if keys.hand.x - keys.blue.x < ideal_x - error:
        while keys.hand.x - keys.blue.x < ideal_x - error:
            print('Blue X = {0}\tHand X = {1}'.format(keys.blue.x, keys.hand.x))
            inv_kin.move_r_shoulder_roll_right()
            move_x += 0.017
            sleep(1) # important!
            print('New angle:', move_x)
            hand_center = keys.hand.y + keys.hand.height/2
    elif keys.hand.x - keys.blue.x > ideal_x + error:
        while keys.hand.x - keys.blue.x > ideal_x + error:
            print('Blue X = {0}\tHand X = {1}'.format(keys.blue.x, keys.hand.x))
            inv_kin.move_r_shoulder_roll_left()
            move_x -= 0.017
            sleep(1) # important!
            print('New angle:', move_x)
            hand_center = keys.hand.y + keys.hand.height/2

    raw_input('Press enter to continue')

    # move down
    while keys.hand.y != -1 and goal > hand_center:        
        print('Blue Y = {0}\tHand Y = {1}'.format(keys.blue.y, hand_center))
        move_y = inv_kin.move_r_shoulder()
        sleep(1) # important!
        print('New angle:', move_y)
        hand_center = keys.hand.y + keys.hand.height/2
    
    print('Hand aligned!')

    return move_x, move_y


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
    move_x, r_shoulder_angle = align_hand()

    original_right[1] += move_x
    original_left[1] += move_x

    l_shoulder_angle = -r_shoulder_angle
    print('r shoulder, l_shoulder', r_shoulder_angle, l_shoulder_angle)
    inv_kin.move_joint_to('r_sho_pitch', original_right[0])
    
    # because our arm length is fixed (and we can't move more in the z-axis),
    # we need to calculate a new z-coordinate, no longer (0, desired, length_arm)
    desired = inv_kin.key_width
    new_z = ((length_arm*length_arm) - (desired*desired))**0.5

    # calculate D:
    d_roll = original_right[1]
    d_roll += inv_kin.calc_sho_roll(desired, new_z)
    print('calculated D', d_roll)

    desired = inv_kin.key_width * 3
    new_z = ((length_arm*length_arm) - (desired*desired))**0.5

    # calculate A
    a_roll = original_left[1]
    a_roll += inv_kin.calc_sho_roll(desired, new_z)
    print('calculated A', a_roll)

    desired = inv_kin.key_width * 2
    new_z = ((length_arm*length_arm) - (desired*desired))**0.5

    # calculate G
    g_roll = original_left[1]
    g_roll += inv_kin.calc_sho_roll(desired, new_z)
    print('calculated G', g_roll)

    # organize for return

    vars['Do']['ready'] = original_right
    vars['Do']['play'] = [r_shoulder_angle, original_right[1], 0]

    vars['Re']['ready'] = [original_right[0], d_roll, 0]
    vars['Re']['play'] = [r_shoulder_angle, d_roll, 0]

    vars['La']['ready'] = [original_left[0], a_roll, 0]
    vars['La']['play'] = [l_shoulder_angle, a_roll, 0]

    vars['Sol']['ready'] = [original_left[0], g_roll, 0]
    vars['Sol']['play'] = [l_shoulder_angle, g_roll, 0]

    # check values1

    raw_input('Press enter to prepare for test! (be ready to kill servos if something went wrong!!!)')

    inv_kin.move_arm('r', vars['Re']['ready'])
    sleep(1)
    inv_kin.move_arm('r', vars['Re']['play'])
    sleep(1)

    inv_kin.move_arm('r', vars['Do']['ready'])
    sleep(1)
    inv_kin.move_arm('r', vars['Do']['play'])
    sleep(1)

    inv_kin.move_arm('r', original_right)
    sleep(1)
    inv_kin.move_arm('l', vars['La']['ready'])
    sleep(1)
    inv_kin.move_arm('l', vars['La']['play'])
    sleep(1)

    inv_kin.move_arm('l', vars['Sol']['ready'])
    sleep(1)
    inv_kin.move_arm('l', vars['Sol']['play'])
    sleep(1)

    with open(ik_config, 'w') as fp:
        c = json.dumps(
            vars, sort_keys=True, indent=2)
        c = c.replace('"##<', '').replace('>##"', '')
        fp.write(c)

    inv_kin.move_arm('l', original_left)
    sleep(1)
    inv_kin.move_arm('r', original_right)
    sleep(1)


def play_sound(path):
    # msg = String()
    # msg.data = path
    # play_sound_pub.publish(msg)
    # print('playing!')
    # s = 'mpg123 Desktop/{}'.format(path)
    s = 'mpg123 ../Desktop/{}'.format(path)
    print(s)
    os.system(s)





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

    play_sound_pub = rospy.Publisher('/play_sound_file', String, queue_size=1)

    music_path = os.path.join(rospkg.get_pkg_dir('events'), 'static')

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
        while cal == 'y':
            calculate_ik_values()
            sleep(1)
            control_module.action().play_action("key_ready")
            sleep(1)
            cal = raw_input('Calibrate again? (y/n)')
        
        rospy.loginfo('Press start button to begin.')

    if song == 'billie':
        skip = raw_input('Press enter for what do you want (s for skip)')
        play_sound('Oscar_Intro3.mp3')
        sleep(3)
        play_sound('Oscar_soundcheck1.mp3')
        control_module.action().play_action("op3_no")
        option = raw_input('Press 0 for no, 1 for yes')
        if option == '0':
            play_sound('Oscar_soundcheck2.mp3')
        else:
            play_sound('Oscar_soundcheck2_1.mp3')


            # control_module.action().play_action("op3_yes")
        # skip = raw_input('Press enter for \'I think I can\' (s for skip)')
        # if skip != 's':
        #     # play_sound('{}/op3_i_can.mp3'.format(music_path))
        #     os.system('espeak \"I think I can play it\" -v en-us')
        #     control_module.action().play_action("op3_yes")
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

