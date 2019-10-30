#!/usr/bin/env python

from enum import Enum
from time import sleep, time

import rosnode
import rospy
from std_msgs.msg import String

from control_modules.control_module import ControlModule
from vision.msg import PositionArea

from piano_player_node import play_keys, calibrate_robot_pos

import roslib.packages as rospkg
import os
import json
import cv2 as cv
from udp_connect import server


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

# -------------------------------
#function to play the whole song
def knocking_on():
    bpm = 72
    beats_to_sec = 60./bpm

    intro_half_verse(beats_to_sec/2)
    print("FIIIIIIIIIIIIIIIIIIIRST VERSEEEEEEEEEEE")
    verse(beats_to_sec/2) #first verse
    print("CHOOOOOOOOOOOOOOOOOOOOOOOOOORUS")
    chorus(beats_to_sec/2)
    print("VEEEEEEEEEEEEEEEEEEEEEEEEEERSE")
    verse(beats_to_sec/2)
    print("CHOOOOOOOOOOOOOOOOOOOOOOOOOORUS")
    chorus(beats_to_sec/2)
    print("**********************************")
    control_module.action().play_action("Ride")
    sleep(0.2)
    control_module.action().play_action("Ride")
    sleep(0.2)
    exit()
#------------------------------------------------


def chorus(beats_to_sec):
    for i in range(16):

        t1 = time()
        control_module.action().play_action("Ride-Kick")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 ) 

        t1 = time()
        control_module.action().play_action("Ride")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 ) 

        t1 = time()
        control_module.action().play_action("Snare-Ride")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 ) 

        t1 = time()
        control_module.action().play_action("Ride")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 ) 
        

def intro_half_verse(beats_to_sec):
    for i in range(8):

        t1 = time()
        control_module.action().play_action("HH-Kick")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 ) 

        t1 = time()    
        control_module.action().play_action("HH")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 ) 

        t1 = time()
        control_module.action().play_action("HH-Snare")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 ) 

        t1 = time()
        control_module.action().play_action("HH")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 ) 


def verse(beats_to_sec):
    for i in range(16):
        t1 = time()
        control_module.action().play_action("HH-Kick")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 )   

        t1 = time()
        control_module.action().play_action("HH")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 )   

        t1 = time()
        control_module.action().play_action("HH-Snare")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 )   

        t1 = time()
        control_module.action().play_action("HH")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 )   


def test_time():
    bpm = 72
    beats_to_sec = 60./bpm
    while True:
        t1 = time()
        control_module.action().play_action("HH-Kick")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 )        


        t1 = time()
        control_module.action().play_action("HH")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 )
        
        t1 = time()
        control_module.action().play_action("HH-Snare")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)   
        print("***********************time passed since start  ", time() - t1 )

        t1 = time()
        control_module.action().play_action("HH")
        while(time() - t1 < beats_to_sec):
            sleep(0.0001)
        print("***********************time passed since start  ", time() - t1 )



def calibrate_drums():
    global control_module
    control_module.action().play_action("drums_ready")
    #test_time() #############################################just for testing of time in motions
    sleep_time = 0.5
    caliration_iterations = 6
    raw_input("******************Press ENTER to begin\n")

    calibrate_motion('HH-Kick', sleep_time, caliration_iterations)
    calibrate_motion('HH-Snare', sleep_time, caliration_iterations)
    calibrate_motion('Snare-Ride', sleep_time, caliration_iterations)


    print("******************Press enter to FINISH calibration")
    print("******************Enter 'r' to REPEAT calibration\n")
    repeat = raw_input()

    if repeat == 'r':
        calibrate_drums()
    
#calibrates one motion (for example kick)
def calibrate_motion(motion, sleep_time, calibration_iterations):
    print("CALIBRATING ", motion)
    while True:
        for  i in range(0, calibration_iterations):
            control_module.action().play_action(motion)
            sleep(sleep_time)
        repeat = raw_input("******************Press 'r' to repeat. Press anything else to continue\n")
        if repeat != 'r':
            break
    



##entry place for adult size
def play_drums():
    global control_module
    server('192.168.31.4')
    knocking_on() #play the song




if __name__ == '__main__':

    # Initalize global variables
    robot_mode = RobotMode.READY
    mode_change = False
    SPIN_RATE = 0.5
    event = 'iros2019'

    robot_size = rospy.get_param('iros2019_node/size')


    # set_variables() # Important !

    # Wait for other nodes to launch
    wait_for_node('/op3_manager')
    # wait_for_node('/vision_node')

    # Initialze ROS node
    rospy.init_node('iros2019_node')
    rate = rospy.Rate(SPIN_RATE)

    # Initialize OP3 ROS generic subscriber
    button_sub = rospy.Subscriber('/robotis/open_cr/button', String, button_callack)

    # red_sub = rospy.Subscriber('/vision_node/keys', PositionArea,
    #                            lambda msg: line.update(msg.x, msg.y, msg.position, msg.angle, msg.area))

    # Initializing modules
    control_module = ControlModule(event, robot_size)

    # Enabling walking module
    sleep(10)
    if robot_size == 'adult':
        drum_calibration = rospy.get_param('iros2019_node/drum_calibration') #check if we are gonna do sound and position check before running the program
        if drum_calibration == "1":
            print("Drum calibration enabled")
            calibrate_drums()


        robot_mode = RobotMode.RUNNING
        mode_change = True
        control_module.action().play_action("drums_ready")
        sleep(1)
        control_module.head().move_head(pan_position=0, tilt_position=-1.0)
        raw_input('Press Enter to START SONG')
    elif robot_size == 'kid':
        calibrate_robot_pos(control_module)

    while not rospy.is_shutdown():
        if mode_change:
            mode_change = False

            if robot_mode == RobotMode.READY:
                rospy.loginfo('Resetting OP3. Press start button to begin.')

                # control_module.walking().stop()
                control_module.head().move_head(pan_position=0, tilt_position=-0.8)
            
            elif robot_mode == RobotMode.RUNNING:
                rospy.loginfo('Starting iros event.')

                if robot_size == 'adult':
                    play_drums()
                elif robot_size == 'kid':
                    play_keys(mode_change, control_module, rate)

        rate.sleep()


