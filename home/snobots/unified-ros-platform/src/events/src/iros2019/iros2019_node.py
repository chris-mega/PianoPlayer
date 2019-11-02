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
from inverse_kinematics import InverseKinematics
import math


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

#===============================================
# KNOCKING ON HEAVENS DOOR
#=================================================


def knocking_on(time_pased):
    bpm = 72
    beats_to_sec = 60./bpm
    time_mario = time() - time_pased
    sleep(0.2) #THIS SLEEP IS IMPORTANT FOR OP-3 AND POLARIS TO PLAY IN SYNC!!!
    print("TIME PASSED SINCE MESSAGE RECEIVED ", str(time_mario))
    
    print("INTROOOOOOOOOOOOOO")
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
    control_module.action().play_action("Crash") 
    exit()
#------------------------------------------------


def chorus(beats_to_sec):
    for i in range(16): 
        play_timed_motion(beats_to_sec, "Ride-Kick")
        play_timed_motion(beats_to_sec, "Ride")
        play_timed_motion(beats_to_sec, "Snare-Ride")
        play_timed_motion(beats_to_sec, "Ride")
        

def intro_half_verse(beats_to_sec):
    for i in range(8):
        play_timed_motion(beats_to_sec, "HH-Kick")
        play_timed_motion(beats_to_sec, "HH")
        play_timed_motion(beats_to_sec, "HH-Snare")
        play_timed_motion(beats_to_sec, "HH")


def verse(beats_to_sec):
    for i in range(16):
        play_timed_motion(beats_to_sec, "HH-Kick")
        play_timed_motion(beats_to_sec, "HH")
        play_timed_motion(beats_to_sec, "HH-Snare")
        play_timed_motion(beats_to_sec, "HH") 

#=======================================================
# CANTONESE SONG
#=====================================================

def canton_song():
    bpm = 70
    beats_to_sec = 60./bpm
    sleep(0.3) #THIS SLEEP IS IMPORTANT FOR OP-3 AND POLARIS TO PLAY IN SYNC!!! PREV VALUE WAS 0.2
    
    print("INTROOOOOOOOOOOOOOOOOOO")
    canton_verse(beats_to_sec/2)

    print("FFFFFFFFIRST VERSEEEEEEEEEEE")
    canton_verse(beats_to_sec/2)
    
    print("BRIDGEEEE!")
    bridge(beats_to_sec/2, False)
    
    print("CHOOOOOORUS")
    chorus_canton(beats_to_sec/2)

    print("POST CHORUS")
    post_chorus(beats_to_sec/2)

    print("SEEEECOND VERSEEEEEEEEEEE")
    canton_verse(beats_to_sec/2)

    print("SEEEECOND BRIDGEEEE")
    bridge(beats_to_sec/2, False)

    print("SECOND CHOOOOOORUS")
    chorus_canton(beats_to_sec/2)

    # print("POST CHOOOOOORUS")
    # post_chorus(beats_to_sec/2)

    print("THIRD   CHOOOOOORUS")
    chorus_canton(beats_to_sec/2)

    print("FINISHED")
    control_module.action().play_action("Crash") 
    exit()


#===== helper methods  


def chorus_canton(beats_to_sec):
    for i in range(0,2):
        for j in range(0, 8):
            if j == 7:
                play_timed_motion(beats_to_sec, "HH-Kick")
                play_timed_motion(beats_to_sec, "HH")
                play_timed_motion(beats_to_sec, "HH-Snare")
                play_timed_motion(beats_to_sec, "Crash-Kick")
            else:
                play_timed_motion(beats_to_sec, "HH-Kick")
                play_timed_motion(beats_to_sec, "HH")
                play_timed_motion(beats_to_sec, "HH-Snare")
                if j == 2 or j == 6:
                    play_timed_motion(beats_to_sec, "HH-Kick")
                else: 
                    play_timed_motion(beats_to_sec, "HH")
        print("Finished one line")


def canton_verse(beats_to_sec):
    for i in range(0,2):
        for j in range(0, 8):
            if j == 7:
                play_timed_motion(beats_to_sec, "HH-Kick")
                play_timed_motion(beats_to_sec, "HH")
                play_timed_motion(beats_to_sec, "HH-Snare")
                play_timed_motion(beats_to_sec, "HH-Snare")
            else:
                play_timed_motion(beats_to_sec, "HH-Kick")
                play_timed_motion(beats_to_sec, "HH")
                play_timed_motion(beats_to_sec, "HH-Snare")
                if j == 2 or j == 6:
                    play_timed_motion(beats_to_sec, "HH-Kick")
                else: 
                    play_timed_motion(beats_to_sec, "HH")
        print("Finished one line")


#before the chorus
def bridge(beats_to_sec, after_chorus):
    for i in range(0,8):
        if i == 7:
            if after_chorus and i is 0: #going from chorus to bridge the shifft from crash to ridfe is very violent, this prevents it
                play_timed_motion(beats_to_sec, "HH-Kick")
            else:
                play_timed_motion(beats_to_sec, "Ride-Kick")
            play_timed_motion(beats_to_sec, "Ride")
            play_timed_motion(beats_to_sec, "Snare-Ride")
            play_timed_motion(beats_to_sec, "Snare-Ride")
        else:
            play_timed_motion(beats_to_sec, "Ride-Kick")
            play_timed_motion(beats_to_sec, "Ride")
            play_timed_motion(beats_to_sec, "Snare-Ride")
            if i == 2 or i == 6:
                play_timed_motion(beats_to_sec, "Ride-Kick")
            else: 
                play_timed_motion(beats_to_sec, "Ride")

#before the chorus
def post_chorus(beats_to_sec):
    for i in range(0,6):
        if i == 7:
            if i is 0: #going from chorus to bridge the shifft from crash to ridfe is very violent, this prevents it
                play_timed_motion(beats_to_sec, "HH-Kick")
            else:
                play_timed_motion(beats_to_sec, "Ride-Kick")
            play_timed_motion(beats_to_sec, "Ride")
            play_timed_motion(beats_to_sec, "Snare-Ride")
            play_timed_motion(beats_to_sec, "Snare-Ride")
        else:
            play_timed_motion(beats_to_sec, "Ride-Kick")
            play_timed_motion(beats_to_sec, "Ride")
            play_timed_motion(beats_to_sec, "Snare-Ride")
            if i == 2 or i == 6:
                play_timed_motion(beats_to_sec, "Ride-Kick")
            else: 
                play_timed_motion(beats_to_sec, "Ride")




#call this function instead of the sleep between the intro and the first verse in the cantonese song!
def non_sleep_cantonese(beats_to_sec):
    play_timed_motion(beats_to_sec, "HH-Kick")
    play_timed_motion(beats_to_sec, "HH")
    play_timed_motion(beats_to_sec, "HH-Snare")
    play_timed_motion(beats_to_sec, "HH")


#==================================
# CANTONESE SONG END
#==========================================

def play_timed_motion(beats_to_sec, motion):
    t1 = time()
    control_module.action().play_action(motion)
    while(time() - t1 < beats_to_sec):
        sleep(0.0001)
    print("******time passed since start  ", time() - t1 )



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
    calibrate_motion('Crash-Kick', sleep_time, caliration_iterations)
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
def play_drums(song, udp):
    global control_module

    if udp == "1":
        raw_input("PRESS ENTER TO WAIT FOR MESSAGE FROM OP-3")
        print("WAITING FOR MESAGE") #wait for message from op-3
        server('192.168.31.11')
    elif udp == "0":
        if song == "0":
            raw_input("PRESS ENTER TO START KNOCKING ON HEAVENS DOOR SONG (NO MSG OP3)")
        else:
            raw_input("PRESS ENTER TO START CANTONESE SONG (NO MSG OP3)")
        
        #count before starting the song
        for i in range(0,4):
            control_module.action().play_action("HH")
            sleep(0.3)



    t1 = time()
    if song == "0":
        knocking_on(t1) # 1 means play knocking on hevavens door
    elif song == "1":
        canton_song()
    else:
        print("Song is not defined!")
        exit()




if __name__ == '__main__':

    # Initalize global variables
    robot_mode = RobotMode.READY
    mode_change = False
    SPIN_RATE = 0.5
    event = 'iros2019'

    robot_size = rospy.get_param('iros2019_node/size')

    # Wait for other nodes to launch
    wait_for_node('/op3_manager')
    # wait_for_node('/vision_node')

    # Initialze ROS node
    rospy.init_node('iros2019_node')
    rate = rospy.Rate(SPIN_RATE)

    # Initialize OP3 ROS generic subscriber
    button_sub = rospy.Subscriber('/robotis/open_cr/button', String, button_callack)


    # Initializing modules
    control_module = ControlModule(event, robot_size)

    # Enabling walking module
    sleep(10)

    inv_kin = InverseKinematics()

    
    if robot_size == 'adult':
        drum_calibration = rospy.get_param('iros2019_node/drum_calibration') #check if we are gonna do sound and position check before running the program
        udp = rospy.get_param('iros2019_node/udp')
        if drum_calibration == "1":
            print("Drum calibration enabled")
            calibrate_drums()


        robot_mode = RobotMode.RUNNING
        mode_change = True
        control_module.action().play_action("drums_ready")
        sleep(1)

        joints = inv_kin.joint_positions
        for jo in joints:
            joints[jo] = math.degrees(joints[jo])
            print(jo, joints[jo])

        

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
                    song = rospy.get_param('iros2019_node/song')
                    play_drums(song, udp)
                elif robot_size == 'kid':
                    play_keys(mode_change, control_module, rate)

        rate.sleep()


