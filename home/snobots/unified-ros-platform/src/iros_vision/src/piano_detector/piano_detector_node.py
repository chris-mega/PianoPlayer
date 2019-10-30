#!/usr/bin/env python

# Author: Christian Melendez
# TODO: change ball detection to piano detection 

import roslib.packages as rospkg
import rosnode
import rospy
import os
import json
import math
import cv2 as cv

from vision import Vision, MAIN_WINDOW_NAME, DEBUG
from object_detection import piano
from time import sleep, time
from yaml_parser import Parser


if __name__ == '__main__':
    SPIN_RATE = 10
    rospy.init_node('piano_detector_node')
    rate = rospy.Rate(SPIN_RATE)

    configuration_directory = rospy.get_param(
        'piano_detector_node/configuration_directory')

    file_path = os.path.join(configuration_directory, 'piano_config.yaml')

    with_cascade = True
    
    vision = Vision(file_path)
    ball_cascade = None

    folder = os.getcwd()
    folder = os.path.join(folder, 'src')
  
    
    while not rospy.is_shutdown():
        vision.read_camera()
        vision.create_hsv()

        for objs in vision.color_objects:
            x, y, area, angle = piano(vision.frame, objs)

        marker_cm = (1,5)

        cv.putText(vision.frame, 'selected: {}'.format(vision.selected_object.name),
            (10, 20), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

        cv.imshow(MAIN_WINDOW_NAME, vision.frame)

        cv.imshow(DEBUG, vision.selected_object.debug)

        if cv.waitKey(1) == ord('q'):
            break

    vision.cap.release()
    cv.destroyAllWindows()
    