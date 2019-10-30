# PianoPlayer
*Developed by Christian Melendez*

Piano player program for a Robotis-Op3 robot

This was originally an assignment for COMP4180-"Intelligent Mobile Robotics" at University of Manitoba
and to compete in IROS 2019 - Humanoid Application Challenge "Robot magic and music".

Please see `Report.pdf` for a detailed description of the project's development

## Dependencies
Your system must contain the following programs/libraries installed
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Robotis OP3 Framework](http://emanual.robotis.com/docs/en/platform/op3/recovery/#op3-manual-installation)
- [Opencv 3](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)
- [Python 2.7](https://www.python.org/downloads/)
- [numpy](https://www.scipy.org/install.html)
- [pyyaml](https://pypi.org/project/PyYAML/)

## Build
Copy the packages inside `src` into your workspace and type `catkin_make`

## Running
- Run `./piano.sh [song] [udp/ik]`
