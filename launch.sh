#! /bin/bash
# File used to launch all requirements at once for the simple distance/communication experiment

echo "LAUNCH: Giving Rights to Read SPI"

sudo chmod a+rw /dev/ttyACM0
sudo chmod a+rw /dev/ttyACM1

initiator="/dev/ttyACM0"
responder="/dev/ttyACM1"

echo "LAUNCH: ROS Compilation"

catkin_make
source devel/setup.bash

echo "LAUNCH: Launching ROS Nodes"
roslaunch communication communication_nodes.launch

echo "LAUNCH: Application Ended"