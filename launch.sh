#! /bin/bash
# File used to launch all requirements at once for the simple distance/communication experiment

echo "LAUNCH: Giving Rights to Read SPI and execute scripts"

sudo chmod a+rw /dev/ttyACM0
sudo chmod a+rw /dev/ttyACM1

sudo chmod +x src/experiments_utils/scripts/sensor_measurement.py
sudo chmod +x src/experiments_utils/scripts/odometry_republisher.py

echo "LAUNCH: Setting up Variables"

initiator="/dev/ttyACM0"
responder="/dev/ttyACM1"

echo "LAUNCH: ROS Compilation"

catkin_make
source devel/setup.bash

echo "LAUNCH: Launching ROS Nodes"
roslaunch experiments_utils read_sensors.launch

echo "LAUNCH: Application Ended"