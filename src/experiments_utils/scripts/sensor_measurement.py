#!/usr/bin/python3
import datetime
import signal
import sys

import serial
import rospy
import tqdm
from experiments_utils.msg import Manage, Distance, Distances

from utils import DWM, parse_distances


def talker():
    module_type = sys.argv[1]
    module_port = sys.argv[2]

    rospy.init_node('sensor_measurement', anonymous=True)

    # Publish the read distances
    publisher = rospy.Publisher(f'/{module_type}/sensor_read', Distances, queue_size=1000)

    try:
        module = DWM(port=module_port)
    except serial.SerialException as e:
        raise ValueError("Couldn't open serial port", e)

    while not rospy.is_shutdown():
        line: str = module.read()

        msg, faulty_frame = parse_distances(line)

        if not faulty_frame:
            publisher.publish(msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
