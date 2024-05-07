#!/usr/bin/python3
import datetime
import signal
import sys

import serial
import rospy
import tqdm
from experiments_utils.msg import Manage, Distance, Distances

from utils import DWM, parse_distances

stop = False


def callback(msg):
    global stop

    if msg.stop:
        stop = True


def signal_handler(sig, frame):
    global stop
    stop = True


def talker():
    simulation = True if sys.argv[1] == "True" else False

    module_type = sys.argv[2]
    module_port = sys.argv[3]

    output_dir = sys.argv[4]
    output_file = sys.argv[5]

    # if output file already exists, raise an error
    if not simulation:
        try:
            with open(f"{output_dir}/{output_file}", "x") as f:
                pass
        except FileExistsError:
            raise ValueError("ERROR: Output file already exists, stopping experiment to avoid deleting data")

    rospy.init_node('sensor_measurement', anonymous=True)

    # Subscribe to the manager command (to stop the node when needed)
    rospy.Subscriber('manager_command', Manage, callback)

    # Publish the read distances
    publisher = rospy.Publisher(
        f'{module_type}/sensor_read',
        Distances,
        queue_size=1000
    )

    if not simulation:
        try:
            module = DWM(port=module_port)
        except serial.SerialException as e:
            raise ValueError("Couldn't open serial port", e)

        with open(f"{output_dir}/{output_file}", "w+") as f:
            start = datetime.datetime.now()

            while not rospy.is_shutdown() and not stop:
                line: str = module.read()

                f.write(f"{(datetime.datetime.now() - start).total_seconds()}&{line}")
                msg, faulty_frame = parse_distances(line)

                if not faulty_frame:
                    publisher.publish(msg)
    else:
        with open(f"{output_dir}/{output_file}", "r") as f:
            # Read file, line by line and output only if timestamp is reached
            lines = f.readlines()

        start = datetime.datetime.now()

        for line in tqdm.tqdm(lines):
            timestamp, distances = line.split("&")

            msg, faulty_frame = parse_distances(distances)
            if not faulty_frame:
                # While not the right moment, do nothing
                while (datetime.datetime.now() - start).total_seconds() < float(timestamp):
                    pass

                publisher.publish(msg)
            if stop:
                break


if __name__ == '__main__':
    # Set signal handler
    signal.signal(signal.SIGINT, signal_handler)

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
