
import serial

from experiments_utils.msg import Distance, Distances, Odometry, Positions


class DWM:
    def __init__(self, port):
        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = 115200

        self.serial.bytesize = serial.EIGHTBITS
        self.serial.parity = serial.PARITY_NONE
        self.serial.stopbits = serial.STOPBITS_ONE
        self.serial.timeout = 1
        self.serial.write_timeout = 0

        self.open()

    def open(self):
        self.serial.open()

    def close(self):
        self.serial.close()

    def read(self) -> str:
        return str(self.serial.readline(), 'utf-8')

    def write(self, string):
        self.serial.write(str.encode(string, encoding="utf-8"))


def parse_distances(line: str) -> Distances:
    msg = Distances()
    msg.ranges = []

    faulty_frame = False

    infos = line.split(",")

    if len(infos) > 1:  # Means that the split was made (therefore at least one information to process)
        sender_robot = infos[0]
        sender_info = sender_robot.split("=")

        if sender_info[1] == "0:100":
            msg.robot_id = ord(sender_info[0])

        for info in infos[1:]:
            try:
                robot, distance = info.split("=")
                distance, certainty = distance.split(":")

                data = Distance()

                data.other_robot_id = ord(robot)
                data.distance = int(distance)
                data.certainty = int(certainty)

                msg.ranges.append(data)
            except ValueError as e:
                pass

    return msg, faulty_frame


def unparse_distances(msg: Distances) -> str:
    line = f"{chr(msg.robot_id)}=0:100"
    for data in msg.ranges:
        if chr(data.other_robot_id) != chr(msg.robot_id):
            line += f",{chr(data.other_robot_id)}={data.distance}:{data.certainty}"
    return f"{line}"


def parse_positions(line: str) -> Positions:
    msg = Positions()
    msg.odometry_data = []

    faulty_frame = False

    infos = line.split("#")

    if len(infos) > 1:
        for info in infos:
            agent_id, positions, orientations = info.split(":")
            x, y, z = [float(i) for i in positions.split(",")]
            a, b, c, d = [float(j) for j in orientations.split(",")]

            msg.odometry_data.append(
                Odometry(
                    id=ord(agent_id),
                    x=x, y=y, z=z,
                    a=a, b=b, c=c, d=d
                )
            )
    else:
        faulty_frame = True

    return msg, faulty_frame


def unparse_positions(msg: Positions) -> str:
    line = f""
    for position in msg.odometry_data:
        if line:
            line += "#"
        # TODO: add the orientation information
        line += f"{chr(position.id)}:{position.x},{position.y},{position.z}:{position.a},{position.b},{position.c},{position.d}"
    return f"{line}"