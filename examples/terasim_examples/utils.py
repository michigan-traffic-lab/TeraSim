import math
import time

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client
from terasim_cosim.redis_msgs import VehiclePlanning


UTM_OFFSET = [-277600 + 102.89, -4686800 + 281.25, 0.0]


def utm_to_sumo_coordinate(x, y):
    """
    Convert the UTM coordinate to the SUMO coordinate. the input will be a list of [x, y].
    """
    return [
        x + UTM_OFFSET[0],
        y + UTM_OFFSET[1],
    ]


def sumo_to_utm_coordinate(x, y):
    """
    Convert the SUMO coordinate to the UTM coordinate. the input will be a list of [x, y].
    """
    return [
        x - UTM_OFFSET[0],
        y - UTM_OFFSET[1],
    ]


def center_coordinate_to_front_coordinate(x, y, heading, length=5.0):
    """
    Convert the center coordinate to the SUMO coordinate. the input will be a list of {x, y, heading, vx}.
    """
    x = x + math.cos(heading) * 0.5 * length
    y = y + math.sin(heading) * 0.5 * length
    return x, y


def front_coordinate_to_center_coordinate(x, y, heading, length=5.0):
    """
    Convert the SUMO coordinate to the center coordinate. the input will be a list of {x, y, heading, vx}.
    """
    x = x - math.cos(heading) * 0.5 * length
    y = y - math.sin(heading) * 0.5 * length
    return x, y


def sumo_heading_to_orientation(sumo_heading):
    """
    Convert the SUMO heading to orientation.
    """
    radians = math.radians(90 - sumo_heading)
    return math.atan2(math.sin(radians), math.cos(radians))


def orientation_to_sumo_heading(orientation):
    """
    Convert the orientation to SUMO heading.
    """
    degrees = math.degrees(orientation)
    degrees = (degrees + 360) % 360
    degrees = (90 - degrees) % 360
    return degrees


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)

    return [qx, qy, qz, qw]


def send_user_av_planning_wrapper(
    x_list: list = [],
    y_list: list = [],
    speed_list: list = [],
    orientation_list: list = [],
    remote_flag: bool = False,
    user_msg: str = "user",
    redis_key: str = VEHICLE_PLANNING
):
    """
    Description: Send the vehicle planning trajectory to the control AV.
    """

    x_list_center = []
    y_list_center = []
    orientation_list_nef = []

    for i in range(len(x_list)):
        orientation = sumo_heading_to_orientation(orientation_list[i])
        x, y = front_coordinate_to_center_coordinate(x_list[i], y_list[i], orientation)

        x_list_center.append(x)
        y_list_center.append(y)
        orientation_list_nef.append(orientation)

    vehicle_planning = VehiclePlanning()

    vehicle_planning.header.timestamp = time.time()
    vehicle_planning.header.information = user_msg
    vehicle_planning.time_resolution = 0.1
    vehicle_planning.go = 1
    vehicle_planning.x_list = x_list_center
    vehicle_planning.y_list = y_list_center
    vehicle_planning.speed_list = speed_list
    vehicle_planning.orientation_list = orientation_list_nef

    # Configure redis key-and data type
    redis_client = create_redis_client(
        key_value_config={redis_key: VehiclePlanning},
        remote_flag=remote_flag,
        pub_channels=[redis_key],
        sub_channels=[],
        latency_src_channels=[],
    )

    redis_client.set(redis_key, vehicle_planning)
