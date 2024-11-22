import math
import time

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client
from terasim_cosim.redis_msgs import PlannedPath, VehicleControl


UTM_OFFSET = [-277600 + 102.89, -4686800 + 281.25]


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


def send_user_av_control_wrapper(
    brake_cmd: float = 0.0,
    throttle_cmd: float = 0.0,
    steering_cmd: float = 0.0,
    gear_cmd: int = 0,
    remote_flag: bool = False,
    user_msg: str = "user",
):
    """
    Description: Send the control command to the control AV.
    """

    if brake_cmd < 0.0 or brake_cmd > 1.0:
        raise ValueError("The brake_cmd should be in the range of [0.0, 1.0].")
    if throttle_cmd < 0.0 or throttle_cmd > 1.0:
        raise ValueError("The throttle_cmd should be in the range of [0.0, 1.0].")
    if steering_cmd < -1.0 or steering_cmd > 1.0:
        raise ValueError("The steering_cmd should be in the range of [-1.0, 1.0].")
    if gear_cmd < 0 or gear_cmd > 4:
        raise ValueError("The gear_cmd should be in the range of [0, 4].")
    if throttle_cmd > 0.0 and brake_cmd > 0.0:
        raise ValueError(
            "The throttle_cmd and brake_cmd cannot be both greater than 0."
        )

    if gear_cmd == 1:
        brake_cmd = 0.5
        throttle_cmd = 0.0
        steering_cmd = 0.0

    if gear_cmd == 3:
        brake_cmd = 0.5
        throttle_cmd = 0.0
        steering_cmd = 0.0

    vehicle_control = VehicleControl()

    vehicle_control.header.timestamp = time.time()
    vehicle_control.header.information = user_msg
    vehicle_control.brake_cmd = brake_cmd
    vehicle_control.throttle_cmd = throttle_cmd
    vehicle_control.steering_cmd = steering_cmd
    vehicle_control.gear_cmd = gear_cmd

    # Configure redis key-and data type
    redis_client = create_redis_client(
        key_value_config={VEHICLE_CONTROL: VehicleControl},
        remote_flag=remote_flag,
        pub_channels=[VEHICLE_CONTROL],
        sub_channels=[],
        latency_src_channels=[],
    )

    redis_client.set(VEHICLE_CONTROL, vehicle_control)


def send_user_av_planning_wrapper(
    x_list: list = [],
    y_list: list = [],
    speed_list: list = [],
    orientation_list: list = [],
    remote_flag: bool = False,
    user_msg: str = "user",
):
    """
    Description: Send the planned path to the control AV.
    """

    x_list_center_utm = []
    y_list_center_utm = []
    orientation_list_nef = []

    for i in range(len(x_list)):
        x, y = front_coordinate_to_center_coordinate(x_list[i], y_list[i])
        x, y = sumo_to_utm_coordinate([x, y])
        orientation = sumo_heading_to_orientation(orientation_list[i])

        x_list_center_utm.append(x)
        y_list_center_utm.append(y)
        orientation_list_nef.append(orientation)

    planned_path = PlannedPath()

    planned_path.header.timestamp = time.time()
    planned_path.header.information = user_msg
    planned_path.time_resolution = 0.1
    planned_path.go = 1
    planned_path.x_list = x_list_center_utm
    planned_path.y_list = y_list_center_utm
    planned_path.speed_list = speed_list
    planned_path.orientation_list = orientation_list_nef

    # Configure redis key-and data type
    redis_client = create_redis_client(
        key_value_config={PLANNED_PATH: PlannedPath},
        remote_flag=remote_flag,
        pub_channels=[PLANNED_PATH],
        sub_channels=[],
        latency_src_channels=[],
    )

    redis_client.set(PLANNED_PATH, planned_path)
