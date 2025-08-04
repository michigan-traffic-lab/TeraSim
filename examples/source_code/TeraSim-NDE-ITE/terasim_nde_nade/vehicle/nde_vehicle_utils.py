import numpy as np
import math
from terasim.overlay import traci
import attr
import sumolib
from terasim.utils import (
    sumo_coordinate_to_center_coordinate,
    sumo_heading_to_orientation,
)
import json
from enum import Enum
from collections import namedtuple
from scipy.interpolate import interp1d
from loguru import logger

# Define the TrajectoryPoint named tuple
TrajectoryPoint = namedtuple("TrajectoryPoint", ["timestep", "position", "heading"])
from terasim_nde_nade.vehicle.nde_vehicle_utils_cython import *
from terasim.overlay import profile
from typing import List, Tuple, Dict, Any, Optional, Callable
from pydantic import BaseModel, validator
from enum import Enum
import addict

# 6.085993383090184e-05
intersection_cutin_prob = (
    1.118159657654468e-04
    * 0.5
    * 2
    * 1.2
    * 1.5
    * 1.25
    * 0.5
    * 0.5
    * 2
    * 0.7
    * 0.5
    * 1.5
    * 1.3
    * 1.1
    * 1.3
    * 0.85
    * 0.9
    * 0.8
    * 0.9
    * 0.9
    * 1.1
)
# 0.013682468839095184
intersection_neglect_conflict_lead_prob = (
    6.677231589776039e-04
    * 3.86
    * 1
    * 0.7
    * 1.2
    * 1.25
    * 1.56
    * 2
    * 0.5
    * 1.5
    * 2
    * 0.9
    * 0.9
    * 1.3
    * 0.85
    * 1.05
    * 1.1
    * 1.1
    * 0.95
    * 1.1
)
# 0.00015787218253532532
intersection_rearend_prob = (
    2.204741193939959e-04
    * 3.08
    * 2.42
    * 2
    * 0.6
    * 0.8
    * 1.25
    * 1.25
    * 0.5
    * 1.61
    * 0.5
    * 0.9
    * 0.8
    * 0.8
    * 0.5
    * 0.5
    * 1.3
    * 0.85
    * 1.1
)
# 0.009219911404943987
intersection_tfl_prob = (
    0.058291608034515015
    * 0.5
    * 0.5
    * 0.5
    * 1.5
    * 0.9
    * 1.7
    * 2
    * 1.5
    * 0.8
    * 0.8
    * 0.5
    * 1.1
    * 0.9
    * 0.5
    * 1.3
    * 0.85
    * 1.05
    * 1.1
)
# 9.693789338786234e-09
intersection_headon_prob = (
    2.994401291981026e-04
    * 0.2
    * 0.2
    * 0.5
    * 1.5
    * 0.8
    * 1.25
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 1.3
    * 0.85
    * 1.1
)
# 0.0005025411072000003
roundabout_fail_to_yield_prob = (
    1.2830400000000002e-03
    / 2
    * 1.5
    * 0.8
    * 1.25
    * 0.8
    * 0.68
    * 0.8
    * 2
    * 0.5
    * 1.5
    * 0.8
)
# 2.3123152310029683e-05
roundabout_cutin_prob = (
    5.3475398926368317e-05
    / 2
    * 1.17
    * 3.07
    * 1.5
    * 0.8
    * 0.8
    * 0.8
    * 0.8
    * 0.55
    * 2
    * 0.95
    * 0.5
    * 0.5
    * 1.5
)
# 0.00027119028871789013
roundabout_neglect_conflict_lead_prob = (
    1.8780196130730532e-04
    / 2
    * 4.15
    * 2.49
    * 0.7
    * 0.8
    * 1.25
    * 0.8
    * 0.5
    * 0.5
    * 1.05
    * 1.3
    * 0.5
    * 1.5
    * 1.5
    * 0.5
    * 2
    * 1.3
)
# 1.8383121748678325e-10
roundabout_rearend_prob = (
    2.2978902185847895e-05
    * 0.2
    * 0.2
    * 0.5
    * 0.8
    * 0.8
    * 0.8
    * 0.8
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
)
# 9.410654625349324e-06
highway_cutin_prob = (
    5.5883079028671922e-06
    * 1.2
    * 1.5
    * 0.9
    * 0.8
    * 1.25
    * 1.25
    * 0.75
    * 0.5
    * 2
    * 1.5
    * 0.8
    * 0.7
    * 0.8
    * 1.5
    * 1.1
    * 2
    * 1.7
)
# 0.258343354368
highway_rearend_prob = (
    3.47e-2
    * 5
    * 5
    * 0.8
    * 0.8
    * 1.1
    * 1.2
    * 1.25
    * 0.85
    * 0.5
    * 0.5
    * 1.5
    * 0.8
    * 1.2
    * 0.8
    * 1.2
    * 0.8
    * 1.2
    * 2
    * 1.7
)

logger.info(
    f"intersection_cutin_prob: {intersection_cutin_prob}, intersection_neglect_conflict_lead_prob: {intersection_neglect_conflict_lead_prob}, intersection_rearend_prob: {intersection_rearend_prob}, intersection_tfl_prob: {intersection_tfl_prob}, intersection_headon_prob: {intersection_headon_prob}, roundabout_fail_to_yield_prob: {roundabout_fail_to_yield_prob}, roundabout_cutin_prob: {roundabout_cutin_prob}, roundabout_neglect_conflict_lead_prob: {roundabout_neglect_conflict_lead_prob}, roundabout_rearend_prob: {roundabout_rearend_prob}, highway_cutin_prob: {highway_cutin_prob}, highway_rearend_prob: {highway_rearend_prob}"
)


class Command(Enum):
    DEFAULT = "default"
    LEFT = "left"
    RIGHT = "right"
    TRAJECTORY = "trajectory"
    ACCELERATION = "acceleration"
    CUSTOM = "custom"


class NDECommand(BaseModel):
    """
    Represents a command for a vehicle in a Non-Deterministic Environment (NDE).
    if the command is "default", the vehicle will follow the SUMO controlled model, other elements will be ignored
    if the command is "left" or "right", the vehicle will change lane to the left or right, other elements will be ignored
    if the command is "trajectory", the vehicle will follow the future trajectory, which will be predicted according to the current acceleration, other elements will be ignored
    if the command is "acceleration", the vehicle will decelerate to stop using the acceleration element
    """

    command_type: Command = Command.DEFAULT
    acceleration: float = 0.0
    future_trajectory: List[Tuple[float, float]] = []
    prob: float = 1.0
    duration: float = None
    info: Dict[str, Any] = {}
    custom_control_command: Dict[str, Any] = None
    custom_execute_control_command: Callable = None

    @validator("duration", pre=True, always=True)
    def set_duration(cls, v):
        return v if v is not None else traci.simulation.getDeltaT()

    class Config:
        slots = True
        extra = "forbid"


def get_next_lane_edge(net, lane_id):
    origin_lane = net.getLane(lane_id)
    outgoing_lanes = [conn.getToLane() for conn in origin_lane.getOutgoing()]
    outgoing_edges = [lane.getEdge() for lane in outgoing_lanes]
    return outgoing_lanes[0].getID(), outgoing_edges[0].getID()


tls_controlled_lane_set = None
from itertools import chain


def cache_tls_controlled_lane_set():
    global tls_controlled_lane_set
    for tls_id in traci.trafficlight.getIDList():
        controlled_links = traci.trafficlight.getControlledLinks(tls_id)
        lane_set = {
            lane
            for link in chain.from_iterable(controlled_links)
            for lane in [link[0], link[-1]]
        }
        tls_controlled_lane_set.update(lane_set)


def get_distance_to_next_tls(veh_id: str):
    next_tls_info = traci.vehicle.getNextTLS(veh_id)
    if next_tls_info:
        tls_distance = next_tls_info[0][2]
        return tls_distance
    else:
        return float("inf")


def get_location(
    veh_id: str,
    lane_id: str = None,
    distance_to_tls_threshold: float = 25,
    highway_speed_threshold: float = 7.5,
    highlight_flag: bool = False,
):
    global tls_controlled_lane_set
    if tls_controlled_lane_set is None:
        tls_controlled_lane_set = set()
        cache_tls_controlled_lane_set()
    lane_id = lane_id if lane_id else traci.vehicle.getLaneID(veh_id)
    if traci.lane.getMaxSpeed(lane_id) > highway_speed_threshold:
        if highlight_flag:
            traci.vehicle.setColor(veh_id, (255, 0, 0, 255))  # red
        return "highway"
    elif (
        lane_id in tls_controlled_lane_set
        or get_distance_to_next_tls(veh_id) < distance_to_tls_threshold
    ):
        if highlight_flag:
            traci.vehicle.setColor(veh_id, (0, 255, 0, 255))  # green
        return "intersection"
    else:
        if highlight_flag:
            traci.vehicle.setColor(veh_id, (0, 0, 255, 255))  # blue
        return "roundabout"


def get_neighbour_lane(net, lane_id, direction="left"):
    if direction == "left":
        indexOffset = 1
    elif direction == "right":
        indexOffset = -1
    else:
        raise ValueError("direction must be either left or right")
    current_lane = net.getLane(lane_id)
    current_edge = current_lane.getEdge()
    all_lanes_on_edge = current_edge.getLanes()
    current_lane_index = all_lanes_on_edge.index(current_lane)
    neighbour_lane_index = current_lane_index + indexOffset
    neighbour_lane = all_lanes_on_edge[neighbour_lane_index]
    neighbour_lane_id = neighbour_lane.getID()
    return neighbour_lane_id


def get_sumo_angle(np_angle):
    sumo_angle = (90 - np_angle) % 360
    return sumo_angle


def get_lane_angle(lane_id, mode="start"):
    if mode == "start":
        relative_position = 0
    elif mode == "end":
        relative_position = traci.lane.getLength(lane_id) - 0.1
    else:
        raise ValueError("mode must be either start or end")
    lane_angle = traci.lane.getAngle(lane_id, relative_position)
    return lane_angle


def is_head_on(ego_obs, leader_info):
    ego_veh_lane_id = ego_obs["lane_id"] if ego_obs else None
    lead_veh_lane_id = traci.vehicle.getLaneID(leader_info[0]) if leader_info else None

    if ego_veh_lane_id is None or lead_veh_lane_id is None:
        return False

    ego_veh_lane_start_angle = get_lane_angle(ego_veh_lane_id, mode="start")
    lead_veh_lane_start_angle = get_lane_angle(lead_veh_lane_id, mode="start")
    ego_veh_lane_end_angle = get_lane_angle(ego_veh_lane_id, mode="end")
    lead_veh_lane_end_angle = get_lane_angle(lead_veh_lane_id, mode="end")

    if None in [
        ego_veh_lane_start_angle,
        lead_veh_lane_start_angle,
        ego_veh_lane_end_angle,
        lead_veh_lane_end_angle,
    ]:
        return False

    start_angle = abs(ego_veh_lane_start_angle - lead_veh_lane_start_angle)
    end_angle = abs(ego_veh_lane_end_angle - lead_veh_lane_end_angle)

    return 120 < start_angle < 240


def get_collision_type_and_prob(
    obs_dict: Dict[str, Any],
    negligence_command: NDECommand,
    location: Optional[str] = None,
) -> Tuple[float, str]:
    """
    Given current observation and the negligence mode, detect what type of collisions will be generated
    """
    if location is None:
        location = get_location(obs_dict["ego"]["veh_id"], obs_dict["ego"]["lane_id"])
    rear_end = negligence_command.info.get("is_car_following_flag", False)
    negligence_mode = negligence_command.info.get("negligence_mode", None)
    if "roundabout" in location:
        if negligence_mode == "LeftFoll" or negligence_mode == "RightFoll":
            return roundabout_cutin_prob, "roundabout_cutin"
        elif rear_end:
            return roundabout_rearend_prob, "roundabout_rearend"
        elif negligence_mode == "TrafficRule":
            return roundabout_fail_to_yield_prob, "roundabout_fail_to_yield"
        else:
            return (
                roundabout_neglect_conflict_lead_prob,
                "roundabout_neglect_conflict_lead",
            )
    elif "highway" in location:
        if negligence_mode == "LeftFoll" or negligence_mode == "RightFoll":
            return highway_cutin_prob, "highway_cutin"
        else:
            return highway_rearend_prob, "highway_rearend"
    elif "intersection" in location:
        if negligence_mode == "LeftFoll" or negligence_mode == "RightFoll":
            return intersection_cutin_prob, "intersection_cutin"
        elif rear_end:
            return intersection_rearend_prob, "intersection_rearend"
        elif is_head_on(
            obs_dict["ego"], negligence_command.info.get("leader_info", None)
        ):
            return intersection_headon_prob, "intersection_headon"
        elif negligence_mode == "TrafficRule":
            return intersection_tfl_prob, "intersection_tfl"
        else:
            return (
                intersection_neglect_conflict_lead_prob,
                "intersection_neglect_conflict_lead",
            )
    else:
        return 0, "no_collision"


from typing import List, Tuple


def angle_difference(angle1, angle2):
    # Compute the difference between the two angles and reduce it to the range [-180, 180]
    diff = (angle1 - angle2 + 180) % 360 - 180
    return abs(diff)


def is_car_following(follow_id: str, leader_id: str) -> bool:
    # check if the follow_id is car following the leader_id using the traci API and detect the future links
    current_edge_id = traci.vehicle.getLaneID(follow_id)
    leader_edge_id = traci.vehicle.getLaneID(leader_id)
    current_angle = traci.vehicle.getAngle(follow_id)
    leader_angle = traci.vehicle.getAngle(leader_id)
    # the two vehicle are on the same link
    if current_edge_id == leader_edge_id:
        return True
    elif angle_difference(current_angle, leader_angle) <= 5:
        return True
    else:
        # the two vehicle are on different links, but the leader is on the future link of the follower
        follower_future_link_infos: List[Tuple] = traci.vehicle.getNextLinks(follow_id)
        if len(follower_future_link_infos) == 0:
            return False
        follower_future_lane_id: str = follower_future_link_infos[0][0]
        follower_future_junction_lane_id: str = follower_future_link_infos[0][
            4
        ]  # + "_0"
        if (
            leader_edge_id in follower_future_lane_id
            or leader_edge_id in follower_future_junction_lane_id
        ):
            return True

        leader_future_link_infos: List[Tuple] = traci.vehicle.getNextLinks(leader_id)
        if len(leader_future_link_infos) == 0:
            return False
        leader_future_lane_id: str = leader_future_link_infos[0][0]
        leader_junction_lane_id: str = leader_future_link_infos[0][4]
        if (
            len(
                set(
                    [
                        follower_future_lane_id,
                        follower_future_junction_lane_id,
                        leader_future_lane_id,
                        leader_junction_lane_id,
                    ]
                )
            )
            < 4  # the leader and follower has the same future link
        ):
            return True
        else:
            return False


def predict_future_distance_velocity_vectorized(
    velocity: float,
    acceleration: float,
    duration_array: np.ndarray,
    max_velocity: float,
) -> np.ndarray:
    """Predict the future distance of the vehicle using vectorized operations for improved performance.

    Args:
        velocity (float): The initial velocity of the vehicle.
        acceleration (float): The acceleration of the vehicle.
        duration_array (np.ndarray): The array of time points at which to calculate distance.
        max_velocity (float): The maximum velocity of the vehicle.

    Returns:
        np.ndarray: The array of future distances at each time point in duration_array.
    """
    # Calculate velocity at each time point, ensuring it does not exceed max_velocity.
    velocity_array = np.clip(velocity + acceleration * duration_array, 0, max_velocity)

    # Calculate the average velocities between consecutive time points.
    average_velocities = 0.5 * (velocity_array[1:] + velocity_array[:-1])

    # Calculate the time differences between consecutive time points.
    time_differences = duration_array[1:] - duration_array[:-1]

    # Calculate distance increments using the average velocities and time differences.
    distance_increments = average_velocities * time_differences

    # Calculate the cumulative distance at each time point.
    cumulative_distances = np.cumsum(distance_increments)
    cumulative_distances = np.insert(
        cumulative_distances, 0, 0
    )  # Include starting point (distance=0)

    # if any distant in cumulative distances is negative, log the error
    if np.any(cumulative_distances < 0):
        logger.critical(
            f"negative distance in cumulative distances: {cumulative_distances}, {duration_array}, {velocity_array}, {average_velocities}, {time_differences}, {distance_increments}"
        )
    return cumulative_distances, velocity_array


def get_lanechange_longitudinal_speed(
    veh_id, current_speed, lane_width=None, lanechange_duration=1.0
):
    if lane_width is None:
        lane_width = traci.lane.getWidth(traci.vehicle.getLaneID(veh_id))
    lateral_speed = lane_width / lanechange_duration
    return math.sqrt(max(current_speed**2 - lateral_speed**2, 0))


@profile
def predict_future_trajectory(
    veh_id,
    obs_dict,
    control_command,
    sumo_net,
    time_horizon_step=4,
    time_resolution=0.5,
    interpolate_resolution=0.1,
    current_time=None,
    veh_info=None,
):
    """Predict the future trajectory of the vehicle.
    all position and heading in this function stays the same definition with sumo, which is (x, y) and angle in degree (north is 0, east is 90, south is 180, west is 270)

    Args:
        veh_id (str): the id of the vehicle
        time_horizon_step (int): the time horizon of the prediction
        time_resolution (float): the time resolution of the prediction

    Returns:
        future_trajectory_dict (dict): the future trajectory of the vehicle
    """
    info = addict.Dict()
    current_time = (
        current_time if current_time is not None else traci.simulation.getTime()
    )
    veh_info = (
        veh_info
        if veh_info is not None
        else get_vehicle_info(veh_id, obs_dict, sumo_net)
    )
    # info.veh_info = str(veh_info)
    # include the original position
    duration_array = np.array(
        [
            time_horizon_id * time_resolution
            for time_horizon_id in range(time_horizon_step + 1)
        ]
    )
    # info.duration_array = str(duration_array)
    acceleration = (
        control_command.acceleration
        if control_command.command_type == Command.ACCELERATION
        else veh_info["acceleration"]
    )
    # info.acceleration = str(acceleration)
    max_velocity = traci.vehicle.getAllowedSpeed(veh_id)
    # info.max_velocity = str(max_velocity)

    # info.future_distance_array = str(future_distance_array)
    # info.future_velocity_array = str(future_velocity_array)
    lane_width = traci.lane.getWidth(veh_info["lane_id"])
    lateral_offset = 0
    if control_command.command_type == Command.LEFT:
        lateral_offset = 1
        veh_info.velocity = get_lanechange_longitudinal_speed(
            veh_id,
            veh_info.velocity,
            lane_width,
        )
    elif control_command.command_type == Command.RIGHT:
        lateral_offset = -1
        veh_info.velocity = get_lanechange_longitudinal_speed(
            veh_id,
            veh_info.velocity,
            lane_width,
        )

    future_distance_array, future_velocity_array = (
        predict_future_distance_velocity_vectorized(
            veh_info["velocity"], acceleration, duration_array, max_velocity
        )
    )
    # info.lateral_offset = str(lateral_offset)

    trajectory_array = np.array(
        [
            veh_info.position[0],
            veh_info.position[1],
            veh_info.heading,
            future_velocity_array[0],
            0,
        ]
    )

    lanechange_finish_trajectory_point = None
    if (
        control_command.command_type == Command.LEFT
        or control_command.command_type == Command.RIGHT
    ):
        lanechange_finish_timestep = np.argmin(
            np.abs(duration_array - control_command.duration)
        )
        lanechange_finish_position, lanechange_finish_final_heading = (
            get_future_position_on_route(
                traci,
                veh_id,
                veh_info["edge_id"],
                veh_info["lane_position"],
                veh_info["lane_index"],
                veh_info["lane_id"],
                veh_info["route_id_list"],
                veh_info["route_length_list"],
                future_distance_array[lanechange_finish_timestep],
                lateral_offset,
                veh_info["upcoming_lane_id_list"],
            )
        )
        lanechange_finish_trajectory_point = np.array(
            [
                lanechange_finish_position[0],
                lanechange_finish_position[1],
                lanechange_finish_final_heading,
                future_velocity_array[lanechange_finish_timestep],
                duration_array[lanechange_finish_timestep],
            ]
        )
        trajectory_array = np.vstack(
            (trajectory_array, lanechange_finish_trajectory_point)
        )

    for duration, distance, velocity in zip(
        duration_array[1:], future_distance_array[1:], future_velocity_array[1:]
    ):
        if (
            lanechange_finish_trajectory_point is not None
            and duration <= lanechange_finish_trajectory_point[-1]
        ):
            continue
        future_position, future_heading = get_future_position_on_route(
            traci,
            veh_id,
            veh_info["edge_id"],
            veh_info["lane_position"],
            veh_info["lane_index"],
            veh_info["lane_id"],
            veh_info["route_id_list"],
            veh_info["route_length_list"],
            distance,
            lateral_offset,
            veh_info["upcoming_lane_id_list"],
        )
        trajectory_array = np.vstack(
            (
                trajectory_array,
                np.array(
                    [
                        future_position[0],
                        future_position[1],
                        future_heading,
                        velocity,
                        duration,
                    ]
                ),
            )
        )
        # info.original_trajectory_array = str(trajectory_array)

    future_trajectory_array = interpolate_future_trajectory(
        trajectory_array, interpolate_resolution
    )
    # info.interpolated_trajectory_array = str(future_trajectory_array)

    future_trajectory_array[:, -1] += current_time
    return future_trajectory_array, info


def get_future_position_on_route(
    traci,
    veh_id: str,
    veh_edge_id: str,
    veh_lane_position: float,
    veh_lane_index: int,
    veh_lane_id: str,
    veh_route_id_list: List[str],
    veh_route_length_list: List[float],
    future_distance: float,
    future_lateral_offset: int,
    upcoming_lane_id_list: List[str],
) -> Tuple[Tuple[float, float], float]:
    """
    Given the current vehicle edge id, lane position, current lane id, and the future distance / future lateral offset, predict the future position of the vehicle.
    """
    veh_lane_position += future_distance
    current_lane_length = traci.lane.getLength(veh_lane_id)
    current_route_index = veh_route_id_list.index(veh_edge_id)

    # calculate the corresponding edge and lane position
    while (
        veh_lane_position > current_lane_length
        and current_route_index < len(veh_route_id_list) - 1
    ):
        current_route_index += 1
        veh_edge_id = veh_route_id_list[current_route_index]
        veh_lane_position -= current_lane_length
        current_lane_length = veh_route_length_list[current_route_index]

    # calculate the new lane index
    veh_lane_id, veh_lane_index = get_future_lane_id_index(
        veh_id,
        veh_edge_id,
        upcoming_lane_id_list,
        veh_lane_index,
        future_lateral_offset,
    )

    veh_lane_position = min(
        veh_lane_position,
        current_lane_length,
        traci.lane.getLength(veh_lane_id) - 0.1,
    )
    future_position = traci.simulation.convert2D(
        veh_edge_id, veh_lane_position, veh_lane_index
    )
    future_heading = traci.lane.getAngle(veh_lane_id, veh_lane_position)
    return future_position, future_heading


def get_future_lane_id_index(
    veh_id, veh_edge_id, upcoming_lane_id_list, original_lane_index, lateral_offset
):
    if traci.edge.getLaneNumber(veh_edge_id) == 1:
        veh_lane_index = 0
        veh_lane_id = veh_edge_id + "_0"
        return veh_lane_id, veh_lane_index
    else:
        max_lane_index = traci.edge.getLaneNumber(veh_edge_id) - 1
        predicted_vehicle_lane_id = get_vehicle_future_lane_id_from_edge(
            veh_edge_id, upcoming_lane_id_list
        )
        if predicted_vehicle_lane_id is not None:
            predicted_veh_lane_index = int(predicted_vehicle_lane_id.split("_")[-1])
        else:
            predicted_veh_lane_index = original_lane_index

        veh_lane_index = min(
            max_lane_index, max(0, predicted_veh_lane_index + lateral_offset)
        )
        veh_lane_id = veh_edge_id + f"_{veh_lane_index}"
        return veh_lane_id, veh_lane_index


def get_vehicle_future_lane_id_from_edge(edge_id, upcoming_lane_id_list):
    return next(
        (lane_id for lane_id in upcoming_lane_id_list if edge_id in lane_id), None
    )


@profile
def get_vehicle_info(veh_id, obs_dict, sumo_net):
    """Generate vehicle information for future trajectory prediction

    Args:
        veh_id (str): input vehicle id

    Returns:
        veh_info (dict): output dictionary of vehicle information
    """
    ego_obs = obs_dict["ego"]
    veh_info = VehicleInfoForPredict(
        id=veh_id,
        acceleration=traci.vehicle.getAcceleration(veh_id),
        route=traci.vehicle.getRoute(veh_id),
        route_index=traci.vehicle.getRouteIndex(veh_id),
        edge_id=traci.vehicle.getRoadID(veh_id),
        lane_id=traci.vehicle.getLaneID(veh_id),
        lane_index=traci.vehicle.getLaneIndex(veh_id),
        position=traci.vehicle.getPosition(veh_id),
        velocity=traci.vehicle.getSpeed(veh_id),
        heading=traci.vehicle.getAngle(veh_id),
        lane_position=traci.vehicle.getLanePosition(veh_id),
        length=traci.vehicle.getLength(veh_id),
    )
    route_with_internal = sumolib.route.addInternal(sumo_net, veh_info.route)
    veh_info.route_id_list = [route._id for route in route_with_internal]
    # veh_info.route_length_list = [route._length for route in route_with_internal]
    veh_info.route_length_list = [
        traci.lane.getLength(edge_id + "_0") for edge_id in veh_info.route_id_list
    ]
    veh_info.upcoming_lane_id_list = get_upcoming_lane_id_list(veh_id)
    return veh_info


def get_upcoming_lane_id_list(veh_id):
    veh_next_links = traci.vehicle.getNextLinks(veh_id)
    current_lane_id = traci.vehicle.getLaneID(veh_id)
    lane_links = traci.lane.getLinks(current_lane_id)
    upcoming_lane_id_list = [current_lane_id]
    if isinstance(lane_links, list) and len(lane_links) > 0:
        for lane_link in lane_links:
            lane_id = lane_link[0]
            via_lane_id = lane_link[4]
            if via_lane_id != "":
                upcoming_lane_id_list.append(via_lane_id)
            upcoming_lane_id_list.append(lane_id)

    if len(veh_next_links) == 0:
        return upcoming_lane_id_list
    for link in veh_next_links:
        lane_id = link[0]
        via_lane_id = link[4]
        upcoming_lane_id_list.append(via_lane_id)
        upcoming_lane_id_list.append(lane_id)
    return upcoming_lane_id_list


from dataclasses import dataclass
from typing import List, Optional


@dataclass
class VehicleInfoForPredict:
    id: str
    acceleration: float
    route: List[str]
    route_index: int
    edge_id: str
    lane_id: str
    lane_index: int
    position: List[float]
    velocity: float
    heading: float
    lane_position: float
    length: float
    route_id_list: Optional[List[str]] = None
    route_length_list: Optional[List[float]] = None
    upcoming_lane_id_list: Optional[List[str]] = None

    def __getitem__(self, item):
        return self.__dict__[item]
