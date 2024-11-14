import time
import json

from terasim.overlay import traci
from terasim_cosim.constants import *
from terasim_cosim.redis_msgs import *
from terasim_cosim.redis_client_wrapper import RedisBasicPlugin
from terasim_cosim.terasim_plugin.utils.convertion import (
    sumo_heading_to_orientation,
    sumo_coordinate_to_center_coordinate,
    sumo_to_utm_coordinate,
    sumo_to_utm_coordinate_withprojection,
)


def calculate_MinDistance(context_info):
    """Calculate the minimum distance metric based on AV and BV data."""
    av_info = context_info["CAV"]
    av_context_info = context_info
    av_x, av_y, av_speed, av_orientation = (
        av_info["x"],
        av_info["y"],
        av_info["speed_long"],
        av_info["orientation"],
    )
    (
        min_distance_bv_x,
        min_distance_bv_y,
        min_distance_bv_speed,
        min_distance_bv_id,
    ) = (None, None, None, None)
    min_distance = 100
    placeholder_value = 30
    for bv_id in av_context_info:
        if bv_id == "CAV":
            continue
        bv_info = av_context_info[bv_id]
        bv_x_tmp, bv_y_tmp, bv_speed_tmp, bv_orientation_tmp = (
            bv_info["x"],
            bv_info["y"],
            bv_info["speed_long"],
            bv_info["orientation"],
        )
        # if the BV is in opposite direction, ignore it, orientation is in radian
        if abs(av_orientation - bv_orientation_tmp) > 1.3:
            continue
        distance = ((av_x - bv_x_tmp) ** 2 + (av_y - bv_y_tmp) ** 2) ** 0.5
        if distance < min_distance:
            min_distance = distance
            min_distance_bv_x, min_distance_bv_y, min_distance_bv_speed = (
                bv_x_tmp,
                bv_y_tmp,
                bv_speed_tmp,
            )
            min_distance_bv_id = bv_id
    if min_distance_bv_x is None or min_distance_bv_y is None:
        return placeholder_value, None
    else:
        if min_distance <= 0 or min_distance > placeholder_value:
            return placeholder_value, None
        return min_distance, json.dumps({
            "min_distance_bv_id": min_distance_bv_id,
            "min_distance_bv_x": min_distance_bv_x,
            "min_distance_bv_y": min_distance_bv_y,
            "min_distance_bv_speed": min_distance_bv_speed,
        })


def calculate_SpeedDifference(context_info):
    """Calculate the speed difference metric based on AV and BV data."""
    av_info = context_info["CAV"]
    av_lane_id = av_info["lane_id"]
    av_lane_speed_limit = traci.lane.getMaxSpeed(av_lane_id)
    av_speed = av_info["speed_long"]
    return max(av_lane_speed_limit - av_speed, 0), None


def calculate_TTC(context_info):
    """Calculate the TTC metric based on AV and BV data."""
    placeholder_value = 25
    av_info = context_info["CAV"]
    av_context_info = context_info
    av_x, av_y, av_speed = av_info["x"], av_info["y"], av_info["speed_long"]
    leading_bv_x, leading_bv_y, leading_bv_speed = None, None, None
    for bv_id in av_context_info:
        if bv_id == "CAV":
            continue
        bv_info = av_context_info[bv_id]
        bv_leading_info = bv_info["leading_info"]
        if bv_leading_info["is_leading_cav"]:
            leading_bv_x, leading_bv_y, leading_bv_speed = (
                bv_info["x"],
                bv_info["y"],
                bv_info["speed_long"],
            )
            break
    if leading_bv_x is None or leading_bv_y is None:
        return placeholder_value, None
    else:
        speed_diff = av_speed - leading_bv_speed
        if speed_diff == 0:
            speed_diff = 1e-5
        ttc = (
            (av_x - leading_bv_x) ** 2 + (av_y - leading_bv_y) ** 2
        ) ** 0.5 / speed_diff
        if ttc <= 0 or ttc > placeholder_value:
            return placeholder_value, None
        return ttc, None


DEFAULT_METRICS_CONFIG = {
    "min_distance": {
        "function": calculate_MinDistance,
        "lower_threshold": 0,
        "upper_threshold": 20,
    },
    "speed_difference": {
        "function": calculate_SpeedDifference,
        "lower_threshold": 0,
        "upper_threshold": 10,
    },
    "ttc": {
        "function": calculate_TTC,
        "lower_threshold": 0,
        "upper_threshold": 2,
    },
}


class TeraSimMetricsPlugin(RedisBasicPlugin):

    def __init__(self, metrics_list=[]):
        key_value_config = {METRIC_INFO: MetricDict}
        super().__init__(key_value_config)
        self.cav_id = "CAV"
        self.observation_range = 50
        self.projection = None
        self.coordinate_transformation = DEFAULT_COORDINATE_TRANSFORMATION
        for metric in metrics_list:
            if metric not in DEFAULT_METRICS_CONFIG:
                raise ValueError(f"Unsupported metric: {metric}")
        self.metrics_list = metrics_list

    def on_step(self, simulator, ctx):
        try:
            # if 1:
            context_info = self.obtain_context_info()
            # Calculate the metric based on AV and BV data (example calculation)
            metrics_msg = MetricDict()
            metrics_msg.header.timestamp = time.time()
            for metric_name in self.metrics_list:
                metric_msg = Metric()
                metric_function = DEFAULT_METRICS_CONFIG[metric_name]["function"]
                metric_msg.metric_value, metric_msg.info = metric_function(context_info)
                if (
                    DEFAULT_METRICS_CONFIG[metric_name]["lower_threshold"]
                    <= metric_msg.metric_value
                    <= DEFAULT_METRICS_CONFIG[metric_name]["upper_threshold"]
                ):
                    metric_msg.alert = True
                else:
                    metric_msg.alert = False
                metrics_msg.data[metric_name] = metric_msg

            self.redis_client.set(METRIC_INFO, metrics_msg)

        except Exception as e:
            print(f"Error in metric calculation: {e}")

    def on_stop(self, simulator, ctx):
        pass

    def obtain_context_info(self):
        """Obtain the context information for the metrics calculation."""
        context_info = {}
        leader_info = traci.vehicle.getLeader(self.cav_id, 1000)
        if leader_info is None:
            leading_vehicle_id = None
            leading_vehicle_distance = None
        else:
            leading_vehicle_id = leader_info[0]
            leading_vehicle_distance = leader_info[1]
        cav_pose = traci.vehicle.getPosition3D(self.cav_id)
        terasim_vehicle_id_list = traci.vehicle.getIDList()
        surrounding_vehicle_id_list = []
        for vehicle_id in terasim_vehicle_id_list:
            veh_pose = traci.vehicle.getPosition3D(vehicle_id)
            dist_ = (
                (cav_pose[0] - veh_pose[0]) ** 2 + (cav_pose[1] - veh_pose[1]) ** 2
            ) ** 0.5
            if dist_ < self.observation_range:
                surrounding_vehicle_id_list.append(vehicle_id)
        for vehicle_id in surrounding_vehicle_id_list:
            obs = {
                "acc": traci.vehicle.getAcceleration(vehicle_id),
                "orientation": sumo_heading_to_orientation(
                    traci.vehicle.getAngle(vehicle_id)
                ),
                "pose": list(
                    traci.vehicle.getPosition3D(vehicle_id)
                ),  # center of front bumper
                "size": [
                    traci.vehicle.getLength(vehicle_id),
                    traci.vehicle.getWidth(vehicle_id),
                    traci.vehicle.getHeight(vehicle_id),
                ],
                "slope": traci.vehicle.getSlope(vehicle_id),
                "v_lat": traci.vehicle.getLateralSpeed(vehicle_id),
                "v_long": traci.vehicle.getSpeed(vehicle_id),
                "edge_id": traci.vehicle.getRoadID(vehicle_id),
                "lane_id": traci.vehicle.getLaneID(vehicle_id),
            }
            # convert from center of front bumber to center of mass
            sumo_front_bumper_coordinate = obs["pose"]
            sumo_center_coordinate = sumo_coordinate_to_center_coordinate(
                sumo_front_bumper_coordinate[0],
                sumo_front_bumper_coordinate[1],
                obs["orientation"],
                obs["size"][0],
            )
            # convert from SUMO coordinate to UTM coordinate
            if self.projection:
                utm_center_coordinate = sumo_to_utm_coordinate_withprojection(
                    sumo_center_coordinate,
                    self.projection,
                )

            else:
                utm_center_coordinate = sumo_to_utm_coordinate(
                    sumo_center_coordinate,
                    offset=self.coordinate_transformation["SUMO_net_offset"],
                )

            context_info[vehicle_id] = {
                "x": utm_center_coordinate[0],
                "y": utm_center_coordinate[1],
                "z": obs["pose"][2],
                "length": obs["size"][0],
                "width": obs["size"][1],
                "height": obs["size"][2],
                "orientation": obs["orientation"],
                "slope": obs["slope"],
                "speed_long": obs["v_long"],
                "speed_lat": obs["v_lat"],
                "accel_long": obs["acc"],
                "edge_id": obs["edge_id"],
                "lane_id": obs["lane_id"],
                "leading_info": (
                    {
                        "is_leading_cav": True,
                        "distance": leading_vehicle_distance,
                    }
                    if vehicle_id == leading_vehicle_id
                    else {
                        "is_leading_cav": False,
                        "distance": None,
                    }
                ),
            }

        return context_info
