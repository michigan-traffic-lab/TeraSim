import math
import time
import copy
import numpy as np

from terasim_utils import *

from terasim.overlay import traci
from terasim.simulator import Simulator

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client
from terasim_cosim.redis_msgs import (
    Actor,
    ActorDict,
    SUMOSignal,
    SUMOSignalDict,
    ConstructionZone,
)


class TeraSimCoSimPlugin:

    def __init__(
        self,
        remote_flag=False,
        control_cav=False,
        control_tls=True,
        keepRoute=1,
        pub_channels=[],
        sub_channels=[],
        latency_src_channels=[],
        closed_lane_ids=[],
        closed_lane_pos=[],
        closed_lane_shapes=[],
    ):
        self._routes = set()

        self.remote_flag = remote_flag
        self.control_cav = control_cav
        self.control_tls = control_tls
        self.keepRoute = keepRoute

        self.pub_channels = pub_channels
        self.sub_channels = sub_channels
        self.latency_src_channels = latency_src_channels

        self.closed_lane_ids = closed_lane_ids
        self.closed_lane_pos = closed_lane_pos
        self.closed_lane_shapes = closed_lane_shapes

    def on_start(self, simulator: Simulator, ctx):
        key_value_config = {
            CAV_INFO: ActorDict,
            TLS_INFO: SUMOSignalDict,
            TERASIM_ACTOR_INFO: ActorDict,
            CONSTRUCTION_ZONE_INFO: ConstructionZone,
        }

        self.redis_client = create_redis_client(
            key_value_config=key_value_config,
            remote_flag=self.remote_flag,
            pub_channels=self.pub_channels,
            sub_channels=self.sub_channels,
            latency_src_channels=self.latency_src_channels,
        )

    def on_step(self, simulator: Simulator, ctx):
        if self.control_cav:
            self.sync_terasim_cav_to_cosim()
        else:
            self.sync_cosim_cav_to_terasim()

        if self.control_tls:
            self.sync_terasim_tls_to_cosim()
        else:
            self.sync_cosim_tls_to_terasim()

        self.sync_terasim_actor_to_cosim()
        self.sync_terasim_construction_zone_to_cosim()

        return True

    def on_stop(self, simulator: Simulator, ctx):
        pass

    def inject(self, simulator: Simulator, ctx):
        self.ctx = ctx
        self.simulator = simulator

        simulator.start_pipeline.hook("cosim_start", self.on_start, priority=-100)
        simulator.step_pipeline.hook("cosim_step", self.on_step, priority=-100)
        simulator.stop_pipeline.hook("cosim_stop", self.on_stop, priority=-100)

    def sync_terasim_construction_zone_to_cosim(self):
        def calculate_edge_and_starting_edge(center_points, lane_width, pos):
            # record edge points
            starting_point = center_points[0]
            ending_point = center_points[-1]

            # calculate left edge
            center_points = np.array(center_points)
            left_edge_points = []
            for i in range(len(center_points) - 1):
                current_point = center_points[i]
                next_point = center_points[i + 1]
                if pos == "right":
                    segment_vector = next_point - current_point
                else:
                    segment_vector = current_point - next_point
                normal_vector = np.array([-segment_vector[1], segment_vector[0]])
                normal_vector_length = np.linalg.norm(normal_vector)
                normal_vector_unit = normal_vector / normal_vector_length
                left_vector = normal_vector_unit * (lane_width / 2)
                left_edge_start = current_point + left_vector
                left_edge_end = next_point + left_vector
                left_edge_points.append(tuple(left_edge_start))
            left_edge_points.append(tuple(left_edge_end))

            # polish starting edge
            left_edge_points[0] = starting_point
            middle_point = np.mean(left_edge_points[:2], axis=0)
            left_edge_points.insert(1, middle_point)

            temp_point = 2 * np.array(left_edge_points[0]) - np.array(
                left_edge_points[1]
            )
            left_edge_points.insert(0, temp_point)

            # polish ending edge
            left_edge_points[-1] = ending_point
            middle_point = np.mean(left_edge_points[-2:], axis=0)
            left_edge_points.insert(-1, middle_point)

            temp_point = 2 * np.array(left_edge_points[-1]) - np.array(
                left_edge_points[-2]
            )
            left_edge_points.insert(-1, temp_point)

            return left_edge_points

        closed_lane_ids = self.closed_lane_ids
        closed_lane_pos = self.closed_lane_pos
        closed_lane_shapes = copy.deepcopy(self.closed_lane_shapes)
        # perform offset correction
        for closed_lane_id, lane_pos in zip(closed_lane_ids, closed_lane_pos):
            shapes = traci.lane.getShape(closed_lane_id)
            # convert list of tuples to list of lists
            shapes = [list(elem) for elem in shapes]
            shapes = calculate_edge_and_starting_edge(
                shapes, traci.lane.getWidth(closed_lane_id), lane_pos
            )
            shapes = [[x - UTM_OFFSET[0], y - UTM_OFFSET[1]] for x, y in shapes]
            closed_lane_shapes.append(shapes)
        construction_zone_terasim_info = ConstructionZone()
        construction_zone_terasim_info.header.timestamp = time.time()
        construction_zone_terasim_info.closed_lane_id = closed_lane_ids
        construction_zone_terasim_info.closed_lane_shapes = closed_lane_shapes
        self.redis_client.set(
            CONSTRUCTION_ZONE_INFO,
            construction_zone_terasim_info,
        )

    def sync_terasim_cav_to_cosim(self):
        orientation = traci.vehicle.getAngle("CAV")
        orientation = sumo_heading_to_orientation(orientation)

        location = traci.vehicle.getPosition3D("CAV")
        location = (location[0], location[1], location[2] if len(location) > 2 else 0.0)

        x = location[0] - UTM_OFFSET[0]
        y = location[1] - UTM_OFFSET[1]
        z = location[2] - UTM_OFFSET[2]

        length = traci.vehicle.getLength("CAV")
        width = traci.vehicle.getWidth("CAV")
        height = traci.vehicle.getHeight("CAV")

        x, y = front_coordinate_to_center_coordinate(x, y, orientation, length)

        speed = traci.vehicle.getSpeed("CAV")

        cav_info = ActorDict()
        cav_info.header.timestamp = time.time()
        cav_info.data["CAV"] = Actor(
            x=x,
            y=y,
            z=z,
            length=length,
            width=width,
            height=height,
            orientation=orientation,
            speed_long=speed,
        )

        self.redis_client.set(CAV_INFO, cav_info)

    def sync_cosim_cav_to_terasim(self):
        try:
            cav_info = self.redis_client.get(CAV_INFO)
        except:
            print("cav_info not available. Exiting...")
            return

        if cav_info:
            cav_info = cav_info.data["CAV"]

            x = cav_info.x + UTM_OFFSET[0]
            y = cav_info.y + UTM_OFFSET[1]
            orientation = cav_info.orientation
            length = cav_info.length
            speed_long = cav_info.speed_long

            x, y = center_coordinate_to_front_coordinate(x, y, orientation, length)
            orientation = orientation_to_sumo_heading(orientation)

            if "CAV" in self.simulator.get_vehID_list():
                traci.vehicle.moveToXY(
                    "CAV", "", 0, x, y, angle=orientation, keepRoute=self.keepRoute
                )
                traci.vehicle.setSpeedMode("CAV", 0)
                traci.vehicle.setSpeed("CAV", speed_long)

    def sync_terasim_actor_to_cosim(self):
        """sync sumo controlled actor to cosim"""
        veh_list = self.simulator.get_vehID_list()
        ped_list = traci.person.getIDList()

        terasim_controlled_vehicle_list = [
            vehID for vehID in veh_list if "BV" in vehID or "POV" in vehID
        ]
        terasim_controlled_pedestrian_list = [
            pedID for pedID in ped_list if "VRU" in pedID
        ]

        terasim_controlled_actor_info_with_timestamp = ActorDict()
        terasim_controlled_actor_info_with_timestamp.header.timestamp = time.time()

        for vehID in terasim_controlled_vehicle_list:
            orientation = traci.vehicle.getAngle(vehID)
            orientation = sumo_heading_to_orientation(orientation)
            location = traci.vehicle.getPosition3D(vehID)
            x = location[0] - UTM_OFFSET[0]
            y = location[1] - UTM_OFFSET[1]
            z = location[2] - UTM_OFFSET[2]
            length = traci.vehicle.getLength(vehID)
            width = traci.vehicle.getWidth(vehID)
            height = traci.vehicle.getHeight(vehID)
            x, y = front_coordinate_to_center_coordinate(x, y, orientation, length)
            speed = traci.vehicle.getSpeed(vehID)
            slope = traci.vehicle.getSlope(vehID)
            type = traci.vehicle.getTypeID(vehID)
            veh_info = Actor(
                x=x,
                y=y,
                z=z,
                length=length,
                width=width,
                height=height,
                orientation=orientation,
                speed_long=speed,
                slope=slope,
                type=type,
            )
            terasim_controlled_actor_info_with_timestamp.data[vehID] = veh_info

        # Pedestrians
        for pedID in terasim_controlled_pedestrian_list:
            orientation = traci.person.getAngle(pedID)
            orientation = sumo_heading_to_orientation(orientation)

            location = traci.person.getPosition3D(pedID)
            x = location[0] - UTM_OFFSET[0]
            y = location[1] - UTM_OFFSET[1]
            z = location[2] - UTM_OFFSET[2]
            length = traci.person.getLength(pedID)
            width = traci.person.getWidth(pedID)
            height = traci.person.getHeight(pedID)
            speed = traci.person.getSpeed(pedID)
            type = traci.person.getTypeID(pedID)
            direction_x, direction_y = math.cos(orientation), math.sin(orientation)
            ped_info = Actor(
                x=x,
                y=y,
                z=z,
                orientation=orientation,
                length=length,
                width=width,
                height=height,
                speed_long=speed,
                direction_x=direction_x,
                direction_y=direction_y,
                type=type,
            )

            terasim_controlled_actor_info_with_timestamp.data[pedID] = ped_info

        self.redis_client.set(
            TERASIM_ACTOR_INFO, terasim_controlled_actor_info_with_timestamp
        )

    def sync_terasim_tls_to_cosim(self):
        sumo_tls = self.get_all_tls_info()

        nextTLS = traci.vehicle.getNextTLS("CAV")
        if nextTLS:
            tlsID, tlsIndex, distance, state = nextTLS[0]
        else:
            state, distance = "", 0.0

        tls_info = SUMOSignalDict()
        tls_info.header.timestamp = time.time()
        tls_info.header.information = "TeraSim"
        tls_info.av_next_tls = state
        tls_info.av_next_dist = distance

        for tls_id in sumo_tls:
            signal = SUMOSignal()
            signal.tls = sumo_tls[tls_id]
            tls_info.data[tls_id] = signal

        self.redis_client.set(TLS_INFO, tls_info)

    def sync_cosim_tls_to_terasim(self):
        cosim_controlled_tls_info = self.redis_client.get(TLS_INFO)
        if cosim_controlled_tls_info:
            data = cosim_controlled_tls_info.data
            for signal_id in data:
                traci.trafficlight.setRedYellowGreenState(
                    signal_id, data[signal_id].tls
                )

    def get_all_tls_info(self):
        self.tls_id_list = traci.trafficlight.getIDList()

        tls_info = {}
        for tls_id in self.tls_id_list:
            tls_info[tls_id] = traci.trafficlight.getRedYellowGreenState(tls_id)
        return tls_info