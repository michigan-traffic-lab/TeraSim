import json
import math
import time

from terasim.overlay import traci
from terasim.simulator import Simulator
from terasim_cosim.constants import *
from terasim_cosim.terasim_plugin.terasim_tls_plugin import TeraSimTLSPlugin
from terasim_cosim.redis_client_wrapper import create_redis_client
from terasim_cosim.redis_msgs import GeneralMsg


def _cal_rear_points(head_center, heading, length=4.8, width=1.8):
    rear_center_x = head_center[0] - length * math.sin(heading)
    rear_center_y = head_center[1] - length * math.cos(heading)
    rear_right = (
        rear_center_x + width / 2 * math.cos(heading),
        rear_center_y - width / 2 * math.sin(heading),
    )
    rear_left = (
        rear_center_x - width / 2 * math.cos(heading),
        rear_center_y + width / 2 * math.sin(heading),
    )
    return rear_left, rear_right


class TeraSimTLSPluginAutowareAI(TeraSimTLSPlugin):
    def __init__(self):
        super().__init__(control_tls=True)
        self.cav_front_tls_info = {}
        self.autoware_ai_tls_configs = {
            "signal_id_list": [
                "NODE_18",
                "NODE_12",
                "NODE_11",
                "NODE_17",
                "NODE_23",
                "NODE_24",
            ],
            "signal_entry_lane": {
                # name of the traffic light intersection
                "NODE_18": {
                    # name of the entry lane
                    "lane": [
                        ["EG_17_1_1_0"],
                        ["EG_17_1_1_0"],
                        ["EG_14_1_1_0"],
                        ["EG_14_1_1_0"],
                        ["EG_9_1_23_0"],
                        ["EG_9_1_23_0"],
                        ["EG_28_2_1_0"],
                        ["EG_28_2_1_0"],
                    ],
                    # connection direction: three bits represent three direction: left 2^0, straight 2^1, right 2^2
                    "direction": [6, 1, 6, 1, 6, 1, 6, 1],
                    # usually four, representing north, east, south and west
                    "orientation": [0, 0, 1, 1, 2, 2, 3, 3],
                    "new_index": 2,
                    "stoplines": [
                        [[48.10, 148.42], [52.03, 148.28]],
                        [[66.04, 140.14], [65.95, 136.58]],
                        [[58.04, 124.06], [54.70, 123.97]],
                        [[41.95, 134.02], [42.09, 137.39]],
                    ],
                },
                "NODE_12": {
                    "lane": [
                        ["EG_8_1_1_0"],
                        ["EG_18_12_171_0"],
                        ["EG_18_12_171_1"],
                        ["EG_14_2_1_0"],
                    ],
                    "direction": [6, 2, 1, 5],
                    "orientation": [0, 2, 2, 3],
                    "new_index": 3,
                    "stoplines": [
                        [[107.66, 145.92], [111.02, 145.77]],
                        [[116.55, 120.48], [110.20, 120.92]],
                        [[116.55, 120.48], [110.20, 120.92]],
                        [[99.78, 131.16], [99.89, 134.78]],
                    ],
                },
                "NODE_11": {
                    "lane": [
                        ["gneE8_0", "EG_7_3_1_0"],
                        ["EG_6_1_1_0"],
                        ["EG_6_2_1_0"],
                        ["EG_8_23_12_0"],
                        ["EG_8_23_12_1"],
                        ["EG_13_1_1_0"],
                        ["EG_13_1_1_0"],
                        ["EG_7_4_2_0"],
                    ],
                    "direction": [1, 6, 1, 6, 1, 6, 1, 6],
                    "orientation": [0, 1, 1, 2, 2, 3, 3, 0],
                    "new_index": 4,
                    "stoplines": [
                        [[110.39, 215.93], [116.90, 215.85]],
                        [[131.01, 203.47], [130.77, 198.50]],
                        [[119.07, 173.64], [112.34, 173.75]],
                        [[104.34, 188.83], [104.57, 192.78]],
                    ],
                },
                "NODE_17": {
                    "lane": [
                        ["EG_16_23_1_0"],
                        ["EG_16_23_1_1"],
                        ["EG_12_1_2_0"],
                        ["gneE5_0", "EG_12_2_1_0"],
                        ["EG_17_23_1_0"],
                        ["EG_17_23_1_1"],
                        ["EG_27_1_1_0"],
                        ["EG_27_2_2_0"],
                        ["gneE6_0", "EG_27_3_1_0"],
                    ],
                    "direction": [1, 6, 1, 6, 1, 4, 2, 1, 6],
                    "orientation": [0, 0, 1, 1, 2, 2, 3, 3, 3],
                    "new_index": 1,
                    "stoplines": [
                        [[50.52, 217.64], [57.59, 217.44]],
                        [[66.66, 204.17], [66.46, 197.51]],
                        [[60.59, 180.68], [53.32, 181.01]],
                        [[43.61, 188.71], [43.95, 198.92]],
                    ],
                },
                "NODE_23": {
                    "lane": [
                        ["EG_23_1_1_0"],
                        ["EG_23_1_1_0"],
                        ["EG_26_12_1_0"],
                        ["EG_26_12_1_1"],
                        ["EG_24_2_1_0"],
                    ],
                    "direction": [2, 1, 4, 1, 6],
                },
                "NODE_24": {
                    "lane": [
                        ["EG_24_1_1_0"],
                        ["EG_24_1_1_0"],
                        ["EG_28_1_1_0"],
                        ["EG_25_2_7_0"],
                    ],
                    "direction": [2, 1, 5, 6],
                },
            },
        }

    def on_start(self, simulator: Simulator, ctx):
        redis_configs = {COSIM_TLS_INFO_KEY: GeneralMsg}
        self.redis_client = create_redis_client(redis_configs)

    def sync_terasim_tls_to_cosim(self):
        self.update_tls_info()

        temp_cav_front_tls_info = GeneralMsg()
        temp_cav_front_tls_info.header.timestamp = time.time()
        temp_str = json.dumps(self.cav_front_tls_info)
        temp_cav_front_tls_info.data = temp_str

        self.redis_client.set(COSIM_TLS_INFO_KEY, temp_cav_front_tls_info)

    def update_tls_info(self):
        """Update traffic light signal information for CAV"""
        vehID = "CAV"
        lane_id = traci.vehicle.getLaneID(vehID)
        print("CAV", lane_id)
        front_tls_status = 0
        stop_find_flag = False
        if (
            self.autoware_ai_tls_configs["signal_entry_lane"] == {}
        ):  # if no traffic light, stop finding traffic lights
            stop_find_flag = True
        cv_ahead_flag = False
        new_tls_id = 0
        stopline1_x, stopline1_y = -100, -100
        stopline2_x, stopline2_y = -100, -100

        if not stop_find_flag:
            for tls_id in self.autoware_ai_tls_configs["signal_id_list"]:
                for i in range(
                    len(
                        self.autoware_ai_tls_configs["signal_entry_lane"][tls_id][
                            "lane"
                        ]
                    )
                ):
                    lane_list = self.autoware_ai_tls_configs["signal_entry_lane"][
                        tls_id
                    ]["lane"][i]
                    if lane_id in lane_list:
                        tls_info = traci.trafficlight.getRedYellowGreenState(tls_id)
                        front_tls_status = tls_info[i]
                        leader_info = traci.vehicle.getLeader(
                            "CAV", 30
                        )  # empty leader: None
                        if (
                            leader_info is not None
                            and traci.vehicle.getLaneID(leader_info[0]) in lane_list
                        ):
                            leader_head_center = traci.vehicle.getPosition(
                                leader_info[0]
                            )
                            leader_heading = (
                                traci.vehicle.getAngle(leader_info[0]) / 180 * math.pi
                            )
                            veh_length = 5.0
                            if "CV" in leader_info[0]:
                                veh_length += traci.vehicle.getSpeed("CAV") + 2
                                cv_ahead_flag = True
                            leader_rear_left, leader_rear_right = _cal_rear_points(
                                leader_head_center,
                                leader_heading,
                                length=veh_length,
                            )
                            stopline1_x, stopline1_y = leader_rear_right
                            stopline2_x, stopline2_y = leader_rear_left
                            print(
                                "!!!!!!!!!!Distance to effective leader:",
                                leader_info,
                                traci.vehicle.getMinGap("CAV"),
                            )
                        else:
                            if leader_info is not None:
                                print(
                                    "??????????????Distance to ineffective leader:",
                                    leader_info,
                                    traci.vehicle.getMinGap("CAV"),
                                )
                                print(
                                    "Reason",
                                    traci.vehicle.getRoadID(leader_info[0]),
                                    lane_list,
                                )
                            orien_ = self.autoware_ai_tls_configs["signal_entry_lane"][
                                tls_id
                            ]["orientation"][i]
                            stopline1_x, stopline1_y = self.autoware_ai_tls_configs[
                                "signal_entry_lane"
                            ][tls_id]["stoplines"][orien_][0]
                            stopline2_x, stopline2_y = self.autoware_ai_tls_configs[
                                "signal_entry_lane"
                            ][tls_id]["stoplines"][orien_][1]
                        stop_find_flag = True
                        break
                if stop_find_flag:
                    break
        if front_tls_status in ["0", "O"]:
            front_tls_status = 0
        if front_tls_status != 0:
            front_tls_status = int(front_tls_status in ["G", "g"]) + 1
            if cv_ahead_flag:
                front_tls_status = 2
            # front_tls_status = 1
            new_tls_id = self.autoware_ai_tls_configs["signal_entry_lane"][tls_id][
                "new_index"
            ]
        self.cav_front_tls_info = {
            "front_tls_status": front_tls_status,
            "new_tls_id": new_tls_id,
            "stopline1_x": stopline1_x,
            "stopline1_y": stopline1_y,
            "stopline2_x": stopline2_x,
            "stopline2_y": stopline2_y,
            "tls_info": self.get_all_tls_info(),
        }
