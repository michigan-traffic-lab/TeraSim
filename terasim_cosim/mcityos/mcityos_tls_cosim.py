import time
import requests

from terasim_cosim.constants import *
from terasim_cosim.redis_msgs import SUMOSignal, SUMOSignalDict
from terasim_cosim.redis_client_wrapper import create_redis_client


class McityOSTLSCosim:
    def __init__(self, server=None, api_key=None):
        # Website where the signal information can be obtained
        self.server = server
        self.api_key = api_key

        # Other constants which are helpful to encode the original signal timing plan
        self.signals_constants = {
            "NODE_18": {
                "intersection_name": "Liberty/State",
                "phase_order": [2, 4, 6, 8],
                "sumo_light": [["Gg", "yy", "rr"]] * 4,
            },
            "NODE_12": {
                "intersection_name": "Liberty/Wolverine",
                "phase_order": [2, 6, 4],
                "sumo_light": [["G", "y", "r"], ["Gg", "yy", "rr"], ["G", "y", "r"]],
            },
            "NODE_11": {
                "intersection_name": "Main/Wolverine",
                "phase_order": [1, 8, 2, 5, 4, 6],
                "sumo_light": [
                    ["g", "y", "r"],
                    ["Gg", "yy", "rr"],
                    ["G", "y", "r"],
                    ["G", "y", "r"],
                    ["Gg", "yy", "rr"],
                    ["G", "y", "r"],
                ],
            },
            "NODE_17": {
                "intersection_name": "Main/State",
                "phase_order": [6, 1, 8, 3, 2, 5, 4, 7],
                "sumo_light": [["G", "y", "r"]] * 6
                + [["GG", "yy", "rr"]]
                + [["G", "y", "r"]],
            },
            "NODE_23": {
                "intersection_name": "Main/Pontiac",
                "phase_order": [2, 4, 6],
                "sumo_light": [["Gg", "yy", "rr"], ["G", "y", "r"], ["G", "y", "r"]],
            },
            "NODE_24": {
                "intersection_name": "Liberty/Pontiac",
                "phase_order": [2, 4, 2],
                "sumo_light": [["Gg", "yy", "rr"], ["G", "y", "r"], ["G", "y", "r"]],
            },
        }
        self.state2int_web = {"green": 0, "yellow": 1, "red": 2}

        self.redis_client = create_redis_client(
            key_value_config={TLS_INFO: SUMOSignalDict}
        )

        print("McityOS traffic light synchronization started!")

    def sync_mcityos_tls_to_cosim(self):
        headers = {"accept": "application/json", "X-API-KEY": self.api_key}

        while True:
            try:
                response = requests.get(self.server + "/intersections", headers=headers)
                data = response.json()

                states = self.extract_signal(data)

                mcityos_tls_info = SUMOSignalDict()
                mcityos_tls_info.header.timestamp = time.time()
                for node_id in states:
                    signal = SUMOSignal()
                    signal.tls = states[node_id]
                    mcityos_tls_info.data[node_id] = signal

                self.redis_client.set(TLS_INFO, mcityos_tls_info)

                time.sleep(0.05)

            except Exception as e:
                print(f"Failed to update signal timing plan from Mcity: {e}")
                time.sleep(1.0)

    def extract_signal(self, json_obj):
        """Extract signal timing plan of all intersections from the Website.

        Args:
            json_obj (dict): All information of the studied intersections, including position and signal timing plan.

        Returns:
            str: Signal timing plan of all intersections in the format of string, which can be parsed by SUMO.
        """
        tls_dict = {}
        for signal_id in self.signals_constants:
            tls_dict[signal_id] = ""
            intersection_name = self.signals_constants[signal_id]["intersection_name"]
            for intersection in json_obj["intersections"]:
                if intersection["name"] == intersection_name:
                    signal_info = intersection
                    for i in range(
                        len(self.signals_constants[signal_id]["phase_order"])
                    ):
                        phase_id = self.signals_constants[signal_id]["phase_order"][i]
                        phase = signal_info["state"]["phases"][str(phase_id)]
                        c_index = self.state2int_web[phase["color"].lower()]
                        tls_dict[signal_id] += self.signals_constants[signal_id][
                            "sumo_light"
                        ][i][c_index]
                    break

        return tls_dict
