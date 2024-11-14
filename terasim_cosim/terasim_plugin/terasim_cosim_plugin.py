import os
import math
import time
import sumolib
import lxml.etree as ET

from terasim.overlay import traci
from terasim.simulator import Simulator
from terasim_cosim.constants import *
from terasim_cosim.terasim_plugin.utils.convertion import *
from terasim_cosim.redis_client_wrapper import create_redis_client
from terasim_cosim.redis_msgs import Vehicle, VehicleDict


class TeraSimCoSimPlugin:

    def __init__(
        self,
        cosim_controlled_vehicle_keys=[],
        control_cav=False,
        keepRoute=1,
        setCAVSpeed=False,
    ):
        self._routes = set()

        self.UTM_offset = [-277600 + 102.89, -4686800 + 281.25, 0.0]

        self.control_cav = control_cav
        self.keepRoute = keepRoute
        self.setCAVSpeed = setCAVSpeed

        self.cosim_controlled_vehicle_keys = cosim_controlled_vehicle_keys

    def on_start(self, simulator: Simulator, ctx):
        key_value_config = {
            CAV_COSIM_VEHICLE_INFO: VehicleDict,
            TERASIM_COSIM_VEHICLE_INFO: VehicleDict,
        }
        for key in self.cosim_controlled_vehicle_keys:
            key_value_config[key] = VehicleDict
        self.redis_client = create_redis_client(key_value_config=key_value_config)

        # self.net = self.get_sumo_net(self.simulator.sumo_config_file_path)
        # self.init_cosim_route()

    def on_step(self, simulator: Simulator, ctx):
        if self.control_cav:
            self.sync_terasim_cav_to_cosim()
        else:
            self.sync_cosim_cav_to_terasim()

        self.sync_terasim_vehicle_to_cosim()
        # self.sync_cosim_vehicle_to_terasim()

        return True

    def on_stop(self, simulator: Simulator, ctx):
        pass

    def inject(self, simulator: Simulator, ctx):
        self.ctx = ctx
        self.simulator = simulator

        simulator.start_pipeline.hook("cosim_start", self.on_start, priority=-100)
        simulator.step_pipeline.hook("cosim_step", self.on_step, priority=-100)
        simulator.stop_pipeline.hook("cosim_stop", self.on_stop, priority=-100)

    def sync_terasim_cav_to_cosim(self):
        orientation = traci.vehicle.getAngle("CAV")
        orientation = sumo_heading_to_orientation(orientation)

        location = traci.vehicle.getPosition3D("CAV")

        x = location[0] - self.UTM_offset[0]
        y = location[1] - self.UTM_offset[1]
        z = location[2] - self.UTM_offset[2]

        length = traci.vehicle.getLength("CAV")
        width = traci.vehicle.getWidth("CAV")
        height = traci.vehicle.getHeight("CAV")

        x, y = sumo_coordinate_to_center_coordinate(x, y, orientation, length)

        speed = traci.vehicle.getSpeed("CAV")

        cav_cosim_vehicle_info = VehicleDict()
        cav_cosim_vehicle_info.header.timestamp = time.time()
        cav_cosim_vehicle_info.data["CAV"] = Vehicle(
            x=x,
            y=y,
            z=z,
            length=length,
            width=width,
            height=height,
            orientation=orientation,
            speed_long=speed,
        )

        self.redis_client.set(CAV_COSIM_VEHICLE_INFO, cav_cosim_vehicle_info)

    def sync_cosim_cav_to_terasim(self):
        try:
            cav_cosim_vehicle_info = self.redis_client.get(CAV_COSIM_VEHICLE_INFO)
        except:
            print("cav_cosim_vehicle_info not available. Exiting...")
            return

        if cav_cosim_vehicle_info:
            cav_info = cav_cosim_vehicle_info.data["CAV"]

            x = cav_info.x + self.UTM_offset[0]
            y = cav_info.y + self.UTM_offset[1]
            orientation = cav_info.orientation
            length = cav_info.length
            speed_long = cav_info.speed_long

            x, y = center_coordinate_to_sumo_coordinate(x, y, orientation, length)
            orientation = orientation_to_sumo_heading(orientation)

            if "CAV" in self.simulator.get_vehID_list():
                traci.vehicle.moveToXY(
                    "CAV", "", 0, x, y, angle=orientation, keepRoute=self.keepRoute
                )
                if self.setCAVSpeed:
                    traci.vehicle.setSpeedMode("CAV", 0)
                    traci.vehicle.setSpeed("CAV", speed_long)

    def sync_terasim_vehicle_to_cosim(self):
        """sync sumo controlled vehicle to cosim"""
        veh_list = self.simulator.get_vehID_list()

        terasim_controlled_vehicle_list = [
            vehID
            for vehID in veh_list
            if "BV" in vehID or "CV" in vehID or "POV" in vehID or "VRU" in vehID
        ]
        terasim_controlled_vehicle_info_with_timestamp = VehicleDict()
        terasim_controlled_vehicle_info_with_timestamp.header.timestamp = time.time()
        for vehID in terasim_controlled_vehicle_list:
            orientation = traci.vehicle.getAngle(vehID)
            orientation = sumo_heading_to_orientation(orientation)

            location = traci.vehicle.getPosition3D(vehID)

            x = location[0] - self.UTM_offset[0]
            y = location[1] - self.UTM_offset[1]
            z = location[2] - self.UTM_offset[2]

            length = traci.vehicle.getLength(vehID)
            width = traci.vehicle.getWidth(vehID)
            height = traci.vehicle.getHeight(vehID)

            x, y = sumo_coordinate_to_center_coordinate(x, y, orientation, length)

            speed = traci.vehicle.getSpeed(vehID)

            slope = traci.vehicle.getSlope(vehID)

            veh_info = Vehicle(
                x=x,
                y=y,
                z=z,
                length=length,
                width=width,
                height=height,
                orientation=orientation,
                speed_long=speed,
                slope=slope,
            )

            terasim_controlled_vehicle_info_with_timestamp.data[vehID] = veh_info

        self.redis_client.set(
            TERASIM_COSIM_VEHICLE_INFO,
            terasim_controlled_vehicle_info_with_timestamp,
        )

    def sync_cosim_vehicle_to_terasim(self):
        cosim_vehicle_id_record = []

        for key in self.cosim_controlled_vehicle_keys:
            try:
                cosim_controlled_vehicle_info = self.redis_client.get(key)
            except:
                print(key + " not available. Exiting...")
                continue

            if cosim_controlled_vehicle_info:
                data = cosim_controlled_vehicle_info.data

                veh_list = self.simulator.get_vehID_list()

                for vehID in data:
                    cosim_vehicle_id_record.append(vehID)

                    if vehID not in veh_list:
                        vclass = traci.vehicletype.getVehicleClass("NDE_URBAN")
                        # if vclass not in self._routes:
                        #     print("Creating route for %s vehicle class", vclass)
                        #     allowed_edges = [
                        #         e for e in self.net.getEdges() if e.allows(vclass)
                        #     ]
                        #     if allowed_edges:
                        #         traci.route.add(
                        #             "cosim_route_{}".format(vclass),
                        #             [allowed_edges[0].getID()],
                        #         )
                        #         self._routes.add(vclass)
                        #     else:
                        #         print(
                        #             "Could not found a route for %s. No vehicle will be spawned in sumo",
                        #             "IDM_waymo_motion",
                        #         )

                        traci.vehicle.add(vehID, "cosim_route_{}".format(vclass))
                        traci.vehicle.setColor(vehID, (150, 255, 255, 255))

                    else:
                        x = data[vehID].x + self.UTM_offset[0]
                        y = data[vehID].y + self.UTM_offset[1]
                        orientation = data[vehID].orientation
                        length = data[vehID].length

                        x, y = center_coordinate_to_sumo_coordinate(
                            x, y, orientation, length
                        )
                        orientation = orientation_to_sumo_heading(orientation)

                        traci.vehicle.moveToXY(
                            vehID,
                            "",
                            0,
                            x,
                            y,
                            angle=orientation,
                            keepRoute=self.keepRoute,
                        )

        veh_list = self.simulator.get_vehID_list()
        for vehID in veh_list:
            if (
                ("BV" not in vehID)
                and ("CAV" not in vehID)
                and (vehID not in cosim_vehicle_id_record)
            ):
                traci.vehicle.unsubscribe(vehID)
                traci.vehicle.remove(vehID)

    def get_sumo_net(self, cfg_file):
        """
        Returns sumo net.

        This method reads the sumo configuration file and retrieve the sumo net filename to create the
        net.
        """
        cfg_file = os.path.join(os.getcwd(), cfg_file)

        tree = ET.parse(cfg_file)
        tag = tree.find("//net-file")
        if tag is None:
            return None

        net_file = os.path.join(os.path.dirname(cfg_file), tag.get("value"))
        print("Reading net file: %s", net_file)

        sumo_net = sumolib.net.readNet(net_file)
        return sumo_net

    def init_cosim_route(self):
        vclass = traci.vehicletype.getVehicleClass("NDE_URBAN")
        allowed_edges = [e for e in self.net.getEdges() if e.allows(vclass)]
        traci.route.add(
            "cosim_route_{}".format(vclass),
            [allowed_edges[0].getID()],
        )
