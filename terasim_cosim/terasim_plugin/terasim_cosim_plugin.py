import time

from terasim.overlay import traci
from terasim.simulator import Simulator

from terasim_cosim.terasim_plugin.utils import *

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client
from terasim_cosim.redis_msgs import Vehicle, VehicleDict


class TeraSimCosimPlugin:

    def __init__(
        self,
        remote_flag=False,
        control_cav=False,
        keepRoute=2,
        CAVSpeedOverride=True,
        pub_channels=[],
        sub_channels=[],
        latency_src_channels=[],
    ):
        self.remote_flag = remote_flag
        self.control_cav = control_cav
        self.keepRoute = keepRoute
        self.CAVSpeedOverride = CAVSpeedOverride

        self.pub_channels = pub_channels
        self.sub_channels = sub_channels
        self.latency_src_channels = latency_src_channels

    def on_start(self, simulator: Simulator, ctx):
        key_value_config = {
            CAV_COSIM_VEHICLE_INFO: VehicleDict,
            TERASIM_COSIM_VEHICLE_INFO: VehicleDict,
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

        self.sync_terasim_vehicle_to_cosim()

        return True

    def on_stop(self, simulator: Simulator, ctx):
        pass

    def inject(self, simulator: Simulator, ctx):
        self.ctx = ctx
        self.simulator = simulator

        simulator.start_pipeline.hook("cosim_start", self.on_start, priority=-100)
        simulator.step_pipeline.hook("cosim_step", self.on_step, priority=-100)
        simulator.stop_pipeline.hook("cosim_stop", self.on_stop, priority=-100)

    def sync_cosim_cav_to_terasim(self):
        try:
            cav_cosim_vehicle_info = self.redis_client.get(CAV_COSIM_VEHICLE_INFO)
        except:
            print("cav_cosim_vehicle_info not available. Exiting...")
            return

        if cav_cosim_vehicle_info:
            cav_info = cav_cosim_vehicle_info.data["CAV"]

            length = cav_info.length
            speed_long = cav_info.speed_long

            orientation = cav_info.orientation

            x, y = utm_to_sumo_coordinate(cav_info.x, cav_info.y)
            x, y = center_coordinate_to_front_coordinate(x, y, orientation, length)

            orientation = orientation_to_sumo_heading(orientation)

            if "CAV" in self.simulator.get_vehID_list():
                traci.vehicle.moveToXY(
                    "CAV",
                    "",
                    0,
                    x,
                    y,
                    angle=orientation,
                    keepRoute=self.keepRoute,
                )
                if self.CAVSpeedOverride:
                    traci.vehicle.setSpeedMode("CAV", 0)
                    traci.vehicle.setSpeed("CAV", speed_long)

    def sync_terasim_cav_to_cosim(self):
        length = traci.vehicle.getLength("CAV")
        width = traci.vehicle.getWidth("CAV")
        height = traci.vehicle.getHeight("CAV")

        orientation = traci.vehicle.getAngle("CAV")
        orientation = sumo_heading_to_orientation(orientation)

        x, y, z = traci.vehicle.getPosition3D("CAV")
        x, y = front_coordinate_to_center_coordinate(x, y, orientation, length)
        x, y = sumo_to_utm_coordinate(x, y)

        speed = traci.vehicle.getSpeed("CAV")

        cav_cosim_vehicle_info = VehicleDict()
        cav_cosim_vehicle_info.header.timestamp = time.time()
        cav_cosim_vehicle_info.data["CAV"] = Vehicle(
            x=x,
            y=y,
            length=length,
            width=width,
            height=height,
            orientation=orientation,
            speed_long=speed,
        )

        self.redis_client.set(CAV_COSIM_VEHICLE_INFO, cav_cosim_vehicle_info)

    def sync_terasim_vehicle_to_cosim(self):
        """sync sumo controlled vehicle to cosim"""
        veh_list = self.simulator.get_vehID_list()

        terasim_controlled_vehicle_list = [
            vehID for vehID in veh_list if vehID != "CAV"
        ]

        terasim_controlled_vehicle_info = VehicleDict()
        terasim_controlled_vehicle_info.header.timestamp = time.time()

        for vehID in terasim_controlled_vehicle_list:
            length = traci.vehicle.getLength(vehID)
            width = traci.vehicle.getWidth(vehID)
            height = traci.vehicle.getHeight(vehID)

            speed = traci.vehicle.getSpeed(vehID)
            slope = traci.vehicle.getSlope(vehID)

            orientation = traci.vehicle.getAngle(vehID)
            orientation = sumo_heading_to_orientation(orientation)

            x, y, z = traci.vehicle.getPosition3D(vehID)

            x, y = front_coordinate_to_center_coordinate(x, y, orientation, length)
            x, y = sumo_to_utm_coordinate(x, y)

            veh_info = Vehicle(
                x=x,
                y=y,
                length=length,
                width=width,
                height=height,
                orientation=orientation,
                speed_long=speed,
                slope=slope,
            )

            terasim_controlled_vehicle_info.data[vehID] = veh_info

        self.redis_client.set(
            TERASIM_COSIM_VEHICLE_INFO,
            terasim_controlled_vehicle_info,
        )
