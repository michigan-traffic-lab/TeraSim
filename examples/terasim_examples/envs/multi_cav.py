import time
import redis

from loguru import logger
from terasim.envs.template import EnvTemplate
from terasim.overlay import traci
from terasim.vehicle.controllers.high_efficiency_controller import (
    HighEfficiencyController,
)
from terasim.vehicle.decision_models.idm_model import IDMModel
from terasim.vehicle.factories.vehicle_factory import VehicleFactory
from terasim.vehicle.sensors.ego import EgoSensor
from terasim.vehicle.vehicle import Vehicle

from utils import *
from terasim_cosim.redis_msgs import Actor, ActorDict


class ExampleVehicleFactory(VehicleFactory):

    def create_vehicle(self, veh_id, simulator):
        sensor_list = [EgoSensor()]
        decision_model = IDMModel()
        controller = HighEfficiencyController(simulator)

        return Vehicle(
            veh_id,
            simulator,
            sensors=sensor_list,
            decision_model=decision_model,
            controller=controller,
        )


class TeraSimEnvForUser(EnvTemplate):
    def __init__(
        self,
        user_step=None,
        *args,
        **kwargs,
    ):
        """Initialize the environment (Please do not change)."""
        super().__init__(*args, **kwargs)
        self.user_step = user_step

        key_value_config = {
            "cav_1_info": ActorDict,
            "cav_2_info": ActorDict,
        }

        self.redis_client = create_redis_client(
            key_value_config=key_value_config,
            remote_flag=False,
            pub_channels=[],
            sub_channels=[],
            latency_src_channels=[],
        )

    def sync_custom_cav_to_terasim(self, CAV_ID, redis_key):
        try:
            veh_info = self.redis_client.get(redis_key)
        except:
            print(f"{redis_key} not available. Exiting...")
            return

        if veh_info:
            veh_info = veh_info.data[CAV_ID]

            x = veh_info.x + UTM_OFFSET[0]
            y = veh_info.y + UTM_OFFSET[1]
            orientation = veh_info.orientation
            length = veh_info.length
            speed_long = veh_info.speed_long

            x, y = center_coordinate_to_front_coordinate(x, y, orientation, length)
            orientation = orientation_to_sumo_heading(orientation)

            if CAV_ID in self.simulator.get_vehID_list():
                traci.vehicle.moveToXY(CAV_ID, "", 0, x, y, angle=orientation, keepRoute=1)

            traci.vehicle.setSpeedMode(CAV_ID, 0)
            traci.vehicle.setSpeed(CAV_ID, speed_long)

    def sync_custom_cav_to_cosim(self, CAV_ID, redis_key):
        orientation = traci.vehicle.getAngle(CAV_ID)
        orientation = sumo_heading_to_orientation(orientation)

        location = traci.vehicle.getPosition3D(CAV_ID)
        location = (location[0], location[1], location[2] if len(location) > 2 else 0.0)

        x = location[0] - UTM_OFFSET[0]
        y = location[1] - UTM_OFFSET[1]
        z = location[2] - UTM_OFFSET[2]

        length = traci.vehicle.getLength(CAV_ID)
        width = traci.vehicle.getWidth(CAV_ID)
        height = traci.vehicle.getHeight(CAV_ID)

        x, y = front_coordinate_to_center_coordinate(x, y, orientation, length)

        speed = traci.vehicle.getSpeed(CAV_ID)

        cav_info = ActorDict()
        cav_info.header.timestamp = time.time()
        cav_info.data[CAV_ID] = Actor(
            x=x,
            y=y,
            z=z,
            length=length,
            width=width,
            height=height,
            orientation=orientation,
            speed_long=speed,
        )

        self.redis_client.set(redis_key, cav_info)

    def on_start(self, ctx):
        """Description: The start function for the environment"""

        redis_client = redis.Redis(host="localhost", port=6379, db=0)
        redis_client.flushall()

        super().on_start(ctx)

        traci.vehicle.add(vehID="CAV", routeID="cav_route", typeID="NDE_URBAN")
        traci.vehicle.setColor("CAV", (255, 0, 0, 255))
        traci.vehicle.setSpeed("CAV", 0.0)

        traci.vehicle.add(vehID="CAV_1", routeID="r_0", typeID="NDE_URBAN")
        traci.vehicle.setColor("CAV_1", (0, 255, 0, 255))

        traci.vehicle.add(vehID="CAV_2", routeID="r_1", typeID="NDE_URBAN")
        traci.vehicle.setColor("CAV_2", (0, 255, 0, 255))

        print("Terasim Started!")

    def on_step(self, ctx):
        """The main step function for the environment"""

        time_start = time.perf_counter()

        continue_flag = self.user_step(traci)

        self.sync_custom_cav_to_terasim(CAV_ID="CAV_1", redis_key="cav_1_info")
        self.sync_custom_cav_to_terasim(CAV_ID="CAV_2", redis_key="cav_2_info")

        # self.sync_custom_cav_to_cosim(CAV_ID="CAV_1", redis_key="cav_1_info")
        # self.sync_custom_cav_to_cosim(CAV_ID="CAV_2", redis_key="cav_2_info")

        time_observation = time.perf_counter()
        logger.info(
            f"Finished executing user step using {time_observation - time_start}s"
        )

        return continue_flag

    def on_stop(self, ctx):
        """Description: The stop function for the environment."""

        super.on_stop(ctx)
        print("Terasim Stopped!")
