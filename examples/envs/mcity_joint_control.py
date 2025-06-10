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

    def on_start(self, ctx):
        """Description: The start function for the environment"""

        redis_client = redis.Redis(host="localhost", port=6379, db=0)
        redis_client.flushall()

        super().on_start(ctx)

        traci.vehicle.add(vehID="CAV", routeID="cav_route", typeID="NDE_URBAN")
        traci.vehicle.setColor("CAV", (255, 0, 0, 255))

        print("Terasim Started!")

    def on_step(self, ctx):
        """The main step function for the environment"""

        time_start = time.perf_counter()

        continue_flag = self.user_step(traci)

        time_observation = time.perf_counter()
        logger.info(
            f"Finished executing user step using {time_observation - time_start}s"
        )

        return continue_flag

    def on_stop(self, ctx):
        """Description: The stop function for the environment."""

        super.on_stop(ctx)
        print("Terasim Stopped!")
