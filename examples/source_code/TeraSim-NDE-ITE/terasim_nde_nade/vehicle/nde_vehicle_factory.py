from terasim.vehicle.factories.vehicle_factory import VehicleFactory
from terasim.vehicle.sensors.ego import EgoSensor
from terasim.vehicle.sensors.local import LocalSensor
from terasim.vehicle.vehicle import Vehicle
from terasim_nde_nade.vehicle.nde_controller import NDEController
from terasim_nde_nade.vehicle.aggressive_controller import AggressiveController
from terasim_nde_nade.vehicle.nde_decision_model import NDEDecisionModel
from terasim_nde_nade.vehicle.conflict_generation_model import ConflictGenerationModel
from terasim_nde_nade.vehicle.nde_ego_sensor import NDEEgoSensor
import json


class NDEVehicleFactory(VehicleFactory):

    def create_vehicle(self, veh_id, simulator):
        sensor_list = [
            NDEEgoSensor(cache=True, cache_history=True, cache_history_duration=1)
        ]
        if veh_id == "CAV":
            decision_model = NDEDecisionModel(
                MOBIL_lc_flag=True,
                stochastic_acc_flag=False,
                reroute=False,
                dynamically_change_vtype=False,
            )
        else:
            decision_model = ConflictGenerationModel(
                MOBIL_lc_flag=True, stochastic_acc_flag=False
            )

        controller = NDEController(simulator)
        return Vehicle(
            veh_id,
            simulator,
            sensors=sensor_list,
            decision_model=decision_model,
            controller=controller,
        )
