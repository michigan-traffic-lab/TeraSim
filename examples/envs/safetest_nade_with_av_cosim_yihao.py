from terasim.overlay import traci
from loguru import logger
import random
import terasim.utils as utils
from terasim_nde_nade.envs.safetest_nade_with_av import SafeTestNADEWithAV
from sumolib.net import Net, readNet
from terasim_cosim.terasim_plugin.utils.VRU import (
    ExternalPerson,
    JayWalkingPerson,
    StopOnCrossingPerson,
    RunRedLightPerson,
    AlteratingRidingBike,
)


class SafeTestNADEWithAVCosim(SafeTestNADEWithAV):

    def on_start(self, ctx):
        print("on start initializing agent controls")
        # self.sumonet_for_construction_zone: Net = readNet(
        #     "/media/mtl/2TB/syh_dev/Mcity-2.0-API-for-AV-motion-planning-main/examples/maps/Mcity_safetest/mcity_disallow_ped_adjusted_test.net.xml",
        #     withInternal=True,
        # )  ## TODO: Fix this
        self.sumonet_for_construction_zone: Net = readNet("/media/mtl/2TB/syh_dev/Mcity-2.0-API-for-AV-motion-planning-main/examples/maps/Mcity_safetest/mcity.net.xml", withInternal=True)
        self.rerouted_vehicles = set()
        self.no_need_reroute_vehicles = set()
        self.person_information: dict[str, dict] = {}
        self.external_person_list: dict[str, ExternalPerson] = {}
        
        # self.reroute_vehicle(self.closed_lane_ids)
        super().on_start(ctx)

    def get_IS_prob(
        self,
        veh_id,
        ndd_control_command_dicts,
        maneuver_challenge_dicts,
        veh_ctx_dicts,
    ):
        return 0.0

    def should_continue_simulation(self):
        num_colliding_vehicles = self.simulator.get_colliding_vehicle_number()

        if "CAV" not in traci.vehicle.getIDList():
            logger.info("CAV left the simulation, stop the simulation.")
            self.record.update(
                {
                    "veh_1_id": None,
                    "veh_2_id": None,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "CAV_left",
                }
            )
            return False

        elif utils.get_time() >= self.warmup_time + self.run_time:
            logger.info("Simulation timeout, stop the simulation.")
            self.record.update(
                {
                    "veh_1_id": None,
                    "veh_2_id": None,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "timeout",
                }
            )
            return False

        if num_colliding_vehicles >= 2:
            colliding_vehicles = self.simulator.get_colliding_vehicles()
            veh_1_id = colliding_vehicles[0]
            veh_2_id = colliding_vehicles[1]
            self.record.update(
                {
                    "veh_1_id": veh_1_id,
                    "veh_1_obs": self.vehicle_list[veh_1_id].observation,
                    "veh_2_id": veh_2_id,
                    "veh_2_obs": self.vehicle_list[veh_2_id].observation,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "collision",
                }
            )
            return False

        return True

    ## helper functions for agent controlling
    def close_lane(self, lane_ids):
        for lane_id in lane_ids:
            traci.lane.setAllowed(lane_id, [])

    def calculate_new_route(self, current_edge, destination_edge):
        """
        计算新的路线。
        使用Dijkstra算法计算新的路线，同时排除关闭的车道。
        """

        # 使用Dijkstra算法计算新的路线
        route = self.sumonet_for_construction_zone.getShortestPath(
            self.sumonet_for_construction_zone.getEdge(current_edge),
            self.sumonet_for_construction_zone.getEdge(destination_edge),
        )

        return [edge.getID() for edge in route[0]]
    
    def neccessary_reroute(self, vehicle_id, closed_lanes):
        edge_ids = traci.vehicle.getRoute(vehicle_id)
        closed_edges = [closed_lane.rsplit("_", 1)[0] for closed_lane in closed_lanes]
        if set(closed_edges) & set(edge_ids):  # Intersection of closed_edges and edge_ids
            return True
        self.no_need_reroute_vehicles.add(vehicle_id)
        return False

    def reroute_vehicle(self, closed_lanes):

        def handle_spawned_vehicle(vehicle_id, closed_lanes, sumo_net):
            """
            Check if a vehicle is spawned at a closed lane and move it to the first open lane on its original route.

            :param vehicle_id: ID of the vehicle to check.
            :param closed_lanes: List of closed lane IDs.
            :param sumo_net: SUMO network object.
            """

            def is_closed_lane(lane_id):
                return lane_id in closed_lanes

            def find_first_open_lane_on_route(route):
                """
                Find the first open lane along the vehicle's original route.

                :param route: The original route of the vehicle (list of edge IDs).
                :return: The first open lane ID along the route, or None if no open lane is found.
                """
                for edge_id in route:
                    edge = sumo_net.getEdge(edge_id)
                    for lane in edge.getLanes():
                        if not is_closed_lane(lane.getID()):
                            return lane.getID()
                return None

            print("handle spawned vehicle")
            current_lane = traci.vehicle.getLaneID(vehicle_id)
            original_route = traci.vehicle.getRoute(vehicle_id)

            if is_closed_lane(current_lane):
                nearby_open_lane = find_first_open_lane_on_route(original_route)

                if nearby_open_lane:
                    print(
                        f"Vehicle {vehicle_id} spawned at closed lane {current_lane}. Moving to first open lane {nearby_open_lane} on original route."
                    )
                    # Move the vehicle to the nearby open lane
                    traci.vehicle.moveToLane(vehicle_id, nearby_open_lane)
                else:
                    print(
                        f"Vehicle {vehicle_id} spawned at closed lane {current_lane}, but no open lane found on its original route."
                    )
            else:
                print(f"Vehicle {vehicle_id} spawned at open lane {current_lane}.")

        print("entered reroute_vehicle")
        vehicle_ids = traci.vehicle.getIDList()
        for vehicle_id in vehicle_ids:
            if (
                vehicle_id in self.rerouted_vehicles
                or vehicle_id in self.no_need_reroute_vehicles
            ):
                continue

            if not self.neccessary_reroute(vehicle_id, closed_lanes):
                continue

            if vehicle_id in self.no_need_reroute_vehicles:
                continue

            # handle_spawned_vehicle(
            #     vehicle_id, closed_lanes, self.sumonet_for_construction_zone
            # )
            current_edge = traci.vehicle.getRoadID(vehicle_id)
            destination_edge = traci.vehicle.getRoute(vehicle_id)[-1]
            try:
                new_route = self.calculate_new_route(current_edge, destination_edge)
                print(
                    f"Rerouting vehicle {vehicle_id} from {current_edge} to {destination_edge}"
                )
                print(f"New route: {new_route}")
                traci.vehicle.setRoute(vehicle_id, new_route)
                self.rerouted_vehicles.add(vehicle_id)
            except Exception as e:
                print(f"Failed to reroute vehicle {vehicle_id}: {e}")

    def vru_control(self):

        # print(
        #     f"-----------step {traci.simulation.getTime()//traci.simulation.getDeltaT()}, time {traci.simulation.getTime()}---------"
        # )
        people_list = self.ped_list
        for id in people_list:
            if id not in self.person_information:
                self.person_information[id] = {
                    "depart-time": traci.simulation.getTime()
                }
        bike_list = [
            id for id in people_list if traci.person.getVehicleClass(id) == "bicycle"
        ]
        person_list = [
            id
            for id in people_list
            if traci.person.getVehicleClass(id) == "pedestrian" and "vpf_" in id
        ]
        # person_list = []
        for p_id in person_list:
            info = f"PERSON {p_id}: road {traci.person.getRoadID(p_id)} \t lane {traci.person.getLaneID(p_id)} \t speed {traci.person.getSpeed(p_id)} \t vclass {traci.person.getVehicleClass(p_id)}"
            if p_id in self.external_person_list:
                info += f" \t EXTERNAL {type(self.external_person_list[p_id])}"
            # print(info)
        for p_id in bike_list:
            info = f"BIKE {p_id}: road {traci.person.getRoadID(p_id)} \t lane {traci.person.getLaneID(p_id)} \t speed {traci.person.getSpeed(p_id)} \t vclass {traci.person.getVehicleClass(p_id)}"
            if p_id in self.external_person_list:
                info += f" \t EXTERNAL {type(self.external_person_list[p_id])}"
            # print(info)
        # print("-----------------")
        # --------乱穿马路----------
        for p_id in person_list:
            if p_id not in self.external_person_list and JayWalkingPerson.satisfy(p_id):
                if random.random() < 0.2:  # FIXME
                    self.external_person_list[p_id] = JayWalkingPerson(
                        p_id, self.sumonet
                    )
        ## --------停在路口----------
        for p_id in person_list:
            if p_id not in self.external_person_list and StopOnCrossingPerson.satisfy(
                p_id, self.sumonet
            ):
                if random.random() < 0.2:  # FIXME
                    self.external_person_list[p_id] = StopOnCrossingPerson(p_id)
        # --------闯红灯--------
        for p_id in person_list:
            if p_id not in self.external_person_list and RunRedLightPerson.satisfy(
                p_id
            ):
                if random.random() < 0.2:  # FIXME
                    self.external_person_list[p_id] = RunRedLightPerson(p_id)
        # ------自行车跑到车道左边
        for p_id in bike_list:
            if p_id not in self.external_person_list and AlteratingRidingBike.satisfy(
                p_id, self.sumonet
            ):
                if random.random() < 0.2:  # FIXME
                    # print(f"adding {p_id} into external riding bikes")
                    self.external_person_list[p_id] = AlteratingRidingBike(p_id)
        # --------执行--------
        action_result = {
            p_id: controller.action()
            for p_id, controller in self.external_person_list.items()
        }
        for p_id, result in action_result.items():
            if result == 1:
                self.external_person_list.pop(p_id)

    def on_step(self, ctx):
        ## TODO: 控制agent代码
        print("on_step control agent")
        self.ped_list = traci.person.getIDList()
        self.closed_lane_ids = ["EG_1_1_1_0", "EG_24_1_1_0"]
        self.reroute_vehicle(self.closed_lane_ids)
        self.close_lane(self.closed_lane_ids)
        print("lanes closed")

        return super().on_step(ctx)
