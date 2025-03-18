from terasim.overlay import traci
from loguru import logger
import random
import terasim.utils as utils
from terasim_nde_nade.envs.safetest_nade_with_av import SafeTestNADEWithAV
import math
from sumolib.net import *
from sumolib.net.edge import *
from sumolib.net.lane import *
from sumolib.net.connection import *
from abc import ABC, abstractmethod


class ExternalPerson(ABC):

    def __init__(self, id):
        self.id: str = id
        self.current_step: int = 0

    @staticmethod
    @abstractmethod
    def satisfy() -> bool:
        pass

    def action(self) -> int:
        if self.current_step == 0:
            self.on_begin()
        if self.until():
            self.on_end()
            return 1
        else:
            self.execute()
            self.current_step += 1
            return 0

    @abstractmethod
    def execute(self) -> None:
        pass

    @abstractmethod
    def until(self) -> bool:
        pass

    @abstractmethod
    def on_begin(self) -> None:
        pass

    @abstractmethod
    def on_end(self) -> None:
        pass


class JayWalkingPerson(ExternalPerson):

    def __init__(self, id, sumonet: Net):
        super().__init__(id)
        self.angle: float = None
        self.sumonet: Net = sumonet
        self.edge: str = traci.person.getRoadID(id)
        self.total_width: float = sum([lane.getWidth() for lane in sumonet.getEdge(self.edge).getLanes()])
        self.curr_distance: float = 0

    @staticmethod
    def satisfy(p_id):
        """
        条件：行人走在一条有车行道的路的人行道上，人行道在最右侧，中间没有任何隔断
        而且人离边的起终点都有一定距离
        """

        edge_id = traci.person.getRoadID(p_id)
        lane_id = traci.person.getLaneID(p_id)
        lane_position = traci.person.getLanePosition(p_id)
        lane_length = traci.lane.getLength(lane_id)
        if lane_position < 3 or lane_position > lane_length - 3:
            return False

        num_lanes: int = traci.edge.getLaneNumber(edge_id)
        has_sidewalk: bool = False
        has_road: bool = False
        have_separation: bool = False

        for i in range(num_lanes):
            lane_id = f"{edge_id}_{i}"
            allowed: list[str] = traci.lane.getAllowed(lane_id)
            disallowed: list[str] = traci.lane.getDisallowed(lane_id)
            if i == 0 and "pedestrian" in allowed:
                has_sidewalk = True
            if (
                "passenger" in allowed
                or "private" in allowed
                or "pedestrian" in disallowed
            ):
                has_road = True
            if i > 0 and i < num_lanes - 1 and "all" in disallowed:
                return False

        return has_sidewalk and has_road and (not have_separation)

    def on_begin(self):
        """让行人转一个方向，如果正向行走，就左转90度；否则右转90度"""
        print("call")

        def is_pedestrian_moving_forward(p_id):

            edge_id = traci.person.getRoadID(p_id)
            edge_shape = self.sumonet.getEdge(edge_id).getShape()
            edge_start = edge_shape[0]
            edge_end = edge_shape[-1]
            edge_angle = (
                -math.degrees(
                    math.atan(
                        (edge_end[0] - edge_start[0]) / (edge_end[1] - edge_start[1])
                    )
                )
                + 90
            ) % 360
            person_angle = traci.person.getAngle(self.id)
            print((edge_angle - person_angle) % 360)
            if (edge_angle - person_angle) % 360 < 180:
                return True
            else:
                return False

        current_angle = traci.person.getAngle(self.id)
        if is_pedestrian_moving_forward(self.id):
            self.angle = (current_angle - 90) % 360  # 左转
            print("turn left")
        # else:
        #     self.angle = (current_angle + 90) % 360  # 右转
        #     print("turn right")

    def execute(self):
        self.curr_distance += force_person_move_forward(self.id, mode=0b110, angle=self.angle)

    def until(self):
        """当行人走出一开始的边时，结束"""
        return self.curr_distance >= self.total_width
        # return self.current_step == 10

    def on_end(self):
        """结束时，直接删掉这个行人"""
        traci.person.remove(self.id)


class StopOnCrossingPerson(ExternalPerson):

    def __init__(self, id):
        super().__init__(id)
        self.total_time: int = max(int(random.gauss(10, 3)), 0)

    @staticmethod
    def satisfy(p_id, sumonet: Net) -> bool:
        edge_id = traci.person.getRoadID(p_id)
        return sumonet.getEdge(edge_id).getFunction() == "crossing"

    def on_begin(self) -> None:
        pass

    def execute(self):
        traci.person.setSpeed(self.id, 0)

    def until(self) -> bool:
        return traci.simulation.getDeltaT() * self.current_step >= self.total_time

    def on_end(self) -> None:
        traci.person.setSpeed(self.id, traci.person.getMaxSpeed(self.id))


class AlteratingRidingBike(ExternalPerson):

    def __init__(self, id) -> None:
        super().__init__(id)
        self.turn_angle = 15
        self.curr_angle = None
        self.step_number = None  

    @staticmethod
    def satisfy(p_id, sumonet: Net) -> bool:
        """
        1在车行道上行驶
        2不能太靠近当前边的终点"""
        
        edge_id = traci.person.getRoadID(p_id)
        is_edge_special = sumonet.getEdge(edge_id).isSpecial()
        if is_edge_special:
            return False
        
        lane_id = traci.person.getLaneID(p_id)
        allowed: list[str] = traci.lane.getAllowed(lane_id)
        if "bicycle" not in allowed:
            return False
        if allowed == ["bicycle"]:
            return False
        
        lane_length = traci.lane.getLength(lane_id)
        lane_position = traci.person.getLanePosition(p_id)
        if (lane_length - lane_position) < 60:
            return False
        
        return True
    
    def on_begin(self) -> None:
        lane_id = traci.person.getLaneID(self.id)
        lane_width = traci.lane.getWidth(lane_id)
        theta = math.radians(self.turn_angle)
        distance = lane_width / math.sin(theta)
        self.step_number = math.floor( distance / traci.person.getMaxSpeed(self.id) / traci.simulation.getDeltaT() )

    def execute(self) -> None:
        """开始时：让它左偏一定度数，然后直走；直走固定距离后，往右偏，再走固定步数；最后角度回归"""
        if self.current_step == 0:
            self.curr_angle = (traci.person.getAngle(self.id) - self.turn_angle) % 360
        if self.current_step == self.step_number:
            self.curr_angle = (traci.person.getAngle(self.id) + 2 * self.turn_angle) % 360
        elif self.current_step == 2 * self.step_number - 1:
            self.curr_angle = (traci.person.getAngle(self.id) + self.turn_angle) % 360

        force_person_move_forward(self.id, angle=self.curr_angle, mode=0b110)
            
    
    def until(self) -> bool:
        return self.current_step == 2 * self.step_number
    
    def on_end(self) -> None:
        pass
        

class RunRedLightPerson(ExternalPerson):

    def __init__(self, id):
        super().__init__(id)
        self.lane = traci.person.getLaneID(self.id)

    @staticmethod
    def satisfy(p_id) -> bool:
        """
        行人必须在一条被红绿灯控制的lane上，并且它的速度很小，我们就判断它在等红灯
        """
        # should be waiting
        if traci.person.getSpeed(p_id) > 0.2:
            return False

        lane_id = traci.person.getLaneID(p_id)
        tls_list = traci.trafficlight.getIDList()
        for tls_id in tls_list:
            controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
            if lane_id in controlled_lanes:
                if traci.person.getVehicleClass(p_id) == "pedestrian":
                    return True
                elif traci.person.getVehicleClass(p_id) == "bicycle":
                    if traci.lane.getLength(lane_id) - traci.person.getLanePosition(p_id) < 3:
                        return True
                
        return False
    
    def on_begin(self):
        pass

    def execute(self):
        force_person_move_forward(self.id, mode=0b011)

    def until(self) -> bool:
        return traci.person.getLaneID(self.id) != self.lane
    
    def on_end(self) -> None:
        pass
    

def force_person_move_forward(id, mode=0b110, speed=None, angle=None) -> float:
    """强制行人向特定方向以特定速度移动
    如果方向不给出，默认为当前方向
    如果速度不给出，默认为该行人的最大速度
    """

    current_pos = traci.person.getPosition(id)
    if speed == None:
        speed = traci.person.getMaxSpeed(id)
    distance = speed * (traci.simulation.getDeltaT())
    if angle == None:
        angle = traci.person.getAngle(id)
    radians = math.radians(angle)
    new_pos = (
        current_pos[0] + distance * math.sin(radians),
        current_pos[1] + distance * math.cos(radians),
    )

    traci.person.moveToXY(id,edgeID="",x=new_pos[0],y=new_pos[1],angle=angle,keepRoute=mode,matchThreshold=20)

    return math.dist(current_pos,new_pos)

class SafeTestNADEWithAVCosim(SafeTestNADEWithAV):

    def __init__(
        self,
        closed_lane_ids=None,
        user_step=None,
        *args,
        **kwargs,
    ):
        """Initialize the environment (Please do not change)."""
        super().__init__(*args, **kwargs)
        self.user_step = user_step
        self.closed_lane_ids = closed_lane_ids if closed_lane_ids else []

    def on_start(self, ctx):
        print("on start initializing agent controls")
        # self.sumonet_for_construction_zone: Net = readNet(
        #     "/media/mtl/2TB/syh_dev/Mcity-2.0-API-for-AV-motion-planning-main/examples/maps/Mcity_safetest/mcity_disallow_ped_adjusted_test.net.xml",
        #     withInternal=True,
        # )  ## TODO: Fix this
        self.sumonet_for_construction_zone: Net = readNet(
            "/media/mtl/2TB/syh_dev/Mcity-2.0-API-for-AV-motion-planning-main/examples/maps/Mcity_safetest/mcity.net.xml",
            withInternal=True,
        )
        self.rerouted_vehicles = set()
        self.no_need_reroute_vehicles = set()
        self.person_information: dict[str, dict] = {}
        self.external_person_list: dict[str, ExternalPerson] = {}
        self.lane_closed = False

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
        self.lane_closed = True

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
        if set(closed_edges) & set(
            edge_ids
        ):  # Intersection of closed_edges and edge_ids
            return True
        self.no_need_reroute_vehicles.add(vehicle_id)
        return False
    # def handle_departing_vehicles(self):
    #     # Get vehicles that are scheduled to be added this step
    #     closed_edges = [closed_lane.rsplit("_", 1)[0] for closed_lane in self.closed_lane_ids]
    #     departed_vehicles = traci.simulation.getDepartedIDList()
    #     for veh_id in departed_vehicles:
    #         departure_edge = traci.vehicle.getRoadID(veh_id)
    #         if departure_edge in closed_edges:
    #             # Calculate a new route from the departure edge to the vehicle's destination.
    #             # This could use your existing calculate_new_route function or any Dijkstra-based approach.
    #             destination_edge = traci.vehicle.getRoute(veh_id)[-1]
    #             try:
    #                 new_route = self.calculate_new_route(departure_edge, destination_edge)
    #                 traci.vehicle.setRoute(veh_id, new_route)
    #                 print(f"Vehicle {veh_id} rerouted pre-spawn from {departure_edge} to avoid closed lane.")
    #             except Exception as e:
    #                 print(f"Failed to reroute vehicle {veh_id}: {e}")
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
    ## Another version of reroute function
    # def reroute_vehicles(self, closed_lanes):
    #     # Determine the closed edges by stripping the lane index from each closed lane id.
    #     closed_edges = [closed_lane.rsplit("_", 1)[0] for closed_lane in closed_lanes]
        
    #     for veh_id in traci.vehicle.getIDList():
    #         # Check if this vehicle's route contains any closed edge.
    #         if not self.neccessary_reroute(veh_id, closed_lanes):
    #             continue

    #         current_edge = traci.vehicle.getRoadID(veh_id)
    #         current_route = traci.vehicle.getRoute(veh_id)
    #         if not current_route:
    #             continue

    #         # If the vehicle is currently on a closed edge, try to force a lane change.
    #         if current_edge in closed_edges:
    #             try:
    #                 lane_index = traci.vehicle.getLaneIndex(veh_id)
    #                 # Retrieve the edge object and its lanes.
    #                 edge_obj = self.sumonet_for_construction_zone.getEdge(current_edge)
    #                 num_lanes = len(edge_obj.getLanes())
    #                 # Try to move the vehicle to an adjacent lane.
    #                 if lane_index < num_lanes - 1:
    #                     new_lane_index = lane_index + 1
    #                 elif lane_index > 0:
    #                     new_lane_index = lane_index - 1
    #                 else:
    #                     new_lane_index = lane_index

    #                 traci.vehicle.changeLane(veh_id, new_lane_index, 100)
    #                 logger.info(
    #                     f"Vehicle {veh_id} spawned on closed lane {lane_index}; the current edge {current_edge} has {num_lanes} lanes; forced lane change to lane {new_lane_index}"
    #                 )
    #                 # Skip further rerouting this step so that the lane change can take effect.
    #                 continue
    #             except Exception as e:
    #                 logger.error(f"Vehicle {veh_id} failed lane change: {e}")
            
    #         # Otherwise, compute a new route from the current (possibly open) edge to the destination.
    #         destination_edge = current_route[-1]
    #         new_route = traci.simulation.findRoute(current_edge, destination_edge)
    #         new_route = new_route.edges
    #         # Check that the new route is valid and actually differs from the current one.
    #         if new_route and new_route != current_route:
    #             traci.vehicle.setRoute(veh_id, new_route)
    #             self.rerouted_vehicles.add(veh_id)
            #     logger.info(f"Vehicle {veh_id} rerouted from {current_route} to {new_route}")
            # else:
            #     logger.warning(f"Vehicle {veh_id} could not be rerouted; no alternative route found")

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
        # self.closed_lane_ids = ["EG_1_1_1_0", "EG_24_1_1_0"]
        # self.close_lane_ids = []
        # self.handle_departing_vehicles()
        self.reroute_vehicle(self.closed_lane_ids)
        if not self.lane_closed:
            self.close_lane(self.closed_lane_ids)
            print("lanes closed")
        self.vru_control()

        return super().on_step(ctx)
