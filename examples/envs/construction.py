import os

from terasim.overlay import traci
from loguru import logger
from terasim_nde_nade.envs.safetest_nade_with_av import SafeTestNADEWithAV
from sumolib.net import *
from sumolib.net.edge import *
from sumolib.net.lane import *
from sumolib.net.connection import *


class ConstructionCosim(SafeTestNADEWithAV):

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

        current_dir = os.path.dirname(os.path.abspath(__file__))
        net_path = os.path.join(current_dir, "..", "maps", "mcity.net.xml")
        net_path = os.path.normpath(net_path)
        self.sumonet_for_construction_zone: Net = readNet(
            net_path,
            withInternal=True,
        )

        self.rerouted_vehicles = set()
        self.no_need_reroute_vehicles = set()
        self.person_information: dict[str, dict] = {}
        self.lane_closed = False

        # self.reroute_vehicle(self.closed_lane_ids)
        super().on_start(ctx)
        traci.vehicle.setColor("CAV", (255, 0, 0, 255))

    ## helper functions for agent controlling
    def close_lane(self, lane_ids):
        for lane_id in lane_ids:
            traci.lane.setAllowed(lane_id, [])
        self.lane_closed = True

    def calculate_new_route(self, current_edge, destination_edge):
        # Usew Dijkstra to compute new path
        route = self.sumonet_for_construction_zone.getShortestPath(
            self.sumonet_for_construction_zone.getEdge(current_edge),
            self.sumonet_for_construction_zone.getEdge(destination_edge),
        )

        return [edge.getID() for edge in route[0]]

    def neccessary_reroute(self, vehicle_id, closed_lanes):
        # Retrieve the current route (list of edge IDs) for the vehicle.
        edge_ids = traci.vehicle.getRoute(vehicle_id)
        # Derive closed edges from the closed lane identifiers.
        closed_edges = [closed_lane.rsplit("_", 1)[0] for closed_lane in closed_lanes]
        # Determine the intersection between the vehicle's route and the closed edges.
        intersect_edges = set(closed_edges) & set(edge_ids)

        # If there is an intersection, check each affected edge.
        if intersect_edges:
            for edge in intersect_edges:
                # Retrieve the edge object. This assumes you have a method to get the edge details.
                edge_obj = self.sumonet_for_construction_zone.getEdge(edge)
                lanes = edge_obj.getLanes()  # List of lane objects for this edge.
                all_lanes_closed = True

                # Check each lane of the edge to see if at least one lane is still open.
                for lane_index in range(len(lanes)):
                    lane_id = f"{edge}_{lane_index}"
                    if lane_id not in closed_lanes:
                        all_lanes_closed = False
                        break  # No need to check further lanes for this edge.

                # If one of the edges in the vehicle's route is completely closed, reroute is necessary.
                if all_lanes_closed:
                    return True

        # If there is no intersection or every intersecting edge has at least one open lane,
        # then the route does not need to be changed.
        self.no_need_reroute_vehicles.add(vehicle_id)
        return False

    # Another version of reroute function
    def reroute_vehicle(self, closed_lanes):
        # Determine the closed edges by stripping the lane index from each closed lane id.
        closed_edges = [closed_lane.rsplit("_", 1)[0] for closed_lane in closed_lanes]

        for veh_id in traci.vehicle.getIDList():
            if veh_id == "CAV":
                None
            # Check if this vehicle's route contains any closed edge.
            if not self.neccessary_reroute(veh_id, closed_lanes):
                continue

            current_edge = traci.vehicle.getRoadID(veh_id)
            current_route = traci.vehicle.getRoute(veh_id)
            if not current_route:
                continue

            # If the vehicle is currently on a closed edge, try to force a lane change.
            if current_edge in closed_edges:
                try:
                    lane_index = traci.vehicle.getLaneIndex(veh_id)
                    # Retrieve the edge object and its lanes.
                    edge_obj = self.sumonet_for_construction_zone.getEdge(current_edge)
                    num_lanes = len(edge_obj.getLanes())
                    # Try to move the vehicle to an adjacent lane.
                    if lane_index < num_lanes - 1:
                        new_lane_index = lane_index + 1
                    elif lane_index > 0:
                        new_lane_index = lane_index - 1
                    else:
                        new_lane_index = lane_index

                    traci.vehicle.changeLane(veh_id, new_lane_index, 100)
                    logger.info(
                        f"Vehicle {veh_id} spawned on closed lane {lane_index}; the current edge {current_edge} has {num_lanes} lanes; forced lane change to lane {new_lane_index}"
                    )
                    # Skip further rerouting this step so that the lane change can take effect.
                    continue
                except Exception as e:
                    logger.error(f"Vehicle {veh_id} failed lane change: {e}")

            # Otherwise, compute a new route from the current (possibly open) edge to the destination.
            destination_edge = current_route[-1]
            new_route = traci.simulation.findRoute(current_edge, destination_edge)
            new_route = new_route.edges

            # Check that the new route is valid and actually differs from the current one.
            if new_route and new_route != current_route:
                traci.vehicle.setRoute(veh_id, new_route)
                self.rerouted_vehicles.add(veh_id)
                logger.info(
                    f"Vehicle {veh_id} rerouted from {current_route} to {new_route}"
                )
            else:
                logger.warning(
                    f"Vehicle {veh_id} could not be rerouted; no alternative route found"
                )

    def on_step(self, ctx):
        # self.closed_lane_ids = ["EG_1_1_1_0", "EG_24_1_1_0"]
        # self.close_lane_ids = []
        # self.handle_departing_vehicles()

        self.reroute_vehicle(self.closed_lane_ids)
        if not self.lane_closed:
            self.close_lane(self.closed_lane_ids)
            print("lanes closed")

        return super().on_step(ctx)
