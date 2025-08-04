from terasim.agent.agent_controller import AgentController
import terasim.utils as utils
import random
from .nde_vehicle_utils import (
    get_next_lane_edge,
    get_neighbour_lane,
    Command,
    NDECommand,
    TrajectoryPoint,
    interpolate_future_trajectory,
)
from terasim.overlay import traci
from addict import Dict
from loguru import logger


def get_all_routes():
    return traci.route.getIDList()


def get_all_route_edges():
    all_routes = get_all_routes()
    all_route_edges = {}
    for route in all_routes:
        all_route_edges[route] = traci.route.getEdges(route)
    return all_route_edges


class NDEController(AgentController):

    def __init__(self, simulator, params=None):
        self.is_busy = False
        self.cached_control_command = None  # this is a dict, containing the control command for the vehicle with the timestep information
        return super().__init__(
            simulator, control_command_schema=NDECommand, params=params
        )

    def _update_controller_status(self, veh_id, current_time=None):
        """Refresh the state of the controller. This function will be called at each timestep as far as vehicle is still in the simulator, even if the vehicle is not controlled."""
        # if the controller is busy, detect if the current simulation time - the time of the cached control command is greater than the duration of the control command, then the controller is not busy anymore
        if self.is_busy:
            current_time = (
                traci.simulation.getTime() if current_time is None else current_time
            )
            if (
                current_time - self.cached_control_command.timestep
                > self.cached_control_command.cached_command.duration
            ):
                self.is_busy = False
                self.cached_control_command = None
                self.all_checks_on(veh_id)

    def execute_control_command(self, veh_id, control_command, obs_dict):
        """Vehicle acts based on the input action.

        Args:
            action (dict): Lonitudinal and lateral actions. It should have the format: {'longitudinal': float, 'lateral': str}. The longitudinal action is the longitudinal acceleration, which should be a float. The lateral action should be the lane change direction. 'central' represents no lane change. 'left' represents left lane change, and 'right' represents right lane change.
        """
        if not self.is_busy:
            if control_command.command_type == Command.DEFAULT:
                # all_checks_on(veh_id)
                return
            elif control_command.command_type == Command.CUSTOM:
                self.cached_control_command = Dict(
                    {
                        "timestep": traci.simulation.getTime(),
                        "cached_command": control_command,
                    }
                )
                self.execute_control_command_onestep(
                    veh_id, self.cached_control_command, obs_dict, first_step=True
                )
                return
            else:
                self.all_checks_off(veh_id)
                # other commands will have duration, which will keep the controller busy
                self.is_busy = True
                # if the control command is a trajectory, then interpolate the trajectory
                control_command = interpolate_control_command(control_command)
                self.cached_control_command = Dict(
                    {
                        "timestep": traci.simulation.getTime(),
                        "cached_command": control_command,
                    }
                )
                self.execute_control_command_onestep(
                    veh_id, self.cached_control_command, obs_dict, first_step=True
                )
        else:
            self.execute_control_command_onestep(
                veh_id, self.cached_control_command, obs_dict, first_step=False
            )

    def execute_control_command_onestep(
        self, veh_id, cached_control_command, obs_dict, first_step=False
    ):
        if cached_control_command["cached_command"].command_type == Command.CUSTOM:
            if (
                cached_control_command["cached_command"].custom_control_command
                is not None
                and cached_control_command[
                    "cached_command"
                ].custom_execute_control_command
                is not None
            ):
                cached_control_command["cached_command"].custom_execute_control_command(
                    veh_id,
                    cached_control_command["cached_command"].custom_control_command,
                    obs_dict,
                )
                return
            else:
                logger.error(
                    "Custom control command or Custom control command execution is not defined"
                )
                return

        if cached_control_command["cached_command"].command_type == Command.TRAJECTORY:
            # pass
            self.execute_trajectory_command(
                veh_id, cached_control_command["cached_command"], obs_dict
            )
        elif (
            cached_control_command["cached_command"].command_type == Command.LEFT
            or cached_control_command["cached_command"].command_type == Command.RIGHT
        ):
            self.execute_lane_change_command(
                veh_id,
                cached_control_command["cached_command"],
                obs_dict,
                first_step=first_step,
            )
        elif (
            cached_control_command["cached_command"].command_type
            == Command.ACCELERATION
        ):
            self.execute_acceleration_command(
                veh_id, cached_control_command["cached_command"], obs_dict
            )
        else:
            logger.error("Invalid command type")
        return

    @staticmethod
    def execute_trajectory_command(veh_id, control_command, obs_dict):
        assert control_command.command_type == Command.TRAJECTORY
        # get the closest timestep trajectory point in control_command.trajectory to current timestep
        trajectory_array = control_command.future_trajectory
        current_timestep = traci.simulation.getTime()
        closest_timestep_trajectory = min(
            trajectory_array, key=lambda x: abs(x[-1] - current_timestep)
        )
        # set the position of the vehicle to the closest timestep trajectory point
        traci.vehicle.moveToXY(
            vehID=veh_id,
            edgeID="",
            laneIndex=-1,
            x=closest_timestep_trajectory[0],
            y=closest_timestep_trajectory[1],
            # angle=closest_timestep_trajectory[2],
            keepRoute=1,
        )
        logger.info(
            f"Setting position of {veh_id} to {closest_timestep_trajectory[0], closest_timestep_trajectory[1]}, current position is {traci.vehicle.getPosition(veh_id)}"
        )
        traci.vehicle.setPreviousSpeed(veh_id, closest_timestep_trajectory[3])

    @staticmethod
    def execute_lane_change_command(
        veh_id, control_command, obs_dict, first_step=False
    ):
        assert (
            control_command.command_type == Command.LEFT
            or control_command.command_type == Command.RIGHT
        )
        if first_step:  # only execute lane change command once
            indexOffset = 1 if control_command.command_type == Command.LEFT else -1
            traci.vehicle.changeLaneRelative(veh_id, indexOffset, utils.get_step_size())

    @staticmethod
    def execute_acceleration_command(veh_id, control_command, obs_dict):
        # logger.critical("the acceleration command should not be executed")
        assert control_command.command_type == Command.ACCELERATION
        acceleration = control_command.acceleration
        final_speed = obs_dict["ego"]["velocity"] + acceleration * utils.get_step_size()
        final_speed = 0 if final_speed < 0 else final_speed
        traci.vehicle.setSpeed(veh_id, final_speed)

    @staticmethod
    def all_checks_on(veh_id):
        traci.vehicle.setSpeedMode(veh_id, 31)
        traci.vehicle.setLaneChangeMode(veh_id, 1621)

    @staticmethod
    def all_checks_off(veh_id):
        traci.vehicle.setSpeedMode(veh_id, 32)
        traci.vehicle.setLaneChangeMode(veh_id, 0)


def interpolate_control_command(control_command):
    if control_command.command_type == Command.TRAJECTORY:
        control_command.future_trajectory = interpolate_future_trajectory(
            control_command.future_trajectory, 0.1
        )  # TODO: Angle cannot be interpolated
        return control_command
    else:
        return control_command
