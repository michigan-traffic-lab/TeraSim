from terasim_nde_nade.envs.safetest_nade import SafeTestNADE
from terasim.overlay import traci
import terasim.utils as utils
import numpy as np
from terasim_nde_nade.vehicle.nde_vehicle_utils import (
    NDECommand,
    Command,
    get_collision_type_and_prob,
    is_car_following,
)
from loguru import logger
from addict import Dict
import copy
from terasim.envs.template import EnvTemplate


class SafeTestNADEWithAV(SafeTestNADE):

    def __init__(self, cache_radius=100, control_radius=50, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cache_radius = cache_radius
        self.control_radius = control_radius

    def cache_vehicle_history(self):
        cav_context = traci.vehicle.getContextSubscriptionResults("CAV")
        if cav_context:
            cached_veh_ids = cav_context.keys()
            for veh_id in cached_veh_ids:
                _ = self.vehicle_list[
                    veh_id
                ].observation  # get the observation of the vehicle to cache it
            # filter the cached_veh_ids by the controlled radius to controlled_veh_ids
            controlled_veh_ids = []
            for veh_id in cav_context:
                if (
                    cav_context[veh_id][traci.constants.VAR_DISTANCE]
                    <= self.control_radius
                ):
                    controlled_veh_ids.append(veh_id)
            return cached_veh_ids, controlled_veh_ids
        else:
            raise ValueError("CAV context is empty")

    def on_start(self, ctx):
        # initialize the surrogate model and add AV to env
        super().on_start(ctx)
        self.add_cav()

    def add_cav(self):
        self.add_vehicle(
            veh_id="CAV",
            route_id="cav_route",
            lane="best",
            lane_id="EG_35_1_14_0",
            position=0,
            speed=0,
        )
        # set the CAV with white color
        traci.vehicle.setColor("CAV", (255, 255, 255, 255))

        traci.vehicle.subscribeContext(
            "CAV",
            traci.constants.CMD_GET_VEHICLE_VARIABLE,
            self.cache_radius,
            [traci.constants.VAR_DISTANCE],
        )

    def on_step(self, ctx):
        cached_veh_ids, controlled_veh_ids = self.cache_vehicle_history()
        # ctx["terasim_controlled_vehicle_ids"] = set(controlled_veh_ids)
        return super().on_step(ctx)

    def reroute_vehicle_if_necessary(self, veh_id, veh_ctx_dicts, obs_dicts):
        if veh_id == "CAV":
            logger.debug("CAV will not be rerouted.")
            return False
        super().reroute_vehicle_if_necessary(veh_id, veh_ctx_dicts, obs_dicts)

    def NADE_decision(self, control_command_dicts, veh_ctx_dicts, obs_dicts):
        predicted_CAV_control_command = self.predict_cav_control_command(
            control_command_dicts, veh_ctx_dicts, obs_dicts
        )
        if veh_ctx_dicts["CAV"] is None:
            veh_ctx_dicts["CAV"] = Dict()
        if predicted_CAV_control_command is not None:
            veh_ctx_dicts["CAV"]["ndd_command_distribution"] = Dict(
                {
                    "negligence": predicted_CAV_control_command,
                    "normal": NDECommand(command_type=Command.DEFAULT, prob=0),
                }
            )
        else:
            veh_ctx_dicts["CAV"]["ndd_command_distribution"] = Dict(
                {
                    "normal": NDECommand(command_type=Command.DEFAULT, prob=1),
                }
            )
        CAV_command_cache = copy.deepcopy(control_command_dicts["CAV"])
        control_command_dicts["CAV"] = NDECommand(command_type=Command.DEFAULT, prob=1)
        (
            ITE_control_command_dicts,
            veh_ctx_dicts,
            weight,
            trajectory_dicts,
            maneuver_challenge_dicts,
            criticality_dicts,
        ) = super().NADE_decision(control_command_dicts, veh_ctx_dicts, obs_dicts)
        ITE_control_command_dicts["CAV"] = CAV_command_cache
        return (
            ITE_control_command_dicts,
            veh_ctx_dicts,
            weight,
            trajectory_dicts,
            maneuver_challenge_dicts,
            criticality_dicts,
        )

    def NADE_importance_sampling(
        self,
        ndd_control_command_dicts,
        maneuver_challenge_dicts,
        veh_ctx_dicts,
        exclude_IS_veh_set=None,
    ):
        exclude_IS_veh_set = set(["CAV"])
        return super().NADE_importance_sampling(
            ndd_control_command_dicts,
            maneuver_challenge_dicts,
            veh_ctx_dicts,
            exclude_IS_veh_set,
        )

    def apply_collision_avoidance(
        self, trajectory_dicts, veh_ctx_dicts, ITE_control_command_dict
    ):
        # add another layer, if CAV is contained in the neglected vehicle list, then does not apply the collision avoidance (as AV will never be controlled)
        negligence_pair_dict = self.get_negligence_pair_dict(veh_ctx_dicts)
        veh_ctx_dicts = self.record_negligence_related_information(
            negligence_pair_dict, veh_ctx_dicts
        )
        if len(negligence_pair_dict):
            logger.critical("Apply CAV-related collision avoidance!")
            neglected_vehicle_id_set = set()
            for neglected_vehicle_list in negligence_pair_dict.values():
                neglected_vehicle_id_set.update(neglected_vehicle_list)
            if "CAV" in neglected_vehicle_id_set:
                self.record.event_info[utils.get_time()].update(
                    {
                        "neglected_vehicle_id": "CAV",
                        "mode": "accept_collision",
                        "additional_info": "CAV_neglected",
                    }
                )
                return ITE_control_command_dict, veh_ctx_dicts, 1.0
        return super().apply_collision_avoidance(
            trajectory_dicts, veh_ctx_dicts, ITE_control_command_dict
        )

    def on_step(self, ctx):
        self.distance_info.after.update(self.update_distance())
        self.record.final_time = utils.get_time()  # update the final time at each step
        self.cache_history_tls_data()
        # clear vehicle context dicts
        veh_ctx_dicts = {}
        # Make NDE decisions for all vehicles
        control_cmds, veh_ctx_dicts = EnvTemplate.make_decisions(self, ctx)
        CAV_control_command_cache = (
            copy.deepcopy(control_cmds["CAV"]) if "CAV" in control_cmds else None
        )
        # first_vehicle_veh = list(control_cmds.keys())[0]
        # for veh_id in control_cmds:
        #     history_data = self.vehicle_list[veh_id].sensors["ego"].history_array
        obs_dicts = self.get_observation_dicts()
        # Make ITE decision, includes the modification of NDD distribution according to avoidability
        control_cmds, veh_ctx_dicts, obs_dicts, should_continue_simulation_flag = (
            self.executeMove(ctx, control_cmds, veh_ctx_dicts, obs_dicts)
        )
        if "CAV" in traci.vehicle.getIDList():
            (
                ITE_control_cmds,
                veh_ctx_dicts,
                weight,
                trajectory_dicts,
                maneuver_challenge_dicts,
                _,
            ) = self.NADE_decision(
                control_cmds, veh_ctx_dicts, obs_dicts
            )  # enable ITE
            self.importance_sampling_weight *= weight  # update weight by negligence
            ITE_control_cmds, veh_ctx_dicts, weight = self.apply_collision_avoidance(
                trajectory_dicts, veh_ctx_dicts, ITE_control_cmds
            )
            self.importance_sampling_weight *= (
                weight  # update weight by collision avoidance
            )
            ITE_control_cmds = self.update_control_cmds_from_predicted_trajectory(
                ITE_control_cmds, trajectory_dicts
            )
            if hasattr(self, "nnde_make_decisions"):
                nnde_control_commands, _ = self.nnde_make_decisions(ctx)
                ITE_control_cmds = self.merge_NADE_NeuralNDE_control_commands(
                    ITE_control_cmds, nnde_control_commands
                )
            self.refresh_control_commands_state()
            if "CAV" in ITE_control_cmds and CAV_control_command_cache is not None:
                ITE_control_cmds["CAV"] = CAV_control_command_cache
            self.execute_control_commands(ITE_control_cmds)
            self.record_step_data(veh_ctx_dicts)
        return should_continue_simulation_flag

    def update_control_cmds_from_predicted_trajectory(
        self, ITE_control_cmds, trajectory_dicts
    ):
        # only apply this function to other vehicles except CAV
        if "CAV" not in ITE_control_cmds:
            return super().update_control_cmds_from_predicted_trajectory(
                ITE_control_cmds, trajectory_dicts
            )
        cav_command_cache = ITE_control_cmds.pop("CAV")
        ITE_control_cmds = super().update_control_cmds_from_predicted_trajectory(
            ITE_control_cmds, trajectory_dicts
        )
        ITE_control_cmds["CAV"] = cav_command_cache
        return ITE_control_cmds

    def predict_future_trajectory_dicts(self, obs_dicts, veh_ctx_dicts):
        # only consider the vehicles that are around the CAV (within 50m range)

        neighbor_veh_ids_set = set(
            traci.vehicle.getContextSubscriptionResults("CAV").keys()
        )
        # add "CAV" to the neighbor_veh_ids_set
        neighbor_veh_ids_set.add("CAV")

        filtered_obs_dicts = {
            veh_id: obs_dicts[veh_id]
            for veh_id in obs_dicts
            if veh_id in neighbor_veh_ids_set
        }
        filtered_veh_ctx_dicts = {
            veh_id: veh_ctx_dicts[veh_id]
            for veh_id in veh_ctx_dicts
            if veh_id in neighbor_veh_ids_set
        }
        return super().predict_future_trajectory_dicts(
            filtered_obs_dicts, filtered_veh_ctx_dicts
        )

    def calculate_total_distance(self):
        try:
            CAV_distance = traci.vehicle.getDistance("CAV")
        except:
            CAV_distance = 0
            veh_id = "CAV"
            if veh_id not in self.distance_info.before:
                CAV_distance += self.distance_info.after[veh_id]
            else:
                CAV_distance += (
                    self.distance_info.after[veh_id] - self.distance_info.before[veh_id]
                )
        return CAV_distance

    def predict_cav_control_command(
        self, control_command_dicts, veh_ctx_dicts, obs_dicts
    ):
        original_cav_speed = obs_dicts["CAV"]["ego"]["velocity"]
        original_cav_acceleration = obs_dicts["CAV"]["ego"]["acceleration"]
        new_cav_speed = traci.vehicle.getSpeedWithoutTraCI("CAV")
        new_cav_acceleration = (
            new_cav_speed - original_cav_speed
        ) / utils.get_step_size()

        original_cav_angle = obs_dicts["CAV"]["ego"]["heading"]
        cav_lane_id = traci.vehicle.getLaneID("CAV")
        cav_lane_position = traci.vehicle.getLanePosition("CAV")
        cav_lane_angle = traci.lane.getAngle(
            laneID=cav_lane_id,
            relativePosition=max(
                cav_lane_position - 0.5 * traci.vehicle.getLength("CAV"), 0
            ),
        )
        CAV_command = None
        # use the difference between the lane change angle adn the original cav angle to predict the control command (LEFT turn or RIGHT turn)
        # the angle is defined as SUmo's angle, the north is 0, the east is 90, the south is 180, the west is 270
        # the angle is in degree
        angle_diff = (cav_lane_angle - original_cav_angle + 180) % 360 - 180

        if angle_diff > 10:
            CAV_command = NDECommand(
                command_type=Command.LEFT,
                prob=1,
                duration=1.0,
                info={"negligence_mode": "LeftFoll"},
            )
        elif angle_diff < -10:
            CAV_command = NDECommand(
                command_type=Command.RIGHT,
                prob=1,
                duration=1.0,
                info={"negligence_mode": "RightFoll"},
            )

        if original_cav_acceleration - new_cav_acceleration > 1.5:
            # predict the cav control command as negligence
            leader_info = traci.vehicle.getLeader("CAV")
            is_car_following_flag = False
            if leader_info is not None:
                is_car_following_flag = is_car_following("CAV", leader_info[0])
            CAV_command = NDECommand(
                command_type=Command.ACCELERATION,
                acceleration=original_cav_acceleration,
                prob=1,
                duration=1.0,
                info={
                    "negligence_mode": "Lead",
                    "is_car_following_flag": is_car_following_flag,
                },
            )

        if CAV_command:
            _, predicted_collision_type = get_collision_type_and_prob(
                obs_dict=obs_dicts["CAV"],
                negligence_command=CAV_command,
            )
            CAV_command.info.update(
                {"predicted_collision_type": predicted_collision_type}
            )
        return CAV_command

    def get_maneuver_challenge(
        self,
        negligence_veh_id,
        negligence_veh_future,
        all_normal_veh_future,
        obs_dicts,
        veh_ctx_dict,
        record_in_ctx=False,
        highlight_flag=True,
        buffer=0,
    ):
        if negligence_veh_id != "CAV":
            cav_future = (
                {"CAV": all_normal_veh_future["CAV"]}
                if "CAV" in all_normal_veh_future
                else None
            )
            return super().get_maneuver_challenge(
                negligence_veh_id,
                negligence_veh_future,
                cav_future,
                obs_dicts,
                veh_ctx_dict,
                record_in_ctx,
                highlight_flag,
                buffer=buffer,
            )
        else:
            return super().get_maneuver_challenge(
                negligence_veh_id,
                negligence_veh_future,
                all_normal_veh_future,
                obs_dicts,
                veh_ctx_dict,
                record_in_ctx,
                highlight_flag,
                buffer=buffer,
            )

    def should_continue_simulation(self):
        num_colliding_vehicles = self.simulator.get_colliding_vehicle_number()
        self._vehicle_in_env_distance("after")

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
        elif "CAV" not in traci.vehicle.getIDList():
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
        return True
    
    def record_step_data(self, veh_ctx_dicts):
        step_log = Dict()
        bv_criticality_list = []
        for veh_id, veh_ctx_dict in veh_ctx_dicts.items():
            maneuver_challenge = veh_ctx_dict.get("maneuver_challenge", None)
            if maneuver_challenge and maneuver_challenge.get("negligence", None):
                step_log[veh_id]["maneuver_challenge"] = maneuver_challenge

            keys = ["avoidable", "conflict_vehicle_list", "mode"]
            step_log[veh_id].update(
                {key: veh_ctx_dict[key] for key in keys if veh_ctx_dict.get(key)}
            )
            if step_log[veh_id].get("avoidable"):
                step_log[veh_id].pop(
                    "avoidable"
                )  # remove the avoidable key if it is True
            if veh_id != "CAV":
                criticality = 0.0
                if "criticality" in veh_ctx_dict and "negligence" in veh_ctx_dict["criticality"]:
                    criticality = veh_ctx_dict["criticality"]["negligence"]
                bv_criticality_list.append(criticality)
        # pop the empty dict
        step_log = {k: v for k, v in step_log.items() if v}
        step_log = {
            "weight": self.importance_sampling_weight,
            "vehicle_log": step_log,
        }
        time_step = utils.get_time()
        self.record.step_info[time_step] = step_log
        self.record.weight_step_info[time_step] = self.step_weight
        self.record.epsilon_step_info[time_step] = self.step_epsilon
        self.record.criticality_step_info[time_step] = sum(bv_criticality_list)
        self.record.drl_obs[time_step] = self.collect_drl_obs(veh_ctx_dicts).tolist()
        overall_avoidable = True
        for veh_id in veh_ctx_dicts:
            if not veh_ctx_dicts[veh_id].get("avoidable", True):
                overall_avoidable = False
        self.record.avoidable[time_step] = overall_avoidable

        return step_log

    def collect_drl_obs(self, veh_ctx_dicts):
        CAV_global_position = list(traci.vehicle.getPosition("CAV"))
        CAV_speed = traci.vehicle.getSpeed("CAV")
        CAV_heading = traci.vehicle.getAngle("CAV")
        CAV_driving_distance = traci.vehicle.getDistance("CAV")
        # position x, position y, CAV driving distance, velocity, heading
        vehicle_info_list = []
        controlled_bv_num = 1
        for veh_id, veh_ctx_dict in veh_ctx_dicts.items():
            if veh_id == "CAV":
                continue
            if "criticality" in veh_ctx_dict and "negligence" in veh_ctx_dict["criticality"]:
                criticality = veh_ctx_dict["criticality"]["negligence"]
                if criticality > 0:
                    vehicle_local_position = list(traci.vehicle.getPosition(veh_id))
                    vehicle_relative_position = [vehicle_local_position[0]-CAV_global_position[0], vehicle_local_position[1]-CAV_global_position[1]]
                    vehicle_speed = traci.vehicle.getSpeed(veh_id)
                    vehicle_heading = traci.vehicle.getAngle(veh_id)
                    vehicle_info_list.extend(vehicle_relative_position + [vehicle_speed] + [vehicle_heading])
                    break
            
        if not vehicle_info_list:
            vehicle_info_list.extend([-100, -100, 0, 0])

        velocity_lb, velocity_ub = 0, 10
        CAV_position_lb, CAV_position_ub = [0, 0], [240, 400]
        driving_distance_lb, driving_distance_ub = 0, 1000
        heading_lb, heading_ub = 0, 360
        vehicle_info_lb, vehicle_info_ub = [-20, -20, 0, 0], [20, 20, 10, 360]

        lb_array = np.array(CAV_position_lb + [velocity_lb] + [driving_distance_lb] + [heading_lb] + vehicle_info_lb)
        ub_array = np.array(CAV_position_ub + [velocity_ub] + [driving_distance_ub] + [heading_ub] + vehicle_info_ub)
        total_obs_for_DRL_ori = np.array(CAV_global_position + [CAV_speed] + [CAV_driving_distance] + [CAV_heading] + vehicle_info_list)
        
        total_obs_for_DRL = 2 * (total_obs_for_DRL_ori - lb_array)/(ub_array - lb_array) - 1
        total_obs_for_DRL = np.clip(total_obs_for_DRL, -5, 5)
        return np.array(total_obs_for_DRL).astype(float)