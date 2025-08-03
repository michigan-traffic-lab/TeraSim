import argparse

from pathlib import Path

from terasim.simulator import Simulator
from terasim.logger.infoextractor import InfoExtractor
from terasim_nde_nade.vehicle.nde_vehicle_factory import NDEVehicleFactory
from envs.construction import ConstructionCosim

from terasim_cosim.constants import *
from terasim_cosim_plugin import TeraSimCoSimPlugin
from terasim_user_functions import user_step

parser = argparse.ArgumentParser(description="Run simulation.")

# Optional arguments
parser.add_argument(
    "--warmup_time_lb", type=int, help="warmup time lower bound", default=300
)
parser.add_argument(
    "--warmup_time_ub", type=int, help="warmup time upper bound", default=900
)
parser.add_argument(
    "--run_time", type=int, help="simulation maximum run time", default=3600
)

args = parser.parse_args()

env = ConstructionCosim(
    user_step=user_step,
    vehicle_factory=NDEVehicleFactory(),
    info_extractor=InfoExtractor,
    warmup_time_lb=args.warmup_time_lb,
    warmup_time_ub=args.warmup_time_ub,
    run_time=args.run_time,
    closed_lane_ids=["EG_1_3_1.61_2"],
)

dir_path = Path(__file__).parent
sim = Simulator(
    sumo_net_file_path=dir_path / "maps" / "mcity.net.xml",
    sumo_config_file_path=dir_path / "maps" / "mcity.sumocfg",
    num_tries=10,
    gui_flag=True,
    realtime_flag=True,
    additional_sumo_args=["--start", "--quit-on-end"],
)

# Vehicle Co-simulation
sim.add_plugin(
    TeraSimCoSimPlugin(
        control_tls=True,  # allow outside source to synchronize av state, like CARLA
        keepRoute=1,  # Map cav to the closest edge
        closed_lane_ids=["EG_1_3_1.61_2"],
        closed_lane_pos=["right"],
        closed_lane_shapes=[]
    )
)

sim.bind_env(env)
sim.run()
