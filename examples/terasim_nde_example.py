import os
import argparse

from pathlib import Path
from terasim_cosim.constants import *
from terasim.simulator import Simulator
from terasim.logger.infoextractor import InfoExtractor
from terasim_nde_nade.vehicle.nde_vehicle_factory import NDEVehicleFactory
from envs.safetest_nade_with_av_cosim import SafeTestNADEWithAVCosim

from terasim_cosim.terasim_plugin.terasim_tls_plugin import TeraSimTLSPlugin
from terasim_cosim.terasim_plugin.terasim_cosim_plugin import TeraSimCoSimPlugin

from terasim_user_functions import user_step

os.environ[ENVIRONMENT_VARIABLE_HOST] = "0.0.0.0"
os.environ[ENVIRONMENT_VARIABLE_PORT] = "2000"
os.environ[ENVIRONMENT_VARIABLE_PASSWORD] = "password"

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

env = SafeTestNADEWithAVCosim(
    user_step=user_step,
    vehicle_factory=NDEVehicleFactory(),
    info_extractor=InfoExtractor,
    warmup_time_lb=args.warmup_time_lb,
    warmup_time_ub=args.warmup_time_ub,
    run_time=args.run_time,
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
        remote_flag=False,  # connect to mcityos, disable for local testing
        control_cav=False,  # allow outside source to synchronize av state, like CARLA
        keepRoute=2,  # Traci keep route defition, 2 being the most flexible
        CAVSpeedOverride=True,  # Allow speed override for CAV in Terasim
        pub_channels=[],  # Publish channels (mcityos remote only)
        sub_channels=[],  # Subscribe channels (mcityos remote only)
        latency_src_channels=[],  # Latency source channels (mcityos remote only)
    )
)

# Traffic Light Co-simulation
sim.add_plugin(
    TeraSimTLSPlugin(
        remote_flag=False,  # connect to mcityos, disable for local testing
        control_tls=True,  # TeraSim controlled traffic lights
        pub_channels=[],  # Publish channels (mcityos remote only)
        sub_channels=[],  # Subscribe channels (mcityos remote only)
        latency_src_channels=[],  # Latency source channels (mcityos remote only)
    )
)

sim.bind_env(env)
sim.run()
