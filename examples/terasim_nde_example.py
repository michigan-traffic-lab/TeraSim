import argparse

from pathlib import Path
from terasim.simulator import Simulator
from terasim.logger.infoextractor import InfoExtractor
from terasim_nde_nade.vehicle.nde_vehicle_factory import NDEVehicleFactory
from envs.safetest_nade_with_av_cosim import SafeTestNADEWithAVCosim

from terasim_cosim.terasim_plugin.terasim_cosim_plugin import TeraSimCosimPlugin

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

sim.add_plugin(TeraSimCosimPlugin())
sim.bind_env(env)
sim.run()
