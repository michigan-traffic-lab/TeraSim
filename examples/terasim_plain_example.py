import argparse

from pathlib import Path

from terasim.simulator import Simulator
from terasim.logger.infoextractor import InfoExtractor

from terasim_cosim.terasim_plugin.terasim_cosim_plugin import TeraSimCosimPlugin

from envs.env_mcity_joint_control import (
    TeraSimEnvForUser,
    ExampleVehicleFactory,
)

parser = argparse.ArgumentParser(description="Run simulation.")

# Optional arguments
parser.add_argument("--gui_flag", action="store_true", help="display sumo gui")
parser.add_argument(
    "--realtime_flag", action="store_true", help="run simulation in realtime"
)
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

if __name__ == "__main__":
    current_path = Path(__file__).parent
    maps_path = current_path / "maps"

    # Create the environment
    env = TeraSimEnvForUser(
        vehicle_factory=ExampleVehicleFactory(),
        info_extractor=InfoExtractor,
        user_step=user_step,
    )

    # Create the simulator
    sim = Simulator(
        sumo_net_file_path=maps_path / "mcity.net.xml",
        sumo_config_file_path=maps_path / "mcity_micro.sumocfg",
        num_tries=10,
        gui_flag=True,
        realtime_flag=True,
    )

    sim.add_plugin(TeraSimCosimPlugin())
    sim.bind_env(env)
    sim.run()