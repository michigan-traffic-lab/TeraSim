from pathlib import Path

from terasim.simulator import Simulator
from terasim.logger.infoextractor import InfoExtractor

from terasim_cosim.constants import *
from terasim_cosim_plugin import TeraSimCoSimPlugin
from terasim_user_functions import user_step

from envs.joint_control import (
    TeraSimEnvForUser,
    ExampleVehicleFactory,
)

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
    sumo_config_file_path=maps_path / "mcity_bike_example.sumocfg",
    num_tries=10,
    gui_flag=True,
    realtime_flag=True,
    additional_sumo_args=["--start", "--quit-on-end"],
)

# Vehicle Co-simulation
sim.add_plugin(
    TeraSimCoSimPlugin(
        control_cav=False,  # allow outside source to synchronize av state, like CARLA
        control_tls=True,  # TeraSim controlled traffic lights
        keepRoute=1,  # Map cav to the closest edge
    )
)
 
sim.bind_env(env)
sim.run()
