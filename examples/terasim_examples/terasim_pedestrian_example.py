import os
from pathlib import Path

from terasim.simulator import Simulator
from terasim.logger.infoextractor import InfoExtractor

from terasim_user_functions import user_step

from terasim_cosim.constants import *
from terasim_cosim.terasim_plugin.terasim_tls_plugin import TeraSimTLSPlugin
from terasim_cosim.terasim_plugin.terasim_cosim_plugin import TeraSimCoSimPlugin

from envs.env_mcity_joint_control import (
    TeraSimEnvForUser,
    ExampleVehicleFactory,
)

os.environ[ENVIRONMENT_VARIABLE_HOST] = "0.0.0.0"
os.environ[ENVIRONMENT_VARIABLE_PORT] = "2000"
os.environ[ENVIRONMENT_VARIABLE_PASSWORD] = "password"

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
    sumo_config_file_path=maps_path / "mcity_pedestrian_example.sumocfg",
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
