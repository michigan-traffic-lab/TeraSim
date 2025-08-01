from pathlib import Path
import argparse
from loguru import logger
from terasim.simulator import Simulator
from terasim_nde_nade.envs.safetest_nade import SafeTestNADE
from terasim.logger.infoextractor import InfoExtractor
from terasim_nde_nade.vehicle.nde_vehicle_factory import NDEVehicleFactory
from terasim.overlay import traci
from terasim_nde_nade.vehicle.nde_vehicle_utils import get_future_position_on_route
import sumolib

parser = argparse.ArgumentParser(description="Run simulation.")
parser.add_argument("--dir", type=str, help="output directory", default="output")
parser.add_argument("--name", type=str, help="experiment name", default="test")
parser.add_argument("--nth", type=str, help="the nth epoch", default="0_0")
parser.add_argument(
    "--aggregateddir", type=str, help="aggregated directory", default="aggregated"
)
args = parser.parse_args()

base_dir = Path(args.dir) / args.name / "raw_data" / args.nth
base_dir.mkdir(parents=True, exist_ok=True)

Path(args.aggregateddir).mkdir(parents=True, exist_ok=True)
aggregated_log_dir = Path(args.aggregateddir) / "loguru_run.log"

logger.add(
    base_dir / "loguru_run.log",
    # filter="terasim_nde_nade",
    level="TRACE",
)

logger.add(
    aggregated_log_dir,
    # filter="terasim_nde_nade",
    level="INFO",
)

env = SafeTestNADE(
    vehicle_factory=NDEVehicleFactory(),
    info_extractor=InfoExtractor,
    log_flag=True,
    log_dir=base_dir,
    warmup_time_lb=50,
    warmup_time_ub=100,
    run_time=60,
)

dir_path = Path(__file__).parent
sim = Simulator(
    sumo_net_file_path=dir_path / "maps" / "Mcity_safetest" / "mcity.net.xml",
    sumo_config_file_path=dir_path / "maps" / "Mcity_safetest" / "mcity.sumocfg",
    num_tries=10,
    gui_flag=False,
    output_path=base_dir,
    sumo_output_file_types=["fcd_all", "collision", "tripinfo"],
)
sim.bind_env(env)

terasim_logger = logger.bind(name="terasim_nde_nade")
terasim_logger.info(f"terasim_nde_nade: Experiment started {args.nth}")

sumo_net = sim.sumo_net
veh_id = "BV_20.36"
sim.start()
traci.vehicle.add(veh_id, "r_20", typeID="NDE_URBAN")
traci.vehicle.moveToXY(veh_id, "", 0, 13.557, 149.657)
route = traci.vehicle.getRoute(veh_id)
route_with_internal = sumolib.route.addInternal(sumo_net, route)
route_id_list = [route._id for route in route_with_internal]
route_length_list = [traci.lane.getLength(edge_id + "_0") for edge_id in route_id_list]

while True:
    traci.simulationStep()
    result = get_future_position_on_route(
        traci,
        ':NODE_24_5',
        0.3135676329972803,
        0,
        ':NODE_24_5_0',
        ['EG_22_1_1', ':NODE_21_0', 'EG_21_1_14', ':NODE_31_1', 'EG_21_1_18', ':NODE_22_3', 'EG_21_1_1', ':NODE_19_2', 'EG_19_1_10', ':NODE_25_0', 'EG_25_2_7', ':NODE_24_5', 'EG_24_2_1'],
        [36.64, 11.58, 10.79, 14.72, 8.95, 20.03, 8.04, 16.9, 18.46, 10.32, 34.72, 21.04, 34.08],
        2.15334971 ,
        0,
    )

    if sim.running is False:
        break
sim.stop()
