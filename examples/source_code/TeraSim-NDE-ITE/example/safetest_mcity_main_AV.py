from loguru import logger
from terasim.simulator import Simulator
from terasim_nde_nade.envs.safetest_nade_with_av import SafeTestNADEWithAV
from terasim.logger.infoextractor import InfoExtractor
from terasim_nde_nade.vehicle.nde_vehicle_factory import NDEVehicleFactory
from pathlib import Path
import argparse

parser = argparse.ArgumentParser(description="Run simulation.")
parser.add_argument("--dir", type=str, help="output directory", default="output_cav")
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
    level="TRACE",
    enqueue=True,
    backtrace=True,
)
logger.add(
    aggregated_log_dir,
    level="INFO",
    enqueue=True,
    backtrace=True,
    serialize=True,
)

env = SafeTestNADEWithAV(
    vehicle_factory=NDEVehicleFactory(),
    info_extractor=InfoExtractor,
    log_flag=True,
    log_dir=base_dir,
    warmup_time_lb=900,
    warmup_time_ub=1200,
    run_time=1200,
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

try:
    sim.run()
except Exception as e:
    terasim_logger.exception(
        f"terasim_nde_nade: Running error catched, {e} at {args.nth} experiment"
    )
