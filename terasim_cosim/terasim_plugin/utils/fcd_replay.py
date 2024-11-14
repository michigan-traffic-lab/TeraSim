import math
import time
import argparse
import xml.etree.ElementTree as ET

import terasim_cosim.redis_msgs as redis_msgs
from terasim_cosim.constants import *
from terasim_cosim.terasim_plugin.utils.convertion import *
from terasim_cosim.redis_client_wrapper import create_redis_client

from fcd_plot import DynamicPlot


def parse_vehicle_data(xml_file, start_time=0.1, stop_time=None):
    """
    Parses vehicle data from an XML file and returns a dictionary containing vehicle data for each timestep.

    Parameters:
        xml_file (str): Path to the XML file.
        start_time (float): The start time from which to begin parsing data.
        stop_time (float): The end time at which to stop parsing data.

    Returns:
        dict: A dictionary where keys are timesteps and values are dictionaries of vehicle data.
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()

    data_by_timestep = {}

    for timestep in root.findall("timestep"):
        current_time = float(timestep.get("time"))

        # Apply start and stop time filters
        if current_time < start_time:
            continue
        if stop_time is not None and current_time > stop_time:
            break  # Assuming timesteps are ordered, we can break early

        vehicles = {}

        # Extract vehicle data within the current timestep
        for vehicle in timestep.findall("vehicle"):
            vehicle_id = vehicle.get("id")
            vehicle_attributes = vehicle.attrib
            vehicles[vehicle_id] = vehicle_attributes

        data_by_timestep[current_time] = vehicles

    return data_by_timestep


def setup_redis_client():
    """
    Configures and returns a Redis client with predefined key-value mappings.

    Returns:
        RedisClient: Configured Redis client.
    """
    key_value_config = {
        TERASIM_COSIM_VEHICLE_INFO: redis_msgs.VehicleDict,
        CAV_COSIM_VEHICLE_INFO: redis_msgs.VehicleDict,
    }
    return create_redis_client(key_value_config=key_value_config)


def process_and_replay_data(vehicle_data, bv_id, start_timestep):
    """
    Processes vehicle data and stores it in Redis.

    Parameters:
        vehicle_data (dict): Vehicle data organized by timesteps.
        redis_client (RedisClient): Redis client for storing data.
    """

    # Setup Redis client
    redis_client = setup_redis_client()

    cav_cosim_vehicle_info = redis_msgs.VehicleDict()
    terasim_cosim_vehicle_info = redis_msgs.VehicleDict()

    plot = DynamicPlot()
    plot.add_data_point(
        timestamp=0,
        cav_speed=None,
        bv_speed=None,
        distance=None,
    )
    time.sleep(2.0)

    cav_speed_window = []

    prev_time = time.time()

    for timestep, vehicles in vehicle_data.items():
        while time.time() - prev_time < 0.1:
            pass

        prev_time = time.time()

        # Set the timestamp for both VehicleDicts
        cav_cosim_vehicle_info.header.timestamp = timestep
        terasim_cosim_vehicle_info.header.timestamp = timestep

        for vehicle_id, attributes in vehicles.items():
            veh = redis_msgs.Vehicle()
            veh.x = float(attributes["x"]) - UTM_OFFSET[0]
            veh.y = float(attributes["y"]) - UTM_OFFSET[1]
            veh.z = float(attributes["z"]) - UTM_OFFSET[2]
            veh.length = 5.0
            veh.width = 1.8
            veh.height = 1.5
            veh.orientation = sumo_heading_to_orientation(float(attributes["angle"]))
            veh.speed_long = float(attributes["speed"])

            # Assign vehicle to the appropriate VehicleDict
            if vehicle_id == "CAV":
                cav_cosim_vehicle_info.data[vehicle_id] = veh
            else:
                terasim_cosim_vehicle_info.data[vehicle_id] = veh

        # Store the updated VehicleDicts in Redis
        if cav_cosim_vehicle_info.data:
            # only update all vehicles when cav updates to prevent vehicles out of sync
            cav = cav_cosim_vehicle_info.data["CAV"]

            print("time", timestep, "speed", cav.speed_long)

            redis_client.set(CAV_COSIM_VEHICLE_INFO, cav_cosim_vehicle_info)
            redis_client.set(TERASIM_COSIM_VEHICLE_INFO, terasim_cosim_vehicle_info)

            if bv_id in terasim_cosim_vehicle_info.data:
                bv = terasim_cosim_vehicle_info.data[bv_id]
                bv_speed = max(0.1, bv.speed_long)
                distance = get_min_dist(
                    cav.x, cav.y, cav.orientation, bv.x, bv.y, bv.orientation
                )
                distance = min(24.5, distance - 2.6)
            else:
                bv_speed = None
                distance = None

            if cav_speed_window == []:
                cav_speed_window = [cav.speed_long] * 1
            else:
                cav_speed_window.pop(0)
                cav_speed_window.append(cav.speed_long)

            plot.add_data_point(
                timestamp=timestep - start_timestep,
                cav_speed=sum(cav_speed_window) / len(cav_speed_window),
                bv_speed=bv_speed,
                distance=distance,
            )


def parse_arguments():
    """
    Parses command-line arguments.

    Returns:
        argparse.Namespace: Parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="Process vehicle data from an XML file and store it in Redis."
    )

    parser.add_argument(
        "--xml_path",
        type=str,
        default="/home/zhijie/Figure/videos/2119736039_1724138119/fcd_all.xml",
        help="Path to the XML file containing vehicle data.",
    )
    parser.add_argument(
        "--bv_id",
        type=str,
        default="BV_17.88",
        help="The id of BV that collides with CAV",
    )
    parser.add_argument(
        "--start_time",
        type=float,
        default=1144.0,
        help="Start time to begin parsing data (default: 0.1).",
    )
    parser.add_argument(
        "--stop_time",
        type=float,
        default=1164.0,
        help="Stop time to end parsing data (default: None, which means until the last timestep).",
    )

    return parser.parse_args()


def main():
    # Parse command-line arguments
    args = parse_arguments()
    xml_file_path = args.xml_path
    start_timestep = args.start_time
    stop_timestep = args.stop_time
    bv_id = args.bv_id

    # Log configuration
    print(f"XML File Path: {xml_file_path}")
    print(f"Start Time: {start_timestep}")
    print(
        f"Stop Time: {stop_timestep if stop_timestep is not None else 'Until last timestep'}"
    )

    # Parse vehicle data from XML
    vehicle_data = parse_vehicle_data(xml_file_path, start_timestep, stop_timestep)
    print("FCD data reading complete!")

    # Process and store data in Redis
    process_and_replay_data(vehicle_data, bv_id, start_timestep)
    while 1:
        continue


if __name__ == "__main__":
    main()
