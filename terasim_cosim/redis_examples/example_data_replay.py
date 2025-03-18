import time
import xml.etree.ElementTree as ET

from terasim_cosim.constants import *
from terasim_cosim.terasim_plugin.utils import *
from terasim_cosim.redis_client_wrapper import create_redis_client

from terasim_cosim.redis_msgs import (
    Actor,
    ActorDict,
    ConstructionZone,
)

key_value_config = {
    CAV_INFO: ActorDict,
    TERASIM_ACTOR_INFO: ActorDict,
    CONSTRUCTION_ZONE_INFO: ConstructionZone,
}

redis_client = create_redis_client(
    key_value_config=key_value_config,
    remote_flag=False,
    pub_channels=[],
    sub_channels=[],
    latency_src_channels=[],
)


def process_fcd_file(file_path, start_time=499.0):
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Iterate over all timestep elements
    for timestep in root.findall("timestep"):
        time_value = float(timestep.get("time"))

        # Skip timesteps before the specified start_time
        if time_value < start_time:
            continue

        print(f"\nTimestep: {time_value}s")

        cav_info = ActorDict()
        cav_info.header.timestamp = time.time()

        terasim_controlled_actor_info_with_timestamp = ActorDict()
        terasim_controlled_actor_info_with_timestamp.header.timestamp = time.time()

        # Iterate over all vehicle elements in the timestep
        for vehicle in timestep.findall("vehicle"):
            vehID = vehicle.get("id")

            x = float(vehicle.get("x")) - UTM_OFFSET[0]
            y = float(vehicle.get("y")) - UTM_OFFSET[1]
            z = float(vehicle.get("z")) - UTM_OFFSET[2]
            orientation = float(vehicle.get("angle"))

            orientation = sumo_heading_to_orientation(orientation)
            x, y = front_coordinate_to_center_coordinate(x, y, orientation)

            if "CAV" in vehID:
                cav_info.data["CAV"] = Actor(
                    x=x,
                    y=y,
                    z=z,
                    length=5.0,
                    width=1.8,
                    height=2.0,
                    orientation=orientation,
                )

            elif "BV" in vehID:
                veh_info = Actor(
                    x=x,
                    y=y,
                    z=z,
                    length=5.0,
                    width=1.8,
                    height=2.0,
                    orientation=orientation,
                )

                terasim_controlled_actor_info_with_timestamp.data[vehID] = veh_info

        redis_client.set(CAV_INFO, cav_info)

        redis_client.set(
            TERASIM_ACTOR_INFO, terasim_controlled_actor_info_with_timestamp
        )

        # Wait for 0.1 seconds to match real-time data recording
        time.sleep(0.1)


if __name__ == "__main__":
    file_path = "fcd_all.xml"  # Update with the actual file path if needed
    process_fcd_file(file_path, start_time=0.0)
