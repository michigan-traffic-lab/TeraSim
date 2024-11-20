import time
import terasim_cosim.redis_msgs as redis_msgs

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


# Configure redis key-and data type
key_value_config = {
    CAV_COSIM_VEHICLE_INFO: redis_msgs.VehicleDict,
    TERASIM_COSIM_VEHICLE_INFO: redis_msgs.VehicleDict,
}

redis_client = create_redis_client(key_value_config=key_value_config)

# For detailed fileds, see redis_msgs/VehicleDict.py
cav_cosim_vehicle_info = redis_msgs.VehicleDict()
terasim_cosim_vehicle_info = redis_msgs.VehicleDict()

# Set the timestamp
cav_cosim_vehicle_info.header.timestamp = time.time()
terasim_cosim_vehicle_info.header.timestamp = time.time()

cav = redis_msgs.Vehicle()
cav.x = 0.0
cav.y = 0.0
cav.z = 0.0
cav.length = 5.0
cav.width = 1.8
cav.height = 1.5
cav.orientation = 0.0
cav.speed_long = 0.0

# Add cav to cav_cosim_vehicle_info
cav_cosim_vehicle_info.data["CAV"] = cav

redis_client.set(CAV_COSIM_VEHICLE_INFO, cav_cosim_vehicle_info)

print(f"Successfully set {CAV_COSIM_VEHICLE_INFO} list to redis!")

# For detailed fileds, see redis_msgs/Vehicle.py
bv_0 = redis_msgs.Vehicle()
bv_0.x = 0.0
bv_0.y = 0.0
bv_0.z = 0.0
bv_0.length = 5.0
bv_0.width = 1.8
bv_0.height = 1.5
bv_0.orientation = 0.0
bv_0.speed_long = 0.0

# Add bv to terasim_cosim_vehicle_info. You can add as many vehicles as you want.
terasim_cosim_vehicle_info.data["BV_0"] = bv_0

redis_client.set(TERASIM_COSIM_VEHICLE_INFO, terasim_cosim_vehicle_info)

print(f"Successfully set {TERASIM_COSIM_VEHICLE_INFO} list to redis!")
