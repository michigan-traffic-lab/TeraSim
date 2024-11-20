import time
import terasim_cosim.redis_msgs as redis_msgs

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


# Configure redis key-and data type
key_value_config = {PLANNED_PATH: redis_msgs.PlannedPath}
redis_client = create_redis_client(key_value_config=key_value_config)

# For detailed fileds, see redis_msgs/VehicleDict.py
planned_path = redis_client.get(PLANNED_PATH)

# Print the timestamp
print("msg timestamp: ", planned_path.header.timestamp)

# Print the time resolution (seconds)
print("time_resolution: ", planned_path.time_resolution)

# Print the planned path of the next at lease 2 seconds with the predefined time resolution
## List of x coordinate of the vehicle's center in UTM coordinate (meters)
print("x_list: ", planned_path.x_list)
## List of y coordinate of the vehicle's center in UTM coordinate (meters)
print("y_list: ", planned_path.y_list)
## List of desired speed of the vehicle (meters per second)
print("speed_list: ", planned_path.speed_list)
## List of desired orientation of the vehicle ranging from -pi to pi, where 0 means the vehicle is heading to the east, pi/2 means the vehicle is heading to the north (radians)
print("orientation_list: ", planned_path.orientation_list)
