import time
import terasim_cosim.redis_msgs as redis_msgs

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


# Configure redis key-and data type
key_value_config = {PLANNED_PATH: redis_msgs.PlannedPath}
redis_client = create_redis_client(key_value_config=key_value_config)

# For detailed fileds, see redis_msgs/PlannedPath.py
planned_path = redis_msgs.PlannedPath()

# Set the timestamp
planned_path.header.timestamp = time.time()

# Set the time resolution (seconds)
planned_path.time_resolution = 0.1

# Set the planned path of the next at lease 2 seconds with the predefined time resolution
## List of x coordinate of the vehicle's center in UTM coordinate (meters)
planned_path.x_list = [0.1] * 20
## List of y coordinate of the vehicle's center in UTM coordinate (meters)
planned_path.y_list = [0.1] * 20
## List of desired speed of the vehicle (meters per second)
planned_path.speed_list = [0.1] * 20
## List of desired orientation of the vehicle ranging from -pi to pi, where 0 means the vehicle is heading to the east, pi/2 means the vehicle is heading to the north (radians)
planned_path.orientation_list = [0.1] * 20

# Set the data to redis
redis_client.set(PLANNED_PATH, planned_path)

print("Successfully set planned path to redis!")
