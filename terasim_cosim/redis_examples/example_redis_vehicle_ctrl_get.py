import time
import terasim_cosim.redis_msgs as redis_msgs

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


# Configure redis key-and data type
key_value_config = {VEHICLE_CONTROL: redis_msgs.VehicleControl}
redis_client = create_redis_client(key_value_config=key_value_config)

# For detailed fileds, see redis_msgs/VehicleControl.py
veh_control = redis_client.get(VEHICLE_CONTROL)

# Print the control commands
print("msg timestamp: ", veh_control.header.timestamp)
print("brake_cmd: ", veh_control.brake_cmd)
print("throttle_cmd: ", veh_control.throttle_cmd)
print("steering_cmd: ", veh_control.steering_cmd)
print("gear_cmd: ", veh_control.gear_cmd)