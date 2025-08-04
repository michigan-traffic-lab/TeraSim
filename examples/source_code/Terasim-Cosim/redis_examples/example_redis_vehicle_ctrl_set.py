import time
import terasim_cosim.redis_msgs as redis_msgs

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


# Configure redis key-and data type
key_value_config = {VEHICLE_CONTROL: redis_msgs.VehicleControl}
redis_client = create_redis_client(key_value_config=key_value_config)

# For detailed fileds, see redis_msgs/VehicleControl.py
veh_control = redis_msgs.VehicleControl()

# Set the timestamp
veh_control.header.timestamp = time.time()
veh_control.header.information = "example"

# [0.0, 1.0]
veh_control.brake_cmd = 0.2

# [0.0, 1.0]
veh_control.throttle_cmd = 0.0

# [-2.5pi, 2.5pi]
veh_control.steering_cmd = 0.0

# NONE   	 =0
# PARK       =1
# REVERSE    =2
# REVERSE    =2
# NEUTRAL    =3
# DRIVE      =4
# LOW        =5
veh_control.gear_cmd = 1

redis_client.set(VEHICLE_CONTROL, veh_control)

print("Successfully set vehicle control to redis!")
