import time
import terasim_cosim.redis_msgs as redis_msgs

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


# Configure redis key-and data type
key_value_config = {VEHICLE_STATE: redis_msgs.VehicleState}
redis_client = create_redis_client(key_value_config=key_value_config)

# For detailed fileds, see redis_msgs/VehicleState.py
veh_state = redis_msgs.VehicleState()

# Set the timestamp
veh_state.header.timestamp = time.time()
veh_state.header.information = "example"

# Set the vehicle speed
veh_state.speed_x = 0.0
veh_state.speed_y = 0.0
veh_state.speed_z = 0.0

# Set the drive by wire status
veh_state.by_wire_enabled = False

# Set the gear position
veh_state.gear_pos = 0

# Set the engine state
veh_state.engine_rpm = 0.0

# Set the throttle state
veh_state.throttle_cmd = 0.0
veh_state.throttle_input = 0.0
veh_state.throttle_state = 0.0
veh_state.throttle_enabled = False
veh_state.throttle_override = False
veh_state.throttle_driver = False
veh_state.throttle_timeout = False

# Set the brake state
veh_state.brake_cmd = 0.0
veh_state.brake_input = 0.0
veh_state.brake_state = 0.0
veh_state.brake_torq_cmd = 0.0
veh_state.brake_torq_input = 0.0
veh_state.brake_torq_state = 0.0
veh_state.brake_boo_output = False
veh_state.brake_enabled = False
veh_state.brake_override = False
veh_state.brake_driver = False
veh_state.brake_timeout = False

# Set the steering state
veh_state.steer_cmd = 0.0
veh_state.steer_state = 0.0
veh_state.steer_torque = 0.0
veh_state.steer_enabled = False
veh_state.steer_override = False
veh_state.steer_timeout = False

# Set the wheel state
veh_state.wheel_v_front_left = 0.0
veh_state.wheel_v_front_right = 0.0
veh_state.wheel_v_rear_left = 0.0
veh_state.wheel_v_rear_right = 0.0

# Set the vehicle state to redis
redis_client.set(VEHICLE_STATE, veh_state)

print("Successfully set vehicle state to redis!")
