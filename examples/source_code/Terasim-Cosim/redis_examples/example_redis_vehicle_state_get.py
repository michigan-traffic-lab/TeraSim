import terasim_cosim.redis_msgs as redis_msgs

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


# Configure redis key-and data type
key_value_config = {VEHICLE_STATE: redis_msgs.VehicleState}
redis_client = create_redis_client(key_value_config=key_value_config)

# For detailed fileds, see redis_msgs/VehicleState.py
veh_state = redis_client.get(VEHICLE_STATE)

print("msg timestamp: ", veh_state.header.timestamp)
print("msg information: ", veh_state.header.information)

print("speed_x: ", veh_state.speed_x)
print("speed_y: ", veh_state.speed_y)
print("speed_z: ", veh_state.speed_z)

print("by_wire_enabled: ", veh_state.by_wire_enabled)

print("gear_pos: ", veh_state.gear_pos)

print("engine_rpm: ", veh_state.engine_rpm)

print("throttle_cmd: ", veh_state.throttle_cmd)
print("throttle_input: ", veh_state.throttle_input)
print("throttle_state: ", veh_state.throttle_state)
print("throttle_enabled: ", veh_state.throttle_enabled)
print("throttle_override: ", veh_state.throttle_override)
print("throttle_driver: ", veh_state.throttle_driver)
print("throttle_timeout: ", veh_state.throttle_timeout)

print("brake_cmd: ", veh_state.brake_cmd)
print("brake_input: ", veh_state.brake_input)
print("brake_state: ", veh_state.brake_state)
print("brake_torq_cmd: ", veh_state.brake_torq_cmd)
print("brake_torq_input: ", veh_state.brake_torq_input)
print("brake_torq_state: ", veh_state.brake_torq_state)
print("brake_boo_output: ", veh_state.brake_boo_output)
print("brake_enabled: ", veh_state.brake_enabled)
print("brake_override: ", veh_state.brake_override)
print("brake_driver: ", veh_state.brake_driver)
print("brake_timeout: ", veh_state.brake_timeout)

print("steer_cmd: ", veh_state.steer_cmd)
print("steer_state: ", veh_state.steer_state)
print("steer_torque: ", veh_state.steer_torque)
print("steer_enabled: ", veh_state.steer_enabled)
print("steer_override: ", veh_state.steer_override)

print("wheel_v_front_left: ", veh_state.wheel_v_front_left)
print("wheel_v_front_right: ", veh_state.wheel_v_front_right)
print("wheel_v_rear_left: ", veh_state.wheel_v_rear_left)
print("wheel_v_rear_right: ", veh_state.wheel_v_rear_right)
