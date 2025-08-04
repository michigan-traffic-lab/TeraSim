import terasim_cosim.redis_msgs as redis_msgs

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


# Configure redis key-and data type
key_value_config = {
    CAV_INFO: redis_msgs.ActorDict,
    TERASIM_ACTOR_INFO: redis_msgs.ActorDict,
}
redis_client = create_redis_client(key_value_config=key_value_config)

# For detailed fileds, see redis_msgs/VehicleDict.py
cav_info = redis_client.get(CAV_INFO)
terasim_actor_info = redis_client.get(TERASIM_ACTOR_INFO)

if cav_info:
    print("cav vehicle msg timestamp: ", cav_info.header.timestamp)

    # There should only be cav info in the dictionary
    for veh, value in cav_info.data.items():
        print("vehicle name: ", veh)
        print("x: ", value.x)
        print("y: ", value.y)
        print("z: ", value.z)
        print("length: ", value.length)
        print("width: ", value.width)
        print("height: ", value.height)
        print("orientation: ", value.orientation)
        print("speed_long: ", value.speed_long)
else:
    print(f"{CAV_INFO} is not available")

if terasim_actor_info:
    print("terasim vehicle msg timestamp: ", terasim_actor_info.header.timestamp)
    # Iterate over the vehicles and print their information
    for veh, value in terasim_actor_info.data.items():
        print("vehicle name: ", veh)
        print("x: ", value.x)
        print("y: ", value.y)
        print("z: ", value.z)
        print("length: ", value.length)
        print("width: ", value.width)
        print("height: ", value.height)
        print("orientation: ", value.orientation)
        print("speed_long: ", value.speed_long)
else:
    print(f"{TERASIM_ACTOR_INFO} is not available")
