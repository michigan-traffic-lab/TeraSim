import terasim_cosim.redis_msgs as redis_msgs

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


# Configure redis key-and data type
key_value_config = {COSIM_TLS_INFO: redis_msgs.SUMOSignalDict}
redis_client = create_redis_client(key_value_config=key_value_config)

# For detailed fileds, see redis_msgs/VehicleDict.py
cosim_tls_info = redis_client.get(COSIM_TLS_INFO)

# Print the timestamp
print("msg timestamp:", cosim_tls_info.header.timestamp)
print("msg information:", cosim_tls_info.header.information)

data = cosim_tls_info.data

for node, status in data.items():
    print(f"{node}: {status.tls}")

print("av_next_tls:", cosim_tls_info.av_next_tls)
print("av_next_dist:", cosim_tls_info.av_next_dist)
