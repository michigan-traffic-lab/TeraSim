import time
import terasim_cosim.redis_msgs as redis_msgs

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


# Configure redis key-and data type
key_value_config = {COSIM_TLS_INFO: redis_msgs.SUMOSignalDict}
redis_client = create_redis_client(key_value_config=key_value_config)

# For detailed fileds, see redis_msgs/VehicleControl.py
cosim_tls_info = redis_msgs.SUMOSignalDict()

# Set the timestamp
cosim_tls_info.header.timestamp = time.time()
cosim_tls_info.header.information = "example"

node_11 = redis_msgs.SUMOSignal()
node_11.tls = "ryyrryyr"

node_12 = redis_msgs.SUMOSignal()
node_12.tls = "rrry"

node_17 = redis_msgs.SUMOSignal()
node_17.tls = "rGrrrGrrr"

node_18 = redis_msgs.SUMOSignal()
node_18.tls = "rryyrryy"

node_23 = redis_msgs.SUMOSignal()
node_23.tls = "rryr"

node_24 = redis_msgs.SUMOSignal()
node_24.tls = "GgrG"

cosim_tls_info.data["NODE_11"] = node_11
cosim_tls_info.data["NODE_12"] = node_12
cosim_tls_info.data["NODE_17"] = node_17
cosim_tls_info.data["NODE_18"] = node_18
cosim_tls_info.data["NODE_23"] = node_23
cosim_tls_info.data["NODE_24"] = node_24

cosim_tls_info.av_next_tls = "G"
cosim_tls_info.av_next_dist = 10.0

redis_client.set(COSIM_TLS_INFO, cosim_tls_info)

print("Successfully set traffic light to redis!")