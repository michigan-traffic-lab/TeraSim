import time
import terasim_cosim.redis_msgs as redis_msgs

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


# Configure redis key-and data type
key_value_config = {FLAG: redis_msgs.GeneralMsg}
redis_client = create_redis_client(key_value_config=key_value_config)

# get the flag
flag = redis_client.get(FLAG)

print("msg timestamp: ", flag.header.timestamp)
print("msg data: ", flag.data)
