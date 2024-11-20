import time
import terasim_cosim.redis_msgs as redis_msgs

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


# Configure redis key-and data type
key_value_config = {FLAG: redis_msgs.GeneralMsg}
redis_client = create_redis_client(key_value_config=key_value_config)

# For detailed fileds, see redis_msgs/GeneralMsg.py
flag = redis_msgs.GeneralMsg()

# Set the timestamp
flag.header.timestamp = time.time()

# Set the flag
flag.data = "True"

redis_client.set(FLAG, flag)

print("Successfully set flag to redis!")
