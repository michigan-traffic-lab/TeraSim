import os
import time

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client

os.environ[ENVIRONMENT_VARIABLE_HOST] = "3.133.16.232"
os.environ[ENVIRONMENT_VARIABLE_PORT] = "6390"
os.environ[ENVIRONMENT_VARIABLE_PASSWORD] = "1G7R1SZDteJZmFa"

redis_client = create_redis_client(
    key_value_config={},
    remote_flag=True,
    pub_channels=[TERASIM_COSIM_VEHICLE_INFO],
    sub_channels=[],
    latency_src_channel=[],
)

print("Redis client is created.")
print("Publishing and subscribing to the channels...")

while True:
    time.sleep(1)
