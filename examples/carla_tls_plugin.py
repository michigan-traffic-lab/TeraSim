import carla
from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client
from terasim_cosim.redis_msgs import SUMOSignal, SUMOSignalDict
class CarlaTLSPlugin:
    def __init__(self):
        self.mapping_dict = {
            "NODE_23": [110, None, 112, 107],
            "NODE_24": [47, None, 48, 52],
        }

    def on_step(self):
        try:
            cosim_tls_info = self.redis_client.get(COSIM_TLS_INFO)
        except:
            print("cosim_tls_info not available. Exiting...")
            return
        
        if cosim_tls_info:
            tls_info = cosim_tls_info.data

            for node_id in tls_info:
                traffic_light_status = tls_info[node_id]
                traffic_light_ids = self.mapping_dict[node_id]
                for i in range(traffic_light_ids):
                    traffic_light_actor = carla.get_actor_from_id(traffic_light_ids[i])
                    traffic_light_actor.set_state(traffic_light_status[i])