import time

from terasim.overlay import traci
from terasim.simulator import Simulator

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client
from terasim_cosim.redis_msgs import SUMOSignal, SUMOSignalDict


class TeraSimTLSPlugin:

    def __init__(self, remote_flag=False, control_tls=True):
        self.remote_flag = remote_flag
        self.control_tls = control_tls

        assert self.remote_flag == False, "Please disable remote_flag for testing"

    def on_start(self, simulator: Simulator, ctx):
        key_value_config = {
            COSIM_TLS_INFO: SUMOSignalDict,
        }

        if self.control_tls:
            self.redis_client = create_redis_client(
                key_value_config=key_value_config,
                remote_flag=self.remote_flag,
                pub_channels=[COSIM_TLS_INFO],
                sub_channels=[],
                latency_src_channels=[],
            )
        else:
            self.redis_client = create_redis_client(
                key_value_config=key_value_config,
                remote_flag=self.remote_flag,
                pub_channels=[],
                sub_channels=[COSIM_TLS_INFO],
                latency_src_channels=[],
            )

    def on_step(self, simulator: Simulator, ctx):
        if self.control_tls:
            self.sync_terasim_tls_to_cosim()
        else:
            self.sync_cosim_tls_to_terasim()

    def on_stop(self, simulator: Simulator, ctx):
        return

    def inject(self, simulator: Simulator, ctx):
        self.ctx = ctx
        self.simulator = simulator

        simulator.start_pipeline.hook("cosim_start", self.on_start, priority=-100)
        simulator.step_pipeline.hook("cosim_step", self.on_step, priority=-100)
        simulator.stop_pipeline.hook("cosim_stop", self.on_stop, priority=-100)

    def sync_terasim_tls_to_cosim(self):
        tls_info = self.get_all_tls_info()

        nextTLS = traci.vehicle.getNextTLS("CAV")
        if nextTLS:
            tlsID, tlsIndex, distance, state = nextTLS[0]
        else:
            state, distance = "", 0.0

        cosim_tls_info = SUMOSignalDict()
        cosim_tls_info.header.timestamp = time.time()
        cosim_tls_info.header.information = "TeraSim"
        cosim_tls_info.av_next_tls = state
        cosim_tls_info.av_next_dist = distance

        for tls_id in tls_info:
            signal = SUMOSignal()
            signal.tls = tls_info[tls_id]
            cosim_tls_info.data[tls_id] = signal

        self.redis_client.set(COSIM_TLS_INFO, cosim_tls_info)

    def sync_cosim_tls_to_terasim(self):
        cosim_controlled_tls_info = self.redis_client.get(COSIM_TLS_INFO)
        if cosim_controlled_tls_info:
            data = cosim_controlled_tls_info.data
            for signal_id in data:
                traci.trafficlight.setRedYellowGreenState(
                    signal_id, data[signal_id].tls
                )

    def get_tls_id_list(self):
        return traci.trafficlight.getIDList()

    def get_all_tls_info(self):
        self.tls_id_list = self.get_tls_id_list()

        tls_info = {}
        for tls_id in self.tls_id_list:
            tls_info[tls_id] = traci.trafficlight.getRedYellowGreenState(tls_id)
        return tls_info
