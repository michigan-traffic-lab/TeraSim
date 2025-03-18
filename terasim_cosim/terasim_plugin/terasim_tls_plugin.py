import time

from terasim.overlay import traci
from terasim.simulator import Simulator

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client
from terasim_cosim.redis_msgs import SUMOSignal, SUMOSignalDict


class TeraSimTLSPlugin:

    def __init__(
        self,
        remote_flag=False,
        control_tls=True,
        pub_channels=[],
        sub_channels=[],
        latency_src_channels=[],
    ):
        self.remote_flag = remote_flag
        self.control_tls = control_tls

        self.pub_channels = pub_channels
        self.sub_channels = sub_channels
        self.latency_src_channels = latency_src_channels

    def on_start(self, simulator: Simulator, ctx):
        key_value_config = {
            TLS_INFO: SUMOSignalDict,
        }

        self.redis_client = create_redis_client(
            key_value_config=key_value_config,
            remote_flag=self.remote_flag,
            pub_channels=self.pub_channels,
            sub_channels=self.sub_channels,
            latency_src_channels=self.latency_src_channels,
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
        sumo_tls = self.get_all_tls_info()

        nextTLS = traci.vehicle.getNextTLS("CAV")
        if nextTLS:
            tlsID, tlsIndex, distance, state = nextTLS[0]
        else:
            state, distance = "", 0.0

        tls_info = SUMOSignalDict()
        tls_info.header.timestamp = time.time()
        tls_info.header.information = "TeraSim"
        tls_info.av_next_tls = state
        tls_info.av_next_dist = distance

        for tls_id in sumo_tls:
            signal = SUMOSignal()
            signal.tls = sumo_tls[tls_id]
            tls_info.data[tls_id] = signal

        self.redis_client.set(TLS_INFO, tls_info)

    def sync_cosim_tls_to_terasim(self):
        cosim_controlled_tls_info = self.redis_client.get(TLS_INFO)
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
