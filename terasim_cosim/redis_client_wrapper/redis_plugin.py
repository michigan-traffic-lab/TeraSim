from terasim_cosim.redis_client_wrapper import create_redis_client


class RedisBasicPlugin:

    def __init__(self, key_value_config):
        self.redis_client = create_redis_client(key_value_config=key_value_config)

    def on_start(self, simulator, ctx):
        pass

    def on_step(self, simulator, ctx):
        raise NotImplementedError

    def on_stop(self, simulator, ctx):
        pass

    def inject(self, simulator, ctx):
        self.ctx = ctx
        self.simulator = simulator

        simulator.start_pipeline.hook("cosim_start", self.on_start, priority=-100)
        simulator.step_pipeline.hook("cosim_step", self.on_step, priority=-100)
        simulator.stop_pipeline.hook("cosim_stop", self.on_stop, priority=-100)
