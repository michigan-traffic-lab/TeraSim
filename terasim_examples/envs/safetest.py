import redis
from terasim.overlay import traci

from terasim_nde_nade.envs.safetest_nade_with_av import SafeTestNADEWithAV


class SafeTestNADEWithAVCosim(SafeTestNADEWithAV):

    def __init__(
        self,
        user_step=None,
        *args,
        **kwargs,
    ):
        """Initialize the environment (Please do not change)."""
        super().__init__(*args, **kwargs)
        self.user_step = user_step

    def on_start(self, ctx):
        """Description: The start function for the environment"""

        redis_client = redis.Redis(host="localhost", port=6379, db=0)
        redis_client.flushall()

        super().on_start(ctx)
        traci.vehicle.setColor("CAV", (255, 0, 0, 255))

    def get_IS_prob(
        self,
        veh_id,
        ndd_control_command_dicts,
        maneuver_challenge_dicts,
        veh_ctx_dicts,
    ):
        return 0.05

    def on_step(self, ctx):
        """The main step function for the environment"""

        super().on_step(ctx)

        return self.user_step(traci)

    def on_stop(self, ctx):
        """Description: The stop function for the environment."""

        super.on_stop(ctx)
        print("Terasim Stopped!")
