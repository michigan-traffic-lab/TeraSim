from terasim_nde_nade.envs.safetest_nade_with_av import SafeTestNADEWithAV


class SafeTestNADEWithAVCosim(SafeTestNADEWithAV):

    def __init__(
        self,
        *args,
        **kwargs,
    ):
        """Initialize the environment (Please do not change)."""
        super().__init__(*args, **kwargs)

    def get_IS_prob(
        self,
        veh_id,
        ndd_control_command_dicts,
        maneuver_challenge_dicts,
        veh_ctx_dicts,
    ):
        return 0.0
