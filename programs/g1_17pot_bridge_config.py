# 17-channel bridge config TEMPLATE for the 20-float MCP3008 firmware variant.
# Packet indices 17-19 are spare and intentionally unbound.
#
# REQUIRED: Replace every joint_index=-1 below with the real Unitree G1 motor
# index for that channel, then set _PLACEHOLDER_JOINTS_REPLACED = True.

from g1_bridge_lib_20ch import BridgeConfig, JointBinding

# Set to True after you have replaced all placeholder joint indices below.
_PLACEHOLDER_JOINTS_REPLACED = False

if not _PLACEHOLDER_JOINTS_REPLACED:
    raise RuntimeError(
        "g1_17pot_bridge_config.py: placeholder joint indices have not been replaced. "
        "Edit joint_index for each binding and set _PLACEHOLDER_JOINTS_REPLACED = True."
    )

PLACEHOLDER_KP = 20.0
PLACEHOLDER_KD = 1.0
PLACEHOLDER_SCALE = 1.0
PLACEHOLDER_OFFSET = 0.0
PLACEHOLDER_MIN_Q = None
PLACEHOLDER_MAX_Q = None

BRIDGE_CONFIG = BridgeConfig(
    name="17-pot control",
    packet_value_count=20,
    joint_bindings=tuple(
        JointBinding(
            name=f"control_{i:02d}",
            packet_index=i,
            joint_index=-1,         # placeholder — replace with real joint index
            kp=PLACEHOLDER_KP,
            kd=PLACEHOLDER_KD,
            scale=PLACEHOLDER_SCALE,
            offset=PLACEHOLDER_OFFSET,
            min_q=PLACEHOLDER_MIN_Q,
            max_q=PLACEHOLDER_MAX_Q,
        )
        for i in range(17)
    ),
)
