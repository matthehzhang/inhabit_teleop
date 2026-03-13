from g1_bridge_lib import BridgeConfig, JointBinding


WRIST_KP = 24.0
WRIST_KD = 1.6


BRIDGE_CONFIG = BridgeConfig(
    name="left wrist joints",
    joint_bindings=(
        JointBinding(
            name="roll",
            packet_index=4,
            joint_index=19,
            kp=WRIST_KP,
            kd=WRIST_KD,
            scale=1.0,
            offset=0.0,
            min_q=None,
            max_q=None,
        ),
        JointBinding(
            name="pitch",
            packet_index=5,
            joint_index=20,
            kp=WRIST_KP,
            kd=WRIST_KD,
            scale=1.0,
            offset=0.0,
            min_q=None,
            max_q=None,
        ),
        JointBinding(
            name="yaw",
            packet_index=6,
            joint_index=21,
            kp=WRIST_KP,
            kd=WRIST_KD,
            scale=1.0,
            offset=0.0,
            min_q=None,
            max_q=None,
        ),
    ),
)
