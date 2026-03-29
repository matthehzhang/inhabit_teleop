# pot-to-joint config for the 24ch MCP3008 teleop rig
# maps pot channels (packet indices) to G1 joint indices
#
# chip0 (ch 0-7):  left arm (7 joints) + waist yaw
# chip1 (ch 8-15): right arm (7 joints) + waist roll
# chip2 (ch 16-19): left hand (4 pots → 7 hand motors, some coupled)
# chip2 (ch 20-23): right hand (4 pots → 7 hand motors, some coupled)
#
# for coupled finger joints (one pot drives two motors), use the same
# packet_index on multiple bindings. eg index_0 and index_1 share a pot.
#
# hand joints: set hand="left" or hand="right", joint_index = hand motor 0-6
# dex3-1 hand motors: 0=thumb_0, 1=thumb_1, 2=thumb_2,
#                      3=index_0, 4=index_1, 5=middle_0, 6=middle_1
#
# scale = radians per volt. ~1.59 for a 300deg pot over 3.3V.
# tune per joint once the rig is wired.

from g1_bridge_lib_20ch import BridgeConfig, JointBinding

# 300 degree pot across 3.3V => ~1.59 rad/V
POT_SCALE = 1.59
FINGER_SCALE = 0.8  # fingers have smaller range, tune this

BRIDGE_CONFIG = BridgeConfig(
    name="pot teleop",
    packet_value_count=24,
    joint_bindings=(
        # chip 0: right arm + waist yaw
        JointBinding(name="right_shoulder_pitch", packet_index=0,  joint_index=22, kp=40.0, kd=1.0, scale=POT_SCALE, max_position_error=0.7,  max_dq=2.5),
        JointBinding(name="right_shoulder_roll",  packet_index=1,  joint_index=23, kp=40.0, kd=1.0, scale=POT_SCALE, max_position_error=0.7,  max_dq=2.5),
        JointBinding(name="right_shoulder_yaw",   packet_index=2,  joint_index=24, kp=40.0, kd=1.0, scale=POT_SCALE, max_position_error=0.7,  max_dq=2.5),
        JointBinding(name="right_elbow",          packet_index=3,  joint_index=25, kp=40.0, kd=1.0, scale=-POT_SCALE, max_position_error=0.7,  max_dq=2.5),
        JointBinding(name="right_wrist_roll",     packet_index=4,  joint_index=26, kp=40.0, kd=1.0, scale=-POT_SCALE, max_position_error=0.7,  max_dq=2.5),
        JointBinding(name="right_wrist_pitch",    packet_index=6,  joint_index=27, kp=40.0, kd=1.0, scale=0.0),
        JointBinding(name="right_wrist_yaw",      packet_index=5,  joint_index=28, kp=40.0, kd=1.0, scale=-POT_SCALE, max_position_error=0.7,  max_dq=2.5),
        #JointBinding(name="waist_yaw",           packet_index=7,  joint_index=12, kp=40.0, kd=2.0, scale=POT_SCALE),

        # chip 1: right arm + waist roll
        #JointBinding(name="right_shoulder_pitch", packet_index=8,  joint_index=22, kp=20.0, kd=1.0, scale=POT_SCALE),
        #JointBinding(name="right_shoulder_roll",  packet_index=9,  joint_index=23, kp=20.0, kd=1.0, scale=POT_SCALE),
        #JointBinding(name="right_shoulder_yaw",   packet_index=10, joint_index=24, kp=20.0, kd=1.0, scale=POT_SCALE),
        #JointBinding(name="right_elbow",          packet_index=11, joint_index=25, kp=20.0, kd=1.0, scale=POT_SCALE),
        #JointBinding(name="right_wrist_roll",     packet_index=12, joint_index=26, kp=20.0, kd=1.0, scale=POT_SCALE),
        #JointBinding(name="right_wrist_pitch",    packet_index=13, joint_index=27, kp=20.0, kd=1.0, scale=POT_SCALE),
        #JointBinding(name="right_wrist_yaw",      packet_index=14, joint_index=28, kp=20.0, kd=1.0, scale=POT_SCALE),
        #JointBinding(name="waist_roll",           packet_index=15, joint_index=13, kp=40.0, kd=2.0, scale=POT_SCALE),

        # chip 2 ch 16-19: left hand (4 pots, 7 motors)
        # pot 16 = thumb (drives thumb_0)
        #JointBinding(name="l_thumb_0",   packet_index=16, joint_index=0, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="left"),
        # pot 17 = thumb curl (drives thumb_1 and thumb_2 together)
        #JointBinding(name="l_thumb_1",   packet_index=17, joint_index=1, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="left"),
        #JointBinding(name="l_thumb_2",   packet_index=17, joint_index=2, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="left"),
        # pot 18 = index (drives index_0 and index_1 together)
        #JointBinding(name="l_index_0",   packet_index=18, joint_index=3, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="left"),
        #JointBinding(name="l_index_1",   packet_index=18, joint_index=4, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="left"),
        # pot 19 = middle (drives middle_0 and middle_1 together)
        #JointBinding(name="l_middle_0",  packet_index=19, joint_index=5, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="left"),
        #JointBinding(name="l_middle_1",  packet_index=19, joint_index=6, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="left"),

        # chip 2 ch 20-23: right hand (4 pots, 7 motors)
        #JointBinding(name="r_thumb_0",   packet_index=20, joint_index=0, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="right"),
        #JointBinding(name="r_thumb_1",   packet_index=21, joint_index=1, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="right"),
        #JointBinding(name="r_thumb_2",   packet_index=21, joint_index=2, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="right"),
        #JointBinding(name="r_index_0",   packet_index=22, joint_index=3, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="right"),
        #JointBinding(name="r_index_1",   packet_index=22, joint_index=4, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="right"),
        #JointBinding(name="r_middle_0",  packet_index=23, joint_index=5, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="right"),
        #JointBinding(name="r_middle_1",  packet_index=23, joint_index=6, kp=10.0, kd=0.5, scale=FINGER_SCALE, hand="right"),
    ),
)
