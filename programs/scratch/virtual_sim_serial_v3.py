import time
import math
import sys

from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )

def mode_check():
    if len(sys.argv) < 2:
        #check for python progname.py enp2s0. if less than 2 args
        #then user is not running w a NIC (network interface card)
        #........................................................
        #if user does not provide name of the network interface
        #ex: eth0, enp2s0, wlan0, lo <- here, enp2s0 is the NIC
        #THEN user is running SIM

        #use dds domain ID 1
        #  - processes using domain # can see each other
        #  - mujuco sim uses domain 1
        #use loopback ntwk interface (which is local machine com only)
        # - this arg chooses which network interface DDS uses
        #basically, only comm w/ other dds process on domain 1 and 
        #using loopback interface
        #..........................................................
        #once comm is set up, publisher (writes commands to DDS) and
        #subscribers (reads state messafges from DDS) can be created
        ChannelFactoryInitialize(1, "lo") #sim
        return "sim" #return sim to let main know its on sim

    else: 
        #use dds domain ID 0
        # - join domain 0
        #use sys.argv[1] as the network interface name
        # - probably enp2s0
        ChannelFactoryInitialize(0, sys.argv[1]) #real robot nic
        return "robot" #return robot

def begin_publisher_subscriber():
    cmd_pub = ChannelPublisher(CMD_TOPIC, HG_LowCmd) 
    cmd_pub.Init()

    state_sub = ChannelSubscriber(STATE_TOPIC, HG_LowState)
    state_sub.Init()
    
    return cmd_pub, state_sub



def init():
    mode = mode_check()
    return mode

def main():
    mode = init()
