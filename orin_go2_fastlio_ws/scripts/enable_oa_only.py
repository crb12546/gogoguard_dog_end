#!/usr/bin/env python3
import sys
import time

SDK_PATH = "/home/unitree/unitree_sdk2_python"
if SDK_PATH not in sys.path:
    sys.path.insert(0, SDK_PATH)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient

def main():
    iface = "eth0"
    if len(sys.argv) > 1:
        iface = sys.argv[1]

    print(f"[enable_oa_only] init sdk on interface: {iface}", flush=True)
    ChannelFactoryInitialize(0, iface)

    client = ObstaclesAvoidClient()
    client.SetTimeout(3.0)
    client.Init()

    print("[enable_oa_only] SwitchSet(True)", flush=True)
    ret = client.SwitchSet(True)
    print(f"[enable_oa_only] SwitchSet ret: {ret}", flush=True)

    time.sleep(0.5)

    print("[enable_oa_only] SwitchGet", flush=True)
    ret, enabled = client.SwitchGet()
    print(f"[enable_oa_only] SwitchGet ret: {ret}, enabled: {enabled}", flush=True)

    print("[enable_oa_only] UseRemoteCommandFromApi(False)", flush=True)
    ret = client.UseRemoteCommandFromApi(False)
    print(f"[enable_oa_only] UseRemoteCommandFromApi ret: {ret}", flush=True)

    print("[enable_oa_only] DONE: obstacle avoidance switch enabled", flush=True)

if __name__ == "__main__":
    main()
