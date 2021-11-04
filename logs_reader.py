import argparse
import pathlib
import os, sys
import glob

try:
    assert "CARLA_HOME" in os.environ
    sys.path.append(
        glob.glob(f'{os.environ["CARLA_HOME"]}\PythonAPI\carla\dist\carla-0.9.10-py3.7-win-amd64.egg')[0])
except IndexError:
    pass
import carla

import numpy as np


def to_carla_timestamp(dict):
    return carla.Timestamp(frame=dict['frame'],
                           elapsed_seconds=dict['elapsed_seconds'],
                           delta_seconds=dict['delta_seconds'],
                           platform_timestamp=dict['platform_timestamp'])


def to_carla_location(dict):
    return carla.Location(x=dict['x'], y=dict['y'], z=dict['z'])


def to_carla_vector3d(dict):
    return carla.Vector3D(x=dict['x'], y=dict['y'], z=dict['z'])


# Input
parser = argparse.ArgumentParser()
parser.add_argument("--logfile", type=pathlib.Path, required=True,
                    help="numpy file with logs in our last-minute data format")
args = parser.parse_args()

file = args.logfile

# Loading data
filecontent = np.load(file, allow_pickle=True)
worldinfo = filecontent["world_info"]
simdata = filecontent["data"]

# Print simulation information
print(f"map name: {worldinfo['map']}")
print(f"sim settings: \n\t" + '\n\t'.join([f"{k}: {v}" for k, v in worldinfo['settings'].items()]))
print()

# Iterating over simulation data
for frame_id, snapshot in simdata.items():
    timestamp = to_carla_timestamp(snapshot['timestamp'])
    actor_status = snapshot['actor_status']
    print(f"timestamp: {timestamp}")
    for actor in actor_status.keys():
        actor_type = actor_status[actor]["type_id"]
        location = to_carla_location(actor_status[actor]["location"])
        velocity = to_carla_vector3d(actor_status[actor]["velocity"])
        acceleration = to_carla_vector3d(actor_status[actor]["acceleration"])
        # debug
        print(f"\tactor_id: {actor}, type: {actor_type}")
        print(f"\t\tlocation: {location}")
        print(f"\t\tvelocity: {velocity}")
        print(f"\t\tacceleration: {acceleration}")
    print()
