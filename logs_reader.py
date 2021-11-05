import argparse
import pathlib
import sys
import glob

try:
    sys.path.append(
        glob.glob('C:\CARLA\CARLA_0.9.11\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.11-py3.7-win-amd64.egg')[0])
except IndexError:
    pass
import carla

import numpy as np
import geometric_offline_rss
import comfort
import law_left
import map_utils

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

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.load_world(worldinfo['map'])
carla_map = world.get_map()
od_map = 'OpenDriveMaps/' + worldinfo['map'] + '.xodr'

processed_map = map_utils.OpenDriveMap(od_map, carla_map)
rss_monitors = []
law_monitors = []
EGO = "vehicle.jeep.wrangler_rubicon"

step_counter = 0

def actor_road_info(actor_id, actor_status):
    loc = to_carla_location(actor_status[actor_id]["location"])
    wp = carla_map.get_waypoint(loc)
    return wp.road_id, wp.lane_id

first_frame, first_snapshot = list(simdata.items())[0]
actor_status = first_snapshot['actor_status']
for actor_id in actor_status.keys():
    if actor_status[actor_id]["type_id"]==EGO:
        ego_id = actor_id
        comfort_monitor = comfort.ComfortMonitor(ego_id)
        ego_road, ego_lane = actor_road_info(ego_id, actor_status)

for actor_id in actor_status.keys():
    if actor_id != ego_id:
        rss_monitors.append(geometric_offline_rss.GeometricRSSOfflineMonitor(ego_id, actor_id, processed_map))
        actor_road, actor_lane = actor_road_info(actor_id, actor_status)
        if (ego_road, ego_lane) != (actor_road, actor_lane):
            law_monitors.append(law_left.SafeLeftMonitor(ego_id, ego_road,
                                                         actor_id, actor_road,
                                                         processed_map))
            law_actor_id = actor_id

assert len(law_monitors) == 1
law_monitor = law_monitors[0]

# Iterating over simulation data
for frame_id, snapshot in simdata.items():
    timestamp = to_carla_timestamp(snapshot['timestamp'])
    actor_status = snapshot['actor_status']
    for monitor in rss_monitors:
        monitor.update(timestamp.elapsed_seconds, actor_status)
    comfort_monitor.update(timestamp.elapsed_seconds, actor_status)
    law_monitor.update(timestamp.elapsed_seconds, actor_status)

# Exporting to csv
for monitor in rss_monitors:
    ego = monitor.ego
    actor = monitor.actor

    safe1 = np.stack((np.array(monitor.timestamps),
                              np.array(monitor.long_distance_trace),
                              np.array(monitor.ego_long_velocity_trace),
                              np.array(monitor.actor_long_velocity_trace),
                              np.array(monitor.ego_long_acceleration_trace),
                              np.array(monitor.actor_long_acceleration_trace),
                              np.array(monitor.actor_angle_trace))).T
    np.savetxt(f"safe1_{ego}_{actor}.csv", safe1, delimiter=",")

    safe2 = np.stack((np.array(monitor.timestamps),
                      np.array(monitor.lat_distance_trace),
                      np.array(monitor.ego_lat_velocity_trace),
                      np.array(monitor.actor_lat_velocity_trace),
                      np.array(monitor.ego_lat_acceleration_trace),
                      np.array(monitor.actor_lat_acceleration_trace),
                      np.array(monitor.actor_angle_trace))).T
    np.savetxt(f"safe2_{ego}_{actor}.csv", safe2, delimiter=",")

comfort = np.stack((np.array(comfort_monitor.timestamps),
                    np.array(comfort_monitor.trace_derivative(comfort_monitor.ego_long_acceleration_trace, delta=0.1)),
                    np.array(comfort_monitor.trace_derivative(comfort_monitor.ego_lat_acceleration_trace, delta=0.1)))).T
np.savetxt(f"comfort_{ego}.csv", comfort, delimiter=",")


legal = np.stack((np.array(law_monitor.timestamps),
                    np.array(law_monitor.ego_long_velocity_trace),
                    np.array(law_monitor.actor_long_velocity_trace),
                    np.array(law_monitor.ego_long_acceleration_trace),
                    np.array(law_monitor.ego_distance_to_junction_trace),
                    np.array(law_monitor.ego_in_junction_trace),
                    np.array(law_monitor.actor_distance_to_junction_trace),
                    np.array(law_monitor.actor_angle_trace))).T
np.savetxt(f"legal_{ego}_{law_actor_id}.csv", legal, delimiter=",")

client.reload_world()
"""
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
"""