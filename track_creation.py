import glob
import sys
try:
    sys.path.append(
        glob.glob('C:\CARLA\CARLA_0.9.10\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.10-py3.7-win-amd64.egg')[0])
except IndexError:
    pass
import carla
import map_utils
import numpy as np
import functools

od_map = 'OpenDriveMaps/Town04.xodr'
client = carla.Client('localhost', 2000)
world = client.load_world('Town04')
carla_map = world.get_map()


WAYPOINT_DIST = 1
# This list of roads corresponds to the 8-shaped loop
TRACK_ROADS = [761, 36, 862, 35, 43, 266, 42, 50, 1174, 49, 902, 48, 775, 47, 1073, 46, 144, 45, 6,
 41, 1400, 40, 1185, 39, 1092, 38, 1601, 37]
waypoints =[w for w in carla_map.generate_waypoints(WAYPOINT_DIST) if w.road_id in TRACK_ROADS and w.lane_id==4]

processed_map = map_utils.OpenDriveMap(od_map, carla_map)
TRACK_LENGTH = 0
for track_road in TRACK_ROADS:
    od_road = processed_map.road_from_id(str(track_road))
    TRACK_LENGTH += float(od_road.get('length'))

# print(TRACK_LENGTH) # =  3049.9244

CONES_EVERY = 15
static_waypoints =[]
static_waypoints.append([w for w in carla_map.generate_waypoints(CONES_EVERY) if w.road_id in TRACK_ROADS and w.lane_id==5])
static_waypoints.append([w for w in carla_map.generate_waypoints(CONES_EVERY) if w.road_id in TRACK_ROADS and w.lane_id==3])
static_items = []
print(f"Spawning {functools.reduce(lambda count, l: count + len(l), cone_waypoints, 0)} Static Items.")
for sublist in static_waypoints:
    for wp in sublist:
        blueprint = world.get_blueprint_library().find(f'static.prop.streetbarrier') # constructioncone') #trafficcone01')
        cone = world.spawn_actor(blueprint, wp.transform)
        static_items.append(cone)

while True:
    try:
        continue
    except KeyboardInterrupt:
        print('\n Destroying all Actors')
        client.apply_batch([carla.command.DestroyActor(s) for s in static_items])
        client.reload_world()
        break