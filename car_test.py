"""
Loads Carla and 2 cars set on dummy wandering autopilot.
For debugging and testing purposes.
"""
import glob
import sys
import rss
import map_utils

try:
    sys.path.append(
        glob.glob('C:\CARLA\CARLA_0.9.11\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.11-py3.7-win-amd64.egg')[0])
except IndexError:
    pass
import carla

import numpy as np
from collections import namedtuple

"""
WORLD SETUP -- synchronous mode with delta = 0.05
"""

od_map = 'OpenDriveMaps/Town05.xodr'
print(f'USING MAP: {od_map[:-5]}')

client = carla.Client('localhost', 2000)
client.set_timeout(3.0)

world = client.load_world(od_map[-11:-5])
map = world.get_map()

traffic_manager = client.get_trafficmanager(8000)
traffic_manager.set_global_distance_to_leading_vehicle(1.0)
traffic_manager.set_synchronous_mode(True)
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

Car = namedtuple('vehicle_properties',field_names=['name','color'])
vehicles_test = [Car('vehicle.bmw.grandtourer','255,21,0'),
                 Car('vehicle.jeep.wrangler_rubicon','0,55,255')]

spawn_points = world.get_map().get_spawn_points()
starting_location = spawn_points[1]

processed_map = map_utils.OpenDriveMap(od_map, map)

"""
vehicles_list = []
for v in vehicles_test:
    blueprint = world.get_blueprint_library().find(v.name)
    blueprint.set_attribute('color', v.color)
    blueprint.set_attribute('role_name', 'autopilot')
    vehicle = world.spawn_actor(blueprint, starting_location)
    vehicle.set_autopilot(True, traffic_manager.get_port())
    vehicles_list.append(vehicle)
    starting_location.location.x += 35.0

print(f'Spawned {len(vehicles_list)} vehicles.')

jeep = vehicles_list[1]
bmw = vehicles_list[0] # Ego

processed_map = map_utils.OpenDriveMap(od_map, map)
rss_monitor = rss.RSSMonitor(bmw, jeep, processed_map)

step_counter = 0
while step_counter<600:
    step_counter+=1
    world.tick()

"""