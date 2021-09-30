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
        glob.glob('C:\CARLA\CARLA_0.9.10\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.10-py3.7-win-amd64.egg')[0])
except IndexError:
    pass
import carla

import numpy as np
from collections import namedtuple

"""
WORLD SETUP -- synchronous mode with delta = 0.05
"""

od_map = 'OpenDriveMaps/Town03.xodr'
print(f'USING MAP: {od_map[:-5]}')

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
synchronous_master = False

world = client.get_world()
# world = client.load_world('Town02')
map = world.get_map()


traffic_manager = client.get_trafficmanager(8000)
traffic_manager.set_global_distance_to_leading_vehicle(1.0)
traffic_manager.set_synchronous_mode(True)

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
synchronous_master = True

blueprints_v = world.get_blueprint_library().filter("vehicle.*")
spawn_points = world.get_map().get_spawn_points()


Car = namedtuple('vehicle_properties',field_names=['name','color'])
vehicles_test = [Car('vehicle.bmw.grandtourer','255,21,0'),
                 Car('vehicle.jeep.wrangler_rubicon','0,55,255')]
# taken from blueprint.id

# starting_location = spawn_points[19]  #town 2
starting_location = spawn_points[1] # Town03  Near starting fountain
# 195 in Town05 starts at bottom highway

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

# example of how to use parameters
traffic_manager.global_percentage_speed_difference(30.0)

jeep = vehicles_list[1]
bmw = vehicles_list[0] # Ego

processed_map = map_utils.OpenDriveMap(od_map, map)
rss_monitor = rss.RSSMonitor(bmw, [jeep], processed_map)

import networkx as nx

def straight_line_distance(vehicle1, vehicle2):
    loc1 = rss_monitor.get_vehicle_location(vehicle1)
    loc2 = rss_monitor.get_vehicle_location(vehicle2)
    wp1 = processed_map.carla_map.get_waypoint(loc1)
    wp2 = processed_map.carla_map.get_waypoint(loc2)
    return processed_map.waypoint_distance(wp1, wp2)

def print_diagnostics(ego, actor):
    ego_loc = rss_monitor.get_vehicle_location(ego)
    actor_loc = rss_monitor.get_vehicle_location(actor)
    ego_wp = processed_map.carla_map.get_waypoint(ego_loc)
    actor_wp = processed_map.carla_map.get_waypoint(actor_loc)
    print(f"Staight Line Distance Between Cars:\n {processed_map.waypoint_distance(ego_wp, actor_wp)}")
    print(f"Longitudinal Distance (ego,actor):: {processed_map.longitudinal_road_distance(ego_loc,actor_loc)}"
          f"(actor,ego):: {processed_map.longitudinal_road_distance(actor_loc,ego_loc)}")
    print(f"Verbose all options")
    d = processed_map.longitudinal_road_distance(ego_loc, actor_loc, verbose=True)
    d=nx.shortest_path_length(processed_map.full_topology, source=str(ego_wp.road_id),
                            target=str(actor_wp.road_id), weight="distance")
    print(f"Shortest Path Length on Naive WP: {d}")
    d_route = nx.shortest_path(processed_map.full_topology, source=str(ego_wp.road_id),
                                target=str(actor_wp.road_id), weight="distance")
    print(f"Shortest Path Route on Naive WP: {d_route}")


print_dt = 80
dt = 0
straight_line_monitor = np.array([])
step_counter = 0
while True:
    try:
        world.tick()
        rss_monitor.update()
        dt +=1

        straight_line_monitor = np.append(straight_line_monitor, straight_line_distance(bmw, jeep))
        if rss_monitor.distance_trace[-1][0]<0:
            break
        if dt==print_dt:
            dt = 0
            print(f"\n")
            print(f"BMW  (Ego):")
            rss_monitor.print_vehicle_road_info(bmw)
            print(f"Jeep:")
            rss_monitor.print_vehicle_road_info(jeep)
            print(f"Distance: {rss_monitor.distance_trace[-1]}")
            print_diagnostics(bmw, jeep)
            print(f"--------------------------------------------")
        if step_counter>=2:
            delta_straight = abs(straight_line_monitor[-2]-straight_line_monitor[-1])
            delta_road = abs(rss_monitor.distance_trace[-2][0]-rss_monitor.distance_trace[-1][0])
            if 2 < 2*delta_straight < delta_road:
                print(f"\n\n\n")
                print(f"Big Jump!")
                print(f"Linear Distance Change: {delta_straight}")
                print(f"Road Longitudinal Distance Change: {delta_road}")
                print(f"\n\n\n")
        step_counter += 1
    except KeyboardInterrupt:
        print('\n Destroying all Vehicles')
        distance_trace = np.array(rss_monitor.distance_trace)
        distance_trace = distance_trace.T[0]
        np.savetxt("simple_monitor_test.csv", np.stack((distance_trace, straight_line_monitor)).T, delimiter=",")
        client.apply_batch([carla.command.DestroyActor(v) for v in vehicles_list])
        client.reload_world()
        break



