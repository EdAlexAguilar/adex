import glob
import sys
try:
    sys.path.append(
        glob.glob('C:\CARLA\CARLA_0.9.10\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.10-py3.7-win-amd64.egg')[0])
except IndexError:
    pass
import carla
import rss
import map_utils
import numpy as np

LOG_FILE = str(r'C:\Users\aguilare\GitHub\adex\recording_scenic_3.log')

od_map = 'OpenDriveMaps/Town05.xodr'

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.load_world(od_map[-11:-5])
carla_map = world.get_map()


new_settings = world.get_settings()
new_settings.synchronous_mode = True
new_settings.fixed_delta_seconds = 0.05
world.apply_settings(new_settings)

# arguments:: show all
# recording_info = client.show_recorder_file_info(LOG_FILE, False)

# arguments:: Start time (0 = beginning), Duration (0 = all), Camera (0 free float, or id)
print(client.replay_file(LOG_FILE, 0, 0, 815))
world.tick() # first tick to start replay file

actor_list = world.get_actors()
# Find an actor by id.
vehicles = [v for v in actor_list.filter('vehicle.*')]
walkers = [w for w in actor_list.filter('walker.*')]

for i, v in enumerate(vehicles):
    print(f"Vehicle {i} : Id: {v.id} : Type {v.type_id}")


def straight_line_distance(vehicle1, vehicle2):
    loc1 = rss_monitor.get_vehicle_location(vehicle1)
    loc2 = rss_monitor.get_vehicle_location(vehicle2)
    wp1 = processed_map.carla_map.get_waypoint(loc1)
    wp2 = processed_map.carla_map.get_waypoint(loc2)
    return processed_map.waypoint_distance(wp1, wp2)



processed_map = map_utils.OpenDriveMap(od_map, carla_map)
rss_monitor = rss.RSSMonitor(vehicles[1], [vehicles[0]], processed_map)
straight_line_monitor = np.array([])
step_counter = 0

# Simulation loop
while True:
    try:
        world.tick()
        straight_line_monitor = np.append(straight_line_monitor, straight_line_distance(vehicles[0], vehicles[1]))
        rss_monitor.update()
    except KeyboardInterrupt:
        print('\n Destroying all Vehicles')
        distance_trace = np.array(rss_monitor.distance_trace)
        distance_trace = distance_trace.T[0]
        np.savetxt("replay_monitor_test.csv", np.stack((distance_trace, straight_line_monitor)).T, delimiter=",")
        client.apply_batch([carla.command.DestroyActor(v) for v in vehicles])
        client.reload_world()
        break

