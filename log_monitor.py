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

# For DEMO
# LOG_FILE = str(r'C:\Users\aguilare\GitHub\adex\record_11102021_140409.log')
# od_map = 'OpenDriveMaps/Town04.xodr'

# For DEBUGGING
LOG_FILE = str(r'C:\Users\aguilare\GitHub\adex\recording_scenic_1.log')
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
recording_info = client.show_recorder_file_info(LOG_FILE, False)

# arguments:: Start time (0 = beginning), Duration (0 = all), Camera (0 free float, or id)
print(client.replay_file(LOG_FILE, 0, 0, 0))
world.tick() # first tick to start replay file

actor_list = world.get_actors()
# Find an actor by id.
vehicles = [v for v in actor_list.filter('vehicle.*')]
print(f"Found {len(vehicles)} Vehicles")
walkers = [w for w in actor_list.filter('walker.*')]

for i, v in enumerate(vehicles):
    print(f"Vehicle {i} : Id: {v.id} : Type {v.type_id}")

processed_map = map_utils.OpenDriveMap(od_map, carla_map)
rss_monitor = rss.RSSMonitor(vehicles[0], vehicles[1], processed_map)

step_counter = 0

# Simulation loop

while True:
    try:
        world.tick()
        rss_monitor.update()
        #step_counter += 1
        #if step_counter%40==0:
        #    print(vehicles[0].get_velocity().x, vehicles[0].get_velocity().y)
    except KeyboardInterrupt:
        print('\n Destroying all Vehicles')
        # rss_monitor.straight_distance_trace
        safe1 = np.stack((np.array(rss_monitor.long_distance_trace),
                          np.array(rss_monitor.ego_long_velocity_trace),
                          np.array(rss_monitor.actor_long_velocity_trace),
                          np.array(rss_monitor.ego_long_acceleration_trace),
                          np.array(rss_monitor.actor_long_acceleration_trace))).T
        np.savetxt("example_safe1.csv", safe1, delimiter=",")
        safe2 = np.stack((np.array(rss_monitor.lat_distance_trace),
                          np.array(rss_monitor.ego_lat_velocity_trace),
                          np.array(rss_monitor.actor_lat_velocity_trace),
                          np.array(rss_monitor.ego_lat_acceleration_trace),
                          np.array(rss_monitor.actor_lat_acceleration_trace))).T
        np.savetxt("example_safe2.csv", safe2, delimiter=",")
        client.apply_batch([carla.command.DestroyActor(v) for v in vehicles])
        client.reload_world()
        break

