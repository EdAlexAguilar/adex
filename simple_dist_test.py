"""
Loads Carla and 2 cars set on dummy wandering autopilot.
For debugging and testing purposes.
"""
import glob
import sys
try:
    sys.path.append(
        glob.glob('C:\CARLA\CARLA_0.9.10\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.10-py3.7-win-amd64.egg')[0])
except IndexError:
    pass
import carla


from collections import namedtuple

"""
WORLD SETUP -- synchronous mode with delta = 0.05
"""
od_map = 'OpenDriveMaps/Town02.xodr'
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

starting_location = spawn_points[19]  #town 2
# starting_location = spawn_points[1] # Town03  Near starting fountain
# 195 in Town05 starts at bottom highway

vehicles_list = []
for v in vehicles_test:
    blueprint = world.get_blueprint_library().find(v.name)
    blueprint.set_attribute('color', v.color)
    blueprint.set_attribute('role_name', 'autopilot')
    vehicle = world.spawn_actor(blueprint, starting_location)
    vehicle.set_autopilot(True, traffic_manager.get_port())
    vehicles_list.append(vehicle)
    starting_location.location.x -= 35.0

print(f'Spawned {len(vehicles_list)} vehicles.')

# example of how to use parameters
traffic_manager.global_percentage_speed_difference(30.0)

jeep = vehicles_list[1]
bmw = vehicles_list[0] # Ego


while True:
    try:
        world.tick()
    except KeyboardInterrupt:
        print('\n Destroying all Vehicles')
        client.apply_batch([carla.command.DestroyActor(v) for v in vehicles_list])
        client.reload_world()
        break



