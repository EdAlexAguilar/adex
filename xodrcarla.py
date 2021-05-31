import glob
import sys
try:
    sys.path.append(
        glob.glob('C:\CARLA\CARLA_0.9.10\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.10-py3.7-win-amd64.egg')[0])
except IndexError:
    pass
import carla


client = carla.Client('localhost', 2000)
client.set_timeout(6.5)


od_map = 'OpenDriveMaps/hotspot1.xodr'
with open(od_map, 'r') as file:
    map_str = file.read().replace('\n', '')

client.generate_opendrive_world(map_str)



#client.reload_world()
#world = client.load_world('Town02')