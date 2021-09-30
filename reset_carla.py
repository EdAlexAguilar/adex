import glob
import sys
try:
    sys.path.append(
        glob.glob('C:\CARLA\CARLA_0.9.10\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.10-py3.7-win-amd64.egg')[0])
except IndexError:
    pass
import carla

client = carla.Client('localhost', 2000)
# client.set_timeout(6.5)

# client.reload_world()
world = client.load_world('Town03')

map = world.get_map()

# client.replay_file("recording_scenic_1.log")
# client.show_recorder_file_info("recording_scenic_1.log")