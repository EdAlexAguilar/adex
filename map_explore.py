import glob
import sys
try:
    sys.path.append(
        glob.glob('C:\CARLA\CARLA_0.9.10\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.10-py3.7-win-amd64.egg')[0])
except IndexError:
    pass
import carla
import networkx as nx
import matplotlib.pyplot as plt


client = carla.Client('localhost', 2000)
client.set_timeout(6.5)

client.reload_world()
world = client.get_world()
world = client.load_world('Town02')

map = world.get_map()
waypoints = map.generate_waypoints(0.5) # waypoints spaced every X meters

topology = map.get_topology()






"""
# WAYPOINT dir
'get_junction', 'get_landmarks', 'get_landmarks_of_type',
'get_left_lane', 'get_right_lane', 'id', 'is_intersection',
'is_junction', 'junction_id', 'lane_change', 'lane_id',
'lane_type', 'lane_width', 'left_lane_marking', 'next',
'next_until_lane_end', 'previous', 'previous_until_lane_start',
'right_lane_marking', 'road_id', 's', 'section_id', 'transform'
"""
