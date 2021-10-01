import glob
import sys
try:
    sys.path.append(
        glob.glob('C:\CARLA\CARLA_0.9.10\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.10-py3.7-win-amd64.egg')[0])
except IndexError:
    pass
import carla
import collections
import matplotlib.pyplot as plt

client = carla.Client('localhost', 2000)
client.set_timeout(2.5)

client.reload_world()
world = client.get_world()
world = client.load_world('Town04')

map = world.get_map()
# od_map = 'OpenDriveMaps/Town05.xodr'
# 779, 1421, 1595, 1594, 1442, 1432
roads_of_int = [761, 36, 862, 35, 43, 266, 42, 50, 1174, 49, 902, 48, 775, 47, 1073, 46, 144, 45, 6,
 41, 1400, 40, 1185, 39, 1092, 38, 1601, 37]

waypoints = map.generate_waypoints(1) # waypoints spaced every X meters

id_dict_wp = collections.defaultdict(list)
for waypoint in waypoints:
    if waypoint.lane_id==4 and waypoint.road_id in roads_of_int:
        id_dict_wp[f"{waypoint.road_id}"].append(waypoint)

plt.subplot(211)
# Invert the y axis since we follow UE4 coordinates
plt.gca().invert_yaxis()
plt.margins(x=0.5, y=0.1)
plt.plot(
    [wp.transform.location.x for wp in waypoints],
    [wp.transform.location.y for wp in waypoints],
    linestyle='', markersize=1, color='blue', marker='.')
plt.subplot(212)
# Invert the y axis since we follow UE4 coordinates
plt.gca().invert_yaxis()
plt.margins(x=0.5, y=0.1)
for road_id, waypoints_in_road in id_dict_wp.items():
    plt.plot(
        [wp.transform.location.x for wp in waypoints_in_road],
        [wp.transform.location.y for wp in waypoints_in_road],
        linestyle='', markersize=1, marker='.')
plt.show()

"""
# WAYPOINT dir
'get_junction', 'get_landmarks', 'get_landmarks_of_type',
'get_left_lane', 'get_right_lane', 'id', 'is_intersection',
'is_junction', 'junction_id', 'lane_change', 'lane_id',
'lane_type', 'lane_width', 'left_lane_marking', 'next',
'next_until_lane_end', 'previous', 'previous_until_lane_start',
'right_lane_marking', 'road_id', 's', 'section_id', 'transform'
"""
