"""
Implements Simple RSS Monitor in Carla
Needs xodr file to work!
"""

import xml.etree.ElementTree as ET
import networkx as nx
import numpy as np
from collections import namedtuple

import constants as c

SETUP -- synchronous mode with delta = 0.05

od_map = 'OpenDriveMaps/Town02.xodr'

map = world.get_map()


def pretty_topology(map):
    def wp_data(waypoint):
        return (waypoint.road_id, waypoint.lane_id) #, waypoint.s)
    raw_topology = map.get_topology()
    topology = []
    for edge in raw_topology:
        r0, l0 = wp_data(edge[0])
        r1, l1 = wp_data(edge[1])
        e = ((r0,l0), (r1,l1))
        topology.append(e)
        # topology.append((edge[0].road_id, edge[1].road_id))
    return topology

topology = pretty_topology(map)
G = nx.Graph()
G.add_edges_from(topology)

def get_road_info(vehicle):
    """
    Given a carla.Vehicle object, returns
    road_id, lane_id, s, section_id
    """
    global map
    vehicle_loc = vehicle.get_location()
    vehicle_wp = map.get_waypoint(vehicle_loc) # has project_to_road=True so wp is at center of closest lane
    return vehicle_wp.road_id, vehicle_wp.lane_id, vehicle_wp.s, vehicle_wp.section_id

def print_road_info(vehicle):
    road_id, lane_id, s, section_id = get_road_info(vehicle)
    print(f'Road ID: {road_id}  Lane ID: {lane_id}  S: {s}  Section ID: {section_id}')

def get_long_distances(ego, v_actors):
    global map
    global G # minimum network topology
    ego_wp = map.get_waypoint(ego.get_location())
    ego_road = ego_wp.road_id
    ego_lane = ego_wp.lane_id
    ego_s = ego_wp.s
    ego_loc = (ego_road, ego_lane)
    distances = []
    for v in v_actors:
        v_wp = map.get_waypoint(v.get_location())
        v_road = v_wp.road_id
        v_lane = v_wp.lane_id
        v_s = v_wp.s
        v_loc = (v_road, v_lane)
        min_path = nx.shortest_path(G, ego_loc, v_loc)
        min_path = [x[0] for x in min_path]
        min_path_roads = roads_from_id(min_path)
        s = longitudinal_dist(ego_s, ego_lane, v_s, v_lane, min_path_roads)
        distances.append(s)
        # road_info = [ego_road, ego_lane, ego_s, v_road, v_lane, v_s]
    return distances, min_path

def find_lane_direction(road, lane_id):
    for side in list(road.find('lanes').find('laneSection')):
        for lane in list(side):
            if lane.get('id') == lane_id:
                return lane.find('userData').find('vectorLane').get('travelDir')

def longitudinal_dist(s1, lane1, s2, lane2, road_list):
    """
    s1, s2:  position of car1, car2 in their road
    road_list : car1 is at element 0, car2 at element -1 : elements are opendrive road

    Assumes roads on road_list are actually connected! It will only sum the
    lengths of the roads and not check connectivity
    """
    assert s1 <= float(road_list[0].get('length'))
    assert s2 <= float(road_list[-1].get('length'))
    if len(road_list) == 1:
        return abs(s1 - s2)
    road1_dist = s1
    road2_dist = s2
    if find_lane_direction(road_list[0], str(lane1)) == 'forward':
        road1_dist *= -1
        road1_dist += float(road_list[0].get('length'))
    if find_lane_direction(road_list[-1], str(lane2)) == 'backward':
        road2_dist *= -1
        road2_dist += float(road_list[-1].get('length'))

    '''
    for link_element in list(road_list[1].find('link')):
        if link_element.get('elementId') == road_list[0].get('id'):
            if link_element.get('contactPoint') == 'end':
                road1_dist *= -1
                road1_dist += float(road_list[0].get('length'))
    for link_element in list(road_list[-2].find('link')):
        if link_element.get('elementId') == road_list[-1].get('id'):
            if link_element.get('contactPoint') == 'end':
                road2_dist *= -1
                road2_dist += float(road_list[-1].get('length'))
    '''
    intermediate_dist = 0
    for road in road_list[1:-1]:
        intermediate_dist += float(road.get('length'))
    return road1_dist + road2_dist + intermediate_dist


def is_drivable(road):
    """
    Returns True if road has at least one drivable lane
    False otherwise
    """
    sections = list(road.find('lanes').find('laneSection'))
    for section in sections:
        sect_lanes = section.findall('lane')
        for lane in sect_lanes:
            if lane.get('type') == 'driving':
                return True
    return False

tree = ET.parse(od_map)
root = tree.getroot()
map_elements = list(root) # includes 1 'header' multiple 'road' 'controller' 'junction'
road_elements = root.findall('road')
road_elements = [road for road in road_elements if is_drivable(road)]
junction_elements = root.findall('junction')
print(f'{od_map} has {len(road_elements)} roads and {len(junction_elements)} junctions')
# attrib
nonjunc_roads = [road for road in road_elements if road.attrib['junction']=='-1']


def roads_from_id(road_id_list):
    global road_elements
    roads = []
    r_ids = [r.get('id') for r in road_elements]
    for id in road_id_list:
        ii = r_ids.index(str(id))
        roads.append(road_elements[ii])
    return roads


def safe_long_dist(v_behind, v_front):
    tau = c.REACTION_TIME
    acc_max = c.ACCEL_MAX
    brake_min = c.BRAKE_MIN
    brake_max = c.BRAKE_MAX
    d_safe = v_behind*tau + 0.5*acc_max*tau**2 + (v_behind + tau*acc_max)**2/(2*brake_min) - v_front**2/(2*brake_max)
    return d_safe

def vector_magnitude(vec):
    return np.sqrt(vec.x**2 + vec.y**2 + vec.z**2)


distance_trace = []
safe_dist_trace = []
# road_info_trace = []
dt = 0 ## world tick counter, used for printing
while True:
    try:
        world.tick()
        dt += 1
        d, connecting_roads = get_long_distances(bmw, [jeep]) # [0]
        distance_trace.append(d[0])
        bmw_loc, bmw_vel = bmw.get_location(), vector_magnitude(bmw.get_velocity())
        jeep_loc,  jeep_vel = jeep.get_location(), vector_magnitude(jeep.get_velocity())
        safe_dist_trace.append(safe_long_dist(bmw_vel, jeep_vel))
        # road_info_trace.append(r_info)
        if dt==20:
            dt=0
            print(f'JEEP')
            print_road_info(jeep)
            print(f'BMW')
            print_road_info(bmw)
            print(f'Lane Distance: {d[0]}')
            print(f' Connecting Roads: {connecting_roads}')
            print(f'Safe Dist: {safe_dist_trace[-1]}')


    except KeyboardInterrupt:
        print('\n Destroying all Vehicles')
        client.apply_batch([carla.command.DestroyActor(v) for v in vehicles_list])
        # client.reload_world()
        np.savetxt("dist_trace.csv", distance_trace, delimiter=",")
        np.savetxt("safe_dist_trace.csv", safe_dist_trace, delimiter=",")
        # np.savetxt("road_info_trace.csv", road_info_trace, delimiter=",")
        break



