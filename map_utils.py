"""
OpenDriveMap class
Contains methods to calculate "longitudinal" lane distance between two points of a map
"""
import xml.etree.ElementTree as ET
import networkx as nx
import collections
import numpy as np


#todo:
"""
For All the drivable roads inside a junction, create sets of waypoints.


Then, once car is inside a junction, calculate shortest waypoint in EACH road, and calculate the
longitudinal distance from that waypoint to the other car. The waypoint that has the minimum value will
be the road to consider. Once road is known, find a more finegrained WP to pinpoint distance.

This should work even if both cars are in the same junction. :)

"""

class OpenDriveMap:
    def __init__(self, od_filepath, carla_map):
        self.od_filepath = od_filepath
        self.carla_map = carla_map

        self.tree = ET.parse(od_filepath)
        self.root = self.tree.getroot()
        self.map_elements = list(self.root) # includes 1 'header' multiple 'road' 'controller' 'junction'
        self.road_elements = self.root.findall('road')
        self.road_elements = [road for road in self.road_elements if self.is_drivable(road)]
        self.road_ids = [r.get('id') for r in self.road_elements]
        self.junction_elements = self.root.findall('junction')
        # print(f'{od_map} has {len(road_elements)} roads and {len(junction_elements)} junctions')
        self.nonjunc_roads = [road for road in self.road_elements if road.attrib['junction'] == '-1']
        self.waypoints = self.carla_map.generate_waypoints(0.5)  # waypoints spaced every X meters
        self.create_id_dicts()
        self.topology = self.minimum_topology()
        self.full_topology = self.road_network_topology()

    def create_id_dicts(self):
        """
        Creates a set of useful lookup tables where the key is the string id of the road/junction
        """
        # Contains All junctions
        self.id_dict_junction_elements = {j.get('id'): j for j in self.junction_elements}
        # Contains Roads which are not in a junction
        self.id_dict_nonjunc_roads = {r.get('id'): r for r in self.nonjunc_roads}
        # Contains all Roads
        self.id_dict_road_elements = {r.get('id'): r for r in self.road_elements}
        # Contains list of Roads which are inside a Junction
        self.id_dict_roads_injunc = {j.get('id'): \
                                           [c.get('connectingRoad') \
                                            for c in j.findall('connection') \
                                            if c.get('connectingRoad') in self.road_ids] \
                                     for j in self.junction_elements}
        # Contains list of waypoints inside junctions
        self.id_dict_junc_wp = collections.defaultdict(list)
        for waypoint in self.waypoints:
            if waypoint.is_junction:
                self.id_dict_junc_wp[f"{waypoint.road_id}"].append(waypoint)

    def is_drivable(self, road):
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

    def road_network_topology(self):
        """
        excludes junctions as separate elements
        only roads are in graph
        """
        graph = nx.DiGraph()
        for road in self.road_elements:
            road_id = road.get('id')
            links = road.find('link')
            for link in links:
                if link.get('elementType')=='road':
                    link_id = link.get('elementId')
                    graph.add_edge(f"{road_id}", f"{link_id}", distance=float(road.get('length')))
                    link_road = self.road_from_id(link_id)
                    graph.add_edge(f"{link_id}", f"{road_id}", distance=float(link_road.get('length')))
        return graph

    def minimum_topology(self):
        """
        uses only list of nonjunc_roads
        junctions are implied from the list of road successor/predecessor
        return: nx graph where nodes are roads or junctions

        assumption: The map has no Junction-Junction connections.
        I.e. when driving in a junction, all possible next roads are non-junc roads
        """
        graph_topology = nx.Graph()
        for road in self.nonjunc_roads:
            road_id = road.get('id')
            links = road.find('link')
            for link in links:
                l_type = link.get('elementType')[0]
                l_id = link.get('elementId')
                if link.tag == 'predecessor':  # only predecessor or successor are defined
                    graph_topology.add_edge(f'{l_type[0]}{l_id}', f'r{road_id}')
                else:
                    graph_topology.add_edge(f'r{road_id}', f'{l_type[0]}{l_id}')
        return graph_topology

    def _x_minimum_topology(self):
        """
        # Do Not Use
        Creates Minimum Topology from Carla Map
        Returns nx.Graph with the topology
        The topology nodes are (road,lane) so hard to use.
        """
        def wp_data(waypoint):
            return (waypoint.road_id, waypoint.lane_id)
        raw_topology = self.carla_map.get_topology()
        topology = []
        for edge in raw_topology:
            r0, l0 = wp_data(edge[0])
            r1, l1 = wp_data(edge[1])
            e = ((r0, l0), (r1, l1))
            topology.append(e)
            # topology.append((edge[0].road_id, edge[1].road_id))
        graph_topology = nx.Graph()
        graph_topology.add_edges_from(topology)
        return graph_topology

    def find_lane_direction(self, road, lane_id):
        for side in list(road.find('lanes').find('laneSection')):
            for lane in list(side):
                if lane.get('id') == lane_id:
                    return lane.find('userData').find('vectorLane').get('travelDir')

    def _longitudinal_dist_between_wp(self, waypoint1, waypoint2):
        """
        Assumes roads on road_list are actually connected! It will only sum the
        lengths of the roads and not check connectivity
        """
        road_id1, lane1, s1 = str(waypoint1.road_id), waypoint1.lane_id, waypoint1.s
        road_id2, lane2, s2 = str(waypoint2.road_id), waypoint2.lane_id, waypoint2.s
        shortest_path = nx.shortest_path_length(self.full_topology, source=road_id1,
                                                target=road_id2, weight="distance")
        if shortest_path == 0:
            return abs(s1 - s2)
        road1 = self.road_from_id(road_id1)
        road2 = self.road_from_id(road_id2)
        if self.find_lane_direction(road1, str(lane1)) == 'forward':
            shortest_path -= s1
        elif self.find_lane_direction(road1, str(lane1)) == 'backward':
            shortest_path += s1
            shortest_path -= float(road1.get('length'))
        if self.find_lane_direction(road2, str(lane2)) == 'forward':
            shortest_path += s2
            shortest_path -= float(road2.get('length'))
        elif self.find_lane_direction(road2, str(lane2)) == 'backward':
            shortest_path += s2
        return shortest_path

    def _x_old_long_dist(self):
        """
        road1_dist = s1
        road2_dist = s2
        if self.find_lane_direction(road1, str(lane1)) == 'forward':
            road1_dist *= -1
            road1_dist += float(road_list[0].get('length'))
        if self.find_lane_direction(road_list[-1], str(lane2)) == 'backward':
            road2_dist *= -1
            road2_dist += float(road_list[-1].get('length'))
        intermediate_dist = 0
        for road in road_list[1:-1]:
            intermediate_dist += float(road.get('length'))
        return road1_dist + road2_dist + intermediate_dist
        """
        pass

    def road_from_id(self, road_id):
        """
        Returns opendrive road objects that has that id
        """
        ii = self.road_ids.index(str(road_id))
        return self.road_elements[ii]

    def road_and_lane_graph_shortest_path(self, roadlane1, roadlane2):
        """
        roadlane = (vehicle_wp.road_id, vehicle_wp.lane_id)   tuples
        """
        return nx.shortest_path(self.carla_topology, roadlane1, roadlane2)

    def feasible_waypoints(self, waypoint, tolerance=0.9):
        """
        Given a waypoint that is in a junction,
        returns list of other waypoints which might also be feasible.
        I.e. the car might be on the other road, due to overlapping definitions
        """
        # if the waypoint is on a nonjunction road
        if not waypoint.is_junction:
            return [waypoint]
        else:
            feasible_waypoints = []
            juncroad_ids = self.id_dict_roads_injunc[str(waypoint.junction_id)]
            for road_id in juncroad_ids:
                distances = [self.waypoint_distance(waypoint, wp) for wp in self.id_dict_junc_wp[road_id]]
                dd = np.argmin(distances)
                if distances[dd] <= tolerance:
                    feasible_waypoints.append(self.id_dict_junc_wp[road_id][dd])
            assert len(feasible_waypoints)>0
            return feasible_waypoints

    def waypoint_coordinates(self, waypoint):
        return np.array([waypoint.transform.location.x,
                         waypoint.transform.location.y,
                         waypoint.transform.location.z])

    def waypoint_distance(self, waypoint1, waypoint2):
        return np.linalg.norm(self.waypoint_coordinates(waypoint1) - self.waypoint_coordinates(waypoint2))

    def longitudinal_road_distance(self, v1_loc, v2_loc):
        """
        v_location = vehicle.get_location() object from Carla
        Project to waypoint - check feasible waypoints (if in junction)
        """
        wp1, wp2 = self.carla_map.get_waypoint(v1_loc), self.carla_map.get_waypoint(v2_loc)
        feasible_wp1 = self.feasible_waypoints(wp1)
        feasible_wp2 = self.feasible_waypoints(wp2)
        distance = 10*self.waypoint_distance(wp1, wp2)
        for f1 in feasible_wp1:
            for f2 in feasible_wp2:
                route_distance = self._longitudinal_dist_between_wp(f1, f2)
                if route_distance < distance:
                    distance = route_distance
        return distance


'''
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

    """
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
    """
    intermediate_dist = 0
    for road in road_list[1:-1]:
        intermediate_dist += float(road.get('length'))
    return road1_dist + road2_dist + intermediate_dist
'''