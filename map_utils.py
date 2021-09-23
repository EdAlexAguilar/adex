"""
OpenDriveMap class
Contains methods to calculate "longitudinal" lane distance between two points of a map
"""
import xml.etree.ElementTree as ET
import networkx as nx


class OpenDriveMap:
    def __init__(self, od_filepath, carla_map):
        self.od_filepath = od_filepath
        self.carla_map = carla_map

        self.tree = ET.parse(od_filepath)
        self.root = self.tree.getroot()
        self.map_elements = list(self.root) # includes 1 'header' multiple 'road' 'controller' 'junction'
        self.road_elements = self.root.findall('road')
        self.road_elements = [road for road in self.road_elements if self.is_drivable(road)]
        self.junction_elements = self.root.findall('junction')
        # print(f'{od_map} has {len(road_elements)} roads and {len(junction_elements)} junctions')
        self.nonjunc_roads = [road for road in self.road_elements if road.attrib['junction'] == '-1']

        self.carla_topology = self.minimum_topology()

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

    def minimum_topology(self):
        """
        Creates Minimum Topology from Carla Map
        Returns nx.Graph with the topology
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

    def longitudinal_dist(self, s1, lane1, s2, lane2, road_list):
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
        if self.find_lane_direction(road_list[0], str(lane1)) == 'forward':
            road1_dist *= -1
            road1_dist += float(road_list[0].get('length'))
        if self.find_lane_direction(road_list[-1], str(lane2)) == 'backward':
            road2_dist *= -1
            road2_dist += float(road_list[-1].get('length'))
        intermediate_dist = 0
        for road in road_list[1:-1]:
            intermediate_dist += float(road.get('length'))
        return road1_dist + road2_dist + intermediate_dist

    def roads_from_id(self, road_id_list):
        """
        List of road_ids
        Returns list of opendrive road objects in that order
        """
        roads = []
        r_ids = [r.get('id') for r in self.road_elements]
        for id in road_id_list:
            ii = r_ids.index(str(id))
            roads.append(self.road_elements[ii])
        return roads

    def road_and_lane_graph_shortest_path(self, roadlane1, roadlane2):
        """
        roadlane = (vehicle_wp.road_id, vehicle_wp.lane_id)   tuples
        """
        return nx.shortest_path(self.carla_topology, roadlane1, roadlane2)


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