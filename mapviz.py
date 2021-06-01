import xml.etree.ElementTree as ET
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from collections import namedtuple, defaultdict

JunctionLane = namedtuple('JunctionLane', ['id_lane', 'pred_lane', 'succ_lane', 'travel_dir'])
JunctionRoad = namedtuple('JunctionRoad', ['id_road', 'pred_road', 'pred_contact', 'succ_road', 'succ_contact', 'turn_relation'])


od_map = 'OpenDriveMaps/hotspot1.xodr'


def get_road_links(road):
    """
    Returns dictionary of predecessor and succesor properties of a road
    If road does not have a pred or succ, then one will be virtually created with the same road_id
    """
    links = road.find('link')
    if len(links)==2:
        pred = links.find('predecessor')
        succ = links.find('successor')
        link_info = {f'pred_{k}': v for k, v in pred.attrib.items()}
        link_info.update({f'succ_{k}': v for k, v in succ.attrib.items()})
    else:
        # Road leads to infinity
        if links[0].tag=='successor':
            # Road has no predecessor
            succ = links[0]
            pred = {'elementType': 'road', 'elementId': road.attrib['id']}
            link_info = {f'pred_{k}': v for k, v in pred.items()}
            link_info.update({f'succ_{k}': v for k, v in succ.attrib.items()})
        elif links[0].tag=='predecessor':
            # Road has no successor
            pred = links[0]
            succ = {'elementType': 'road', 'elementId': road.attrib['id']}
            link_info = {f'pred_{k}': v for k, v in pred.attrib.items()}
            link_info.update({f'succ_{k}': v for k, v in succ.items()})
        else:
            raise KeyError("Road has no successor and no predecessor!")
    return link_info


def create_road_topology(nj_roads):
    """
    input: list of nonjunction_roads
    output nx graph and edge_names dict
    """
    G = nx.Graph()
    road_names = {}
    for road in nj_roads:
        road_link = get_road_links(road)
        road_id = road.attrib['id']
        if road_link['pred_elementType'] == 'road':
            if int(road_id)<int(road_link['pred_elementId']):
                in_node = f'R{road_id}R{road_link["pred_elementId"]}'
            elif int(road_id)>int(road_link['pred_elementId']):
                in_node = f'R{road_link["pred_elementId"]}R{road_id}'
            else:
                in_node = f'R{road_id}'
        if road_link['pred_elementType'] == 'junction':
            in_node = f'J{road_link["pred_elementId"]}'
        if road_link['succ_elementType'] == 'road':
            if int(road_id)<int(road_link['succ_elementId']):
                out_node = f'R{road_id}R{road_link["succ_elementId"]}'
            elif int(road_id)>int(road_link['succ_elementId']):
                out_node = f'R{road_link["succ_elementId"]}R{road_id}'
            else:
                out_node = f'R{road_id}'
        if road_link['succ_elementType'] == 'junction':
            out_node = f'J{road_link["succ_elementId"]}'
        G.add_edge(in_node, out_node)
        road_names.update({(in_node, out_node): road.attrib['name']})
    return G, road_names


def create_road_topology2(nonjunction_roads):
    """
    nonjunction_roads: list of non-junction road elements
    junctions are implied from the list of road successor/predecessor
    return: nx directed graph where nodes are roads or junctions
    """
    #G = nx.DiGraph()
    G = nx.Graph()
    for road in nonjunction_roads:
        road_id = road.get('id')
        links = road.find('link')
        for link in links:
            l_type = link.get('elementType')[0]
            l_id = link.get('elementId')
            if link.tag == 'predecessor':
                G.add_edge(f'{l_type[0]}{l_id}',f'r{road_id}')
            else:
                G.add_edge(f'r{road_id}', f'{l_type[0]}{l_id}')
    return G



def get_junction_road_info(junction_road):
    """
        Input is Opendrive Road Element that is inside junction
    """
    assert junction_road.get('junction') != '-1'
    id = junction_road.get('id')
    link = junction_road.find('link')
    pred = link.find('predecessor').get('elementId')
    p_contact = link.find('predecessor').get('contactPoint')
    succ = link.find('successor').get('elementId')
    s_contact = link.find('successor').get('contactPoint')
    turn_relation = 0 #junction_road.find('signals').find('signalReference').find('userData').find('vectorSignal').get('turnRelation')
    assert link.find('predecessor').get('elementType')=='road'
    assert link.find('successor').get('elementType')=='road'
    return JunctionRoad(id, pred, p_contact, succ, s_contact, turn_relation)

def get_junction_lanes(junction_road):
    """
    Input is Opendrive Road Element that is inside junction
    """
    assert junction_road.get('junction') != '-1'
    junctionlanes = []
    sections = list(junction_road.find('lanes').find('laneSection'))
    for section in sections:
        sect_lanes = section.findall('lane')
        for lane in sect_lanes:
            if lane.get('type')=='driving':
                id = lane.get('id')
                link = lane.find('link')
                pred = link.find('predecessor').get('id')
                succ = link.find('successor').get('id')
                travel_dir = lane.find('userData').find('vectorLane').get('travelDir')
                junctionlanes.append(JunctionLane(id, pred, succ, travel_dir))
    return junctionlanes

def get_roads_in_junction(junction_id, all_roads):
    """
        Given a junction_id, returns list of roads inside it
        Id must be string
    """
    roads = []
    for road in all_roads:
        if road.attrib['junction']==junction_id:
            roads.append(road)
    return roads

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

def longitudinal_dist(s1, s2, road_list):
    """
    s1, s2:  position of car1, car2 in their road
    road_list : car1 is at element 0, car2 at element -1 :: elements opendrive road

    Assumes roads on road_list are actually connected! It will only sum the
    lengths of the roads and not check connectivity
    """
    assert s1 <= float(road_list[0].get('length'))
    assert s2 <= float(road_list[-1].get('length'))
    if len(road_list)==1:
        return abs(s1-s2)
    road1_dist = s1
    road2_dist = s2
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
    intermediate_dist = 0
    for road in road_list[1:-1]:
        intermediate_dist += float(road.get('length'))
    return road1_dist + road2_dist + intermediate_dist

def roads_from_id(road_id_list):
    global road_elements
    roads = []
    r_ids = [r.get('id') for r in road_elements]
    for id in road_id_list:
        ii = r_ids.index(str(id))
        roads.append(road_elements[ii])
    return roads

tree = ET.parse(od_map)
root = tree.getroot()
map_elements = list(root) # includes 1 'header' multiple 'road' 'controller' 'junction'

road_elements = root.findall('road')
road_elements = [road for road in road_elements if is_drivable(road)]

junction_elements = root.findall('junction')

print(f'{od_map} has {len(road_elements)} roads and {len(junction_elements)} junctions')
# attrib

nonjunc_roads = [road for road in road_elements if road.attrib['junction']=='-1']

print(f'There are {len(nonjunc_roads)} Non-Junction Roads')


dJunction = {j.get('id'):j for j in junction_elements}
dNonJuncRoads = {r.get('id'):r for r in nonjunc_roads}
dRoads = {r.get('id'):r for r in road_elements}
dRoadInJunc = {j.get('id'):[c.get('connectingRoad') for c in j.findall('connection')] for j in junction_elements}

"""
if vehicle_road.get('junction')!=-1:
    X = dRoadInJunc[vehicle_road.get('junction')]
    X = [dRoads[junc_road] for junc_road in X]
    dist = min(list(map(DISTANCE_FUNC, X)))
"""

G = create_road_topology2(nonjunc_roads)

dEquivNonJuncRoads = defaultdict(list)
for road in nonjunc_roads:
    road_neighbors = [junct for junct in G.neighbors(f'r{road.get("id")}')]
    road_neighbors = tuple(np.sort(road_neighbors))
    dEquivNonJuncRoads[road_neighbors].append(f'r{road.get("id")}')
dEquivNonJuncRoads = dict(dEquivNonJuncRoads)



"""
nx.draw(G, with_labels=True)
print(G.edges())
# G, r_names = create_road_topology(nonjunc_roads)
# pos = nx.spring_layout(G)
# nx.draw(G, pos, with_labels=True)
# nx.draw_networkx_edge_labels(G, pos, r_names)
plt.show()
"""

"""
j400 = get_roads_in_junction('11', road_elements)

for r in j400:
    print(get_junction_road_info(r))
    print(get_junction_lanes(r))
    print(f'\n')
"""



"""
r0.find('lanes').find('laneSection').find('left').findall('lane')[2].find('userData').find('vect
orLane').get('travelDir')



Starting from a (road, lane), determine the set of future roads 'easily' accessible
the lane is used just to have a reference of 'forward'
Roads overlap inside junctions
So cannot just look naively at s-value. Need to know how long they overlap.

Starting from a (road, lane), need all possible "lateral" neighbors
(also from the rear?)
This implies extending longitudinal futures and looking at lateral


"""

