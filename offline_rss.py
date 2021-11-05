"""
Implements Simple RSS Monitor in Carla
"""

import numpy as np

import sys
import glob

try:
    sys.path.append(
        glob.glob('C:\CARLA\CARLA_0.9.11\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.11-py3.7-win-amd64.egg')[0])
except IndexError:
    pass
import carla


def to_carla_location(dict):
    return carla.Location(x=dict['x'], y=dict['y'], z=dict['z'])


def to_carla_vector3d(dict):
    return carla.Vector3D(x=dict['x'], y=dict['y'], z=dict['z'])

class RSSOfflineMonitor:
    """
    1 Monitor for every other actor involved
    """
    def __init__(self, ego_id, actor_id, processed_map):
        self.ego = ego_id # ego vehicle
        self.actor = actor_id # A SINGLE ACTOR
        self.processed_map = processed_map # Custom Map Object "OpenDriveMap'
        # self.straight_distance_trace = []
        self.timestamps =[]
        self.long_distance_trace = []
        self.ego_long_velocity_trace = []
        self.actor_long_velocity_trace = []
        self.ego_long_acceleration_trace = []
        self.actor_long_acceleration_trace = []
        self.lat_distance_trace = []
        self.ego_lat_velocity_trace = []
        self.actor_lat_velocity_trace = []
        self.ego_lat_acceleration_trace = []
        self.actor_lat_acceleration_trace = []
        self.get_vehicle_dimensions()  # Half width, and Half lengths

    def get_vehicle_dimensions(self):
        """
        In fact widths and lengths are **HALF** of the value.
        """
        self.ego_width = 0.5 #self.ego.bounding_box.extent.y
        self.ego_length = 1.5 #self.ego.bounding_box.extent.x
        self.actor_width = 0.5 #self.actor.bounding_box.extent.y
        self.actor_length = 1.5 #self.actor.bounding_box.extent.x

    def update(self, timestamp, actor_status):
        self.timestamps.append(timestamp)
        ego_dict = actor_status[self.ego]
        actor_dict = actor_status[self.actor]
        long_dist = self.get_long_distance(ego_dict, actor_dict)
        lat_dist = self.get_lat_distance(ego_dict, actor_dict)
        ego_v_long, ego_v_lat = self.get_longlat_velocity(ego_dict)
        ego_acc_long, ego_acc_lat = self.get_longlat_acceleration(ego_dict)
        actor_v_long, actor_v_lat = self.get_longlat_velocity(actor_dict)
        actor_acc_long, actor_acc_lat = self.get_longlat_acceleration(actor_dict)
        self.long_distance_trace.append(long_dist)
        self.ego_long_velocity_trace.append(ego_v_long)
        self.actor_long_velocity_trace.append(actor_v_long)
        self.ego_long_acceleration_trace.append(ego_acc_long)
        self.actor_long_acceleration_trace.append(actor_acc_long)
        self.lat_distance_trace.append(lat_dist)
        self.ego_lat_velocity_trace.append(ego_v_lat)
        self.actor_lat_velocity_trace.append(actor_v_lat)
        self.ego_lat_acceleration_trace.append(ego_acc_lat)
        self.actor_lat_acceleration_trace.append(actor_acc_lat)


    def get_vehicle_location(self, vehicle):
        """
        Given a carla.Vehicle object, returns
        road_id, lane_id, s, section_id
        """
        vehicle_loc = to_carla_location(vehicle["location"])
        return vehicle_loc
        # vehicle_wp = self.processed_map.carla_map.get_waypoint(vehicle_loc)  # has project_to_road=True so wp is at center of closest lane
        # return vehicle_wp
        # return vehicle_wp.road_id, vehicle_wp.lane_id, vehicle_wp.s, vehicle_wp.section_id

    def print_vehicle_road_info(self, vehicle):
        """Prints carla.Vehicle object road information"""
        vehicle_wp = self.processed_map.carla_map.get_waypoint(vehicle.get_location())
        print(f"Road ID: {vehicle_wp.road_id}  Lane ID: {vehicle_wp.lane_id}"
              f"  S: {vehicle_wp.s}  Section ID: {vehicle_wp.section_id}")

    def get_long_distance(self, ego, actor):
        ego_location = self.get_vehicle_location(ego)
        actor_location = self.get_vehicle_location(actor)
        dist = self.processed_map.longitudinal_road_distance(ego_location, actor_location)
        #todo: direction of travel of both roads might affect this?
        ego_offset_long, _ = self.get_wp_offset(ego_location)
        actor_offset_long, _ = self.get_wp_offset(actor_location)
        dist -= (self.ego_length + self.actor_length)
        return dist

    def get_lat_distance(self, ego, actor):
        ego_location = self.get_vehicle_location(ego)
        actor_location = self.get_vehicle_location(actor)
        dist = self.processed_map.lateral_road_distance(ego_location, actor_location)
        _, ego_offset_lat = self.get_wp_offset(ego_location)
        _, actor_offset_lat = self.get_wp_offset(actor_location)
        #todo: this assumes car is facing forward, but is just rigidly displaced
        dist -= (self.ego_width + self.actor_width + ego_offset_lat + actor_offset_lat)
        return dist

    def get_wp_offset(self, location):
        wp_location = self.processed_map.carla_map.get_waypoint(location, project_to_road=False).transform.location
        road_orientation = self.processed_map.location_orientation(location)
        offset = location - wp_location
        offset_long, offset_lat = self.vector3D_longlat(offset, road_orientation)
        return offset_long, offset_lat

    def get_longlat_velocity(self, vehicle):
        vel = to_carla_vector3d(vehicle["velocity"])
        veh_loc = self.get_vehicle_location(vehicle)
        road_orientation = self.processed_map.location_orientation(veh_loc)
        v_long, v_lat = self.vector3D_longlat(vel, road_orientation)
        return v_long, v_lat

    def get_longlat_acceleration(self, vehicle):
        acc = to_carla_vector3d(vehicle["acceleration"])
        veh_loc = self.get_vehicle_location(vehicle)
        road_orientation = self.processed_map.location_orientation(veh_loc)
        a_long, a_lat = self.vector3D_longlat(acc, road_orientation)
        return a_long, a_lat

    def vector3D_longlat(self, vec, orientation):
        v = self.vec3D_np(vec)
        ori = self.vec3D_np(orientation)
        long = np.abs(v @ ori)
        v_lat = v - long*ori
        lat = np.linalg.norm(v_lat)
        return long, lat

    def vec3D_np(self, vec):
        return np.array([vec.x, vec.y, vec.z])

    def straight_line_distance(self):
        pass
