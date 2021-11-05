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

class GeometricRSSOfflineMonitor:
    """
    1 Monitor for every other actor involved
    """
    def __init__(self, ego_id, actor_id, processed_map):
        self.ego = ego_id # ego vehicle
        self.actor = actor_id # A SINGLE ACTOR
        self.processed_map = processed_map # Custom Map Object "OpenDriveMap'
        # self.straight_distance_trace = []
        self.timestamps = []
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
        self.actor_angle_trace = []
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
        ego_orientation = self.get_car_orientation(ego_dict)
        actor_dict = actor_status[self.actor]
        ego_v_long, ego_v_lat = np.linalg.norm(self.vec3D_np(to_carla_vector3d(actor_dict["velocity"]))), 0
        ego_acc_long, ego_acc_lat = self.get_longlat_acceleration(ego_dict, ego_orientation)
        actor_acc_long, actor_acc_lat = self.get_longlat_acceleration(actor_dict, ego_orientation)
        actor_v_long, actor_v_lat = self.get_longlat_velocity(actor_dict, ego_orientation)
        long_dist, lat_dist = self.get_longlat_distance(ego_dict, actor_dict, ego_orientation)
        angle = self.get_actor_angle(actor_dict, ego_orientation)
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
        self.actor_angle_trace.append(angle)

    def get_car_orientation(self, actor_dict):
        vel = self.vec3D_np(to_carla_vector3d(actor_dict["velocity"]))
        orientation = vel / np.linalg.norm(vel)
        return orientation

    def get_actor_angle(self, actor, ego_orientation):
        actor_orientation = self.get_car_orientation(actor)
        actor_angle = np.arctan2(actor_orientation[1], actor_orientation[0])
        ego_angle = np.arctan2(ego_orientation[1], ego_orientation[0])
        return ego_angle - actor_angle

    def get_vehicle_location(self, vehicle):
        vehicle_loc = to_carla_location(vehicle["location"])
        return vehicle_loc

    def get_longlat_distance(self, ego, actor, orientation):
        ego_loc = self.vec3D_np(self.get_vehicle_location(ego))
        actor_loc = self.vec3D_np(self.get_vehicle_location(actor))
        distance = actor_loc - ego_loc
        d_long = distance @ orientation
        d_lat = np.linalg.norm(distance - d_long*orientation)
        d_long -= self.ego_length - 1
        d_lat -= self.ego_width -1
        return d_long, d_lat

    def get_longlat_velocity(self, vehicle, orientation):
        vel = to_carla_vector3d(vehicle["velocity"])
        v_long, v_lat = self.vector3D_longlat(vel, orientation)
        return v_long, v_lat

    def get_longlat_acceleration(self, vehicle, orientation):
        acc = to_carla_vector3d(vehicle["acceleration"])
        a_long, a_lat = self.vector3D_longlat(acc, orientation)
        return a_long, a_lat

    def vector3D_longlat(self, vec, orientation):
        v = self.vec3D_np(vec)
        ori = orientation
        long = v @ ori
        v_lat = v - long*ori
        lat = np.linalg.norm(v_lat)
        return long, lat

    def vec3D_np(self, vec):
        return np.array([vec.x, vec.y, vec.z])