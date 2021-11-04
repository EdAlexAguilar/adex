"""
Implements Simple RSS Monitor in Carla
"""
import constants as c
import numpy as np

class RSSMonitor:
    """
    1 Monitor for every other actor involved
    """
    def __init__(self, ego, actor, processed_map):
        self.ego = ego # ego vehicle
        self.actor = actor # A SINGLE ACTOR
        self.processed_map = processed_map # Custom Map Object "OpenDriveMap'
        # self.straight_distance_trace = []
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
        self.ego_width = self.ego.bounding_box.extent.y
        self.ego_length = self.ego.bounding_box.extent.x
        self.actor_width = self.actor.bounding_box.extent.y
        self.actor_length = self.actor.bounding_box.extent.x

    def update(self):
        long_dist = self.get_long_distance()
        lat_dist = self.get_lat_distance()
        ego_v_long, ego_v_lat = self.get_longlat_velocity(self.ego)
        ego_acc_long, ego_acc_lat = self.get_longlat_acceleration(self.ego)
        actor_v_long, actor_v_lat = self.get_longlat_velocity(self.actor)
        actor_acc_long, actor_acc_lat = self.get_longlat_acceleration(self.actor)
        self.long_distance_trace.append(long_dist)
        self.ego_long_velocity_trace.append(ego_v_long)
        self.actor_long_velocity_trace.append(actor_v_long)
        self.ego_long_acceleration_trace.append(ego_acc_long)
        self.actor_long_acceleration_trace.append(actor_acc_lat)
        self.lat_distance_trace.append(lat_dist)
        self.ego_lat_velocity_trace.append(ego_acc_long)
        self.actor_lat_velocity_trace.append(actor_acc_long)
        self.ego_lat_acceleration_trace.append(ego_acc_lat)
        self.actor_lat_acceleration_trace.append(actor_acc_lat)

    def reset_monitor(self):
        self.distance_trace = []

    def safe_long_dist(self, v_behind, v_front):
        tau = c.REACTION_TIME
        acc_max = c.ACCEL_MAX
        brake_min = c.BRAKE_MIN
        brake_max = c.BRAKE_MAX
        d_safe = v_behind * tau + 0.5 * acc_max * tau ** 2 + (v_behind + tau * acc_max) ** 2 / (
                    2 * brake_min) - v_front ** 2 / (2 * brake_max)
        return d_safe

    def get_vehicle_location(self, vehicle):
        """
        Given a carla.Vehicle object, returns
        road_id, lane_id, s, section_id
        """
        vehicle_loc = vehicle.get_location()
        return vehicle_loc
        # vehicle_wp = self.processed_map.carla_map.get_waypoint(vehicle_loc)  # has project_to_road=True so wp is at center of closest lane
        # return vehicle_wp
        # return vehicle_wp.road_id, vehicle_wp.lane_id, vehicle_wp.s, vehicle_wp.section_id

    def print_vehicle_road_info(self, vehicle):
        """Prints carla.Vehicle object road information"""
        vehicle_wp = self.processed_map.carla_map.get_waypoint(vehicle.get_location())
        print(f"Road ID: {vehicle_wp.road_id}  Lane ID: {vehicle_wp.lane_id}"
              f"  S: {vehicle_wp.s}  Section ID: {vehicle_wp.section_id}")

    def get_long_distance(self):
        ego_location = self.get_vehicle_location(self.ego)
        actor_location = self.get_vehicle_location(self.actor)
        dist = self.processed_map.longitudinal_road_distance(ego_location, actor_location)
        #todo: direction of travel of both roads might affect this?
        ego_offset_long, _ = self.get_wp_offset(ego_location)
        actor_offset_long, _ = self.get_wp_offset(actor_location)
        dist -= (self.ego_length + self.actor_length)
        return dist

    def get_lat_distance(self):
        ego_location = self.get_vehicle_location(self.ego)
        actor_location = self.get_vehicle_location(self.actor)
        dist = self.processed_map.lateral_road_distance(ego_location, actor_location)
        _, ego_offset_lat = self.get_wp_offset(ego_location)
        _, actor_offset_lat = self.get_wp_offset(actor_location)
        #todo: this assumes car is facing forward, but is just rigidly displaced
        dist -= (self.ego_width + self.actor_width + ego_offset_lat + actor_offset_lat)
        return dist

    def get_wp_offset(self, location):
        wp_location = self.processed_map.carla_map.get_waypoint(location).transform.location
        road_orientation = self.processed_map.location_orientation(location)
        offset = location - wp_location
        offset_long, offset_lat = self.vector3D_longlat(offset, road_orientation)
        return offset_long, offset_lat

    def get_longlat_velocity(self, vehicle):
        vel = vehicle.get_velocity()
        veh_loc = self.get_vehicle_location(vehicle)
        road_orientation = self.processed_map.location_orientation(veh_loc)
        v_long, v_lat = self.vector3D_longlat(vel, road_orientation)
        return v_long, v_lat

    def get_longlat_acceleration(self, vehicle):
        acc = vehicle.get_acceleration()
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
