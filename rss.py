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
        self.long_distance_trace = []
        self.safe_long_distance_trace = []
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

    def update_dynamic_variables(self):
        """
        Once per Step, makes some Carla API calls to update
        dynamic variables of actors involved
        """
        self.ego_transform = self.ego.get_transform()
        self.ego_location = self.ego_transform.location
        self.ego_forward = self.ego_transform.rotation.get_forward_vector()
        self.ego_right = self.ego_transform.rotation.get_right_vector()
        self.ego_velocity = self.ego.get_velocity()
        self.ego_acceleration = self.ego.get_acceleration()
        self.actor_transform = self.actor.get_transform()
        self.actor_location = self.actor_transform.location
        self.actor_forward = self.actor_transform.rotation.get_forward_vector()
        self.actor_velocity = self.actor.get_velocity()
        self.actor_acceleration = self.actor.get_acceleration()

    def update(self):
        # Start by updating dynamic variables
        self.update_dynamic_variables()
        # Calculate Variables of Interest
        long_dist = self.get_long_distance()
        lat_dist = self.get_lat_distance()
        ego_v_long, ego_v_lat = self.longlat_roadprojection(self.ego_location, self.ego_velocity)
        ego_acc_long, ego_acc_lat = self.longlat_roadprojection(self.ego_location, self.ego_acceleration)
        actor_v_long, actor_v_lat = self.longlat_roadprojection(self.actor_location, self.actor_velocity)
        actor_acc_long, actor_acc_lat = self.longlat_roadprojection(self.actor_location, self.actor_acceleration)
        if self.actor_in_front():
            safe_long_d = self.safe_long_dist(ego_v_long, actor_v_long)
        else:
            safe_long_d = 0
        # update the traces with all variables
        self.long_distance_trace.append(long_dist)
        self.safe_long_distance_trace.append(safe_long_d)
        self.ego_long_velocity_trace.append(ego_v_long)
        self.actor_long_velocity_trace.append(actor_v_long)
        self.ego_long_acceleration_trace.append(ego_acc_long)
        self.actor_long_acceleration_trace.append(actor_acc_long)
        self.lat_distance_trace.append(lat_dist)
        self.ego_lat_velocity_trace.append(ego_v_lat)
        self.actor_lat_velocity_trace.append(actor_v_lat)
        self.ego_lat_acceleration_trace.append(ego_acc_lat)
        self.actor_lat_acceleration_trace.append(actor_acc_lat)


    def get_current_variables(self, elapsed_time):
        """
        Used for Real-Time Monitoring
        """
        vars_dict = {
            "elapsed_time": np.array(elapsed_time),
            "long_dist": np.array(self.long_distance_trace[-1]),
            "ego_v_long": np.array(self.ego_long_velocity_trace[-1]),
            "actor_v_long": np.array(self.actor_long_velocity_trace[-1]),
            "ego_a_long": np.array(self.ego_long_acceleration_trace[-1]),
            "actor_a_long": np.array(self.actor_long_acceleration_trace[-1]),
            "lat_dist": np.array(self.lat_distance_trace[-1]),
            "ego_v_lat": np.array(self.ego_lat_velocity_trace[-1]),
            "actor_v_lat": np.array(self.actor_lat_velocity_trace[-1]),
            "ego_a_lat": np.array(self.ego_lat_acceleration_trace[-1]),
            "actor_a_lat": np.array(self.actor_lat_acceleration_trace[-1])
        }
        return vars_dict

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

    def actor_in_front(self):
        """
        Returns True if Actor is ahead of Ego
        """
        if self.ego_forward.dot(self.actor_location - self.ego_location) > 0:
            return True
        else:
            return False

    def actor_in_right(self):
        """
        Returns True if Actor is to the right of Ego
        """
        if self.ego_right.dot(self.actor_location - self.ego_location) > 0:
            return True
        else:
            return False

    def print_vehicle_road_info(self, vehicle):
        """Prints carla.Vehicle object road information"""
        vehicle_wp = self.processed_map.carla_map.get_waypoint(vehicle.get_location())
        print(f"Road ID: {vehicle_wp.road_id}  Lane ID: {vehicle_wp.lane_id}"
              f"  S: {vehicle_wp.s}  Section ID: {vehicle_wp.section_id}")

    def get_long_distance(self):
        if self.actor_in_front() is False:
            return 0.0
        dist = self.processed_map.longitudinal_road_distance(self.ego_location, self.actor_location)
        # todo: direction of travel of both roads might affect this?
        dist -= (self.ego_length + self.actor_length)
        return max(dist, 0)

    def get_lat_distance(self):
        """
        # this assumes car is facing forward, but is just rigidly displaced
        # typically steering angles are small enough for this assumption to be correct
        # furthermore, the bounding boxes are slightly bigger than the vehicles
        """
        if self.actor_in_right():
            dist = self.processed_map.lateral_road_distance(self.ego_location, self.actor_location)
        else:
            dist = self.processed_map.lateral_road_distance(self.actor_location, self.ego_location)
        dist -= (self.ego_width + self.actor_width)
        return max(dist, 0)

    def get_wp_offset(self, location):
        wp_location = self.processed_map.carla_map.get_waypoint(location).transform.location
        road_orientation = self.processed_map.location_orientation(location)
        offset = location - wp_location
        offset_long, offset_lat = self.vector3D_longlat(offset, road_orientation)
        return offset_long, offset_lat

    def longlat_roadprojection(self, location, vector):
        """
        location is of vehicle
        vector is e.g. vehicle velocity or vehicle accel
        """
        road_wp_rotation = self.processed_map.carla_map.get_waypoint(location).transform.rotation
        road_forward = road_wp_rotation.get_forward_vector()
        road_right = road_wp_rotation.get_right_vector()
        vec_long = road_forward.dot(vector)
        vec_lat = road_right.dot(vector)
        return vec_long, vec_lat

    def vector3D_longlat(self, vec, orientation):
        v = self.vec3D_np(vec)
        ori = self.vec3D_np(orientation)
        long = np.abs(v @ ori)
        v_lat = v - long*ori
        lat = np.linalg.norm(v_lat)
        return long, lat

    def vec3D_np(self, vec):
        return np.array([vec.x, vec.y, vec.z])
