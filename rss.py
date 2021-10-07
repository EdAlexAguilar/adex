"""
Implements Simple RSS Monitor in Carla
Needs xodr file to work!
"""
import constants as c


class RSSMonitor:
    def __init__(self, ego, actors, processed_map):
        self.ego = ego # ego vehicle
        self.actors = actors # list of all other vehicles and actors to monitor
        self.processed_map = processed_map # Custom Map Object "OpenDriveMap'
        self.distance_trace = []
        self.straight_distance_trace = []

    def update(self):
        dist = self.get_long_distances()
        self.distance_trace.append(dist)

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
        """Prints carla.Vehicle object road informarion"""
        vehicle_wp = self.processed_map.carla_map.get_waypoint(vehicle.get_location())
        print(f"Road ID: {vehicle_wp.road_id}  Lane ID: {vehicle_wp.lane_id}"
              f"  S: {vehicle_wp.s}  Section ID: {vehicle_wp.section_id}")

    def get_long_distances(self):
        ego_location = self.get_vehicle_location(self.ego)
        distances = []
        for actor in self.actors:
            actor_location = self.get_vehicle_location(actor)
            dist = self.processed_map.longitudinal_road_distance(ego_location, actor_location)
            distances.append(dist)
        return distances

    def straight_line_distance(self):
        pass



#todo: offset car dimensions (1/2)car1_length , (1/2)car2_length

def vector_magnitude(vec):
    return np.sqrt(vec.x**2 + vec.y**2 + vec.z**2)