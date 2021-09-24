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

    def update(self):
        dist, min_path = self.get_long_distances()
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

    def get_vehicle_road_info(self, vehicle):
        """
        Given a carla.Vehicle object, returns
        road_id, lane_id, s, section_id
        """
        vehicle_loc = vehicle.get_location()
        vehicle_wp = self.processed_map.carla_map.get_waypoint(vehicle_loc)  # has project_to_road=True so wp is at center of closest lane
        return vehicle_wp.road_id, vehicle_wp.lane_id, vehicle_wp.s, vehicle_wp.section_id

    def print_vehicle_road_info(self, vehicle):
        """Prints carla.Vehicle object road informarion"""
        road_id, lane_id, s, section_id = self.get_vehicle_road_info(vehicle)
        print(f"Road ID: {road_id}  Lane ID: {lane_id}  S: {s}  Section ID: {section_id}")

    def get_long_distances(self):
        ego_road, ego_lane, ego_s, _ = self.get_vehicle_road_info(self.ego)
        ego_loc = (ego_road, ego_lane)
        distances = []
        for vehicle in self.actors:
            v_road, v_lane, v_s, _ = self.get_vehicle_road_info(vehicle)
            v_loc = (v_road, v_lane)

            # todo: Pass all of this map processing to map_utils to have a single call

            min_path = self.processed_map.road_and_lane_graph_shortest_path(ego_loc, v_loc)
            min_path = [r[0] for r in min_path] # takes road ids from min_path
            min_path_roads = self.processed_map.roads_from_id(min_path)

            dist = self.processed_map.longitudinal_dist(ego_s, ego_lane, v_s, v_lane, min_path_roads)

            distances.append(dist)
        return distances, min_path


#todo: offset car dimensions (1/2)car1_length , (1/2)car2_length

def vector_magnitude(vec):
    return np.sqrt(vec.x**2 + vec.y**2 + vec.z**2)