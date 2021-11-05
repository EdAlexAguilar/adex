"""
Implements Simple Jerk Monitor - Needs processed map to decompose accels
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

class ComfortMonitor:
    """
    1 Monitor for every other actor involved
    """
    def __init__(self, ego_id):
        self.ego = ego_id # ego vehicle
        self.timestamps = []
        self.ego_long_acceleration_trace = []
        self.ego_lat_acceleration_trace = []

    def update(self, timestamp, actor_status):
        self.timestamps.append(timestamp)
        ego_dict = actor_status[self.ego]
        ego_orientation = self.get_car_orientation(ego_dict)
        ego_acc_long, ego_acc_lat = self.get_longlat_acceleration(ego_dict, ego_orientation)
        self.ego_long_acceleration_trace.append(ego_acc_long)
        self.ego_lat_acceleration_trace.append(ego_acc_lat)

    def get_car_orientation(self, actor_dict):
        vel = self.vec3D_np(to_carla_vector3d(actor_dict["velocity"]))
        orientation = vel / np.linalg.norm(vel)
        return orientation

    def vector3D_longlat(self, vec, orientation):
        v = self.vec3D_np(vec)
        ori = orientation
        long = v @ ori
        v_lat = v - long*ori
        lat = np.linalg.norm(v_lat)
        return long, lat

    def vec3D_np(self, vec):
        return np.array([vec.x, vec.y, vec.z])

    def derivative(self, f, k, delta):
        """ careful, there are no assertions"""
        return (f[k-2] - 8*f[k-1] + 8*f[k+1] - f[k+2]) / (12 * delta)

    def get_longlat_acceleration(self, vehicle, orientation):
        acc = to_carla_vector3d(vehicle["acceleration"])
        a_long, a_lat = self.vector3D_longlat(acc, orientation)
        return a_long, a_lat

    def trace_derivative(self, trace, delta=0.05):
        D_trace = [0,0]
        for k in range(2, len(trace)-2):
            D_trace.append(self.derivative(trace, k, delta))
        D_trace.append(0)
        D_trace.append(0)
        return D_trace
