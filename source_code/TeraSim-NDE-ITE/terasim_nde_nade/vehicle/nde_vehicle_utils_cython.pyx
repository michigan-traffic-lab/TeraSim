from typing import List, Tuple
cimport cython
import numpy as np
cimport numpy as np
from shapely.geometry import LineString
from libc.math cimport sqrt, pow, sin, cos, atan2, M_PI
from scipy.interpolate import interp1d


def sumo_trajectory_to_normal_trajectory(np.ndarray[double, ndim=2] sumo_trajectory, double veh_length=5.0):
    cdef int n = sumo_trajectory.shape[0]
    cdef np.ndarray[double, ndim=2] normal_trajectory = np.empty((n, sumo_trajectory.shape[1]))
    cdef double rad

    for i in range(n):
        rad = (90 - sumo_trajectory[i, 2]) * M_PI / 180.0  # Convert to radians
        normal_trajectory[i, 2] = atan2(sin(rad), cos(rad))
        normal_trajectory[i, 0] = sumo_trajectory[i, 0] - veh_length / 2 * cos(normal_trajectory[i, 2])
        normal_trajectory[i, 1] = sumo_trajectory[i, 1] - veh_length / 2 * sin(normal_trajectory[i, 2])
        if sumo_trajectory.shape[1] > 3:
            normal_trajectory[i, 3:] = sumo_trajectory[i, 3:]
    return normal_trajectory


def get_circle_center_list(np.ndarray[double, ndim=1] traj_point, double veh_length, double tem_len):
    cdef double heading = traj_point[2]
    cdef double cos_heading = cos(heading)
    cdef double sin_heading = sin(heading)
    cdef double offset = veh_length / 2 - tem_len
    cdef np.ndarray[double, ndim=2] center_list = np.zeros((3, 2))

    center_list[0, 0] = traj_point[0] + offset * cos_heading
    center_list[0, 1] = traj_point[1] + offset * sin_heading
    center_list[1, 0] = traj_point[0]
    center_list[1, 1] = traj_point[1]
    center_list[2, 0] = traj_point[0] - offset * cos_heading
    center_list[2, 1] = traj_point[1] - offset * sin_heading

    return center_list

def collision_check(np.ndarray[double, ndim=2] traj1, np.ndarray[double, ndim=2] traj2, double veh_length, double tem_len, double circle_r):
    cdef int i, j, k
    cdef double dist, dx, dy
    cdef np.ndarray[double, ndim=2] center_list_1, center_list_2
    cdef np.ndarray[double, ndim=1] traj_point1, traj_point2

    traj1 = sumo_trajectory_to_normal_trajectory(traj1, veh_length)
    traj2 = sumo_trajectory_to_normal_trajectory(traj2, veh_length)

    for i in range(traj1.shape[0]):
        traj_point1 = traj1[i]
        traj_point2 = traj2[i]

        center_list_1 = get_circle_center_list(traj_point1, veh_length, tem_len)
        center_list_2 = get_circle_center_list(traj_point2, veh_length, tem_len)

        for j in range(center_list_1.shape[0]):
            for k in range(center_list_2.shape[0]):
                dx = center_list_1[j, 0] - center_list_2[k, 0]
                dy = center_list_1[j, 1] - center_list_2[k, 1]
                dist = sqrt(dx * dx + dy * dy)
                if dist <= 2 * circle_r:
                    return True, traj1[i, 3]

    return False, None

def interpolate_future_trajectory(np.ndarray[double, ndim=2] trajectory_list_array, double interpolate_resolution):
    cdef np.ndarray[double, ndim=1] time_values = trajectory_list_array[:, -1]
    cdef np.ndarray[double, ndim=2] position_values = trajectory_list_array[:, :-1]

    # Create the interpolation function
    interpolation_function = interp1d(time_values, position_values, axis=0, kind='linear')

    # Create the new time values
    cdef int num_points = int((time_values[-1] - time_values[0]) / interpolate_resolution) + 1
    cdef np.ndarray[double, ndim=1] new_time_values = np.linspace(time_values[0], time_values[-1], num_points)

    # Interpolate the position values
    cdef np.ndarray[double, ndim=2] new_position_values = interpolation_function(new_time_values)

    # Combine the new time and position values
    cdef np.ndarray[double, ndim=2] new_trajectory_list_array = np.hstack((new_position_values, new_time_values[:, None]))

    return new_trajectory_list_array

cpdef bint is_intersect(np.ndarray[double, ndim=2] trajectory1, np.ndarray[double, ndim=2] trajectory2, double veh_length, double tem_len, double circle_r):
    cdef np.ndarray[double, ndim=1] trajectory1_start = trajectory1[0, :2]
    cdef np.ndarray[double, ndim=1] trajectory2_start = trajectory2[0, :2]

    if sqrt(pow(trajectory1_start[0] - trajectory2_start[0], 2) + pow(trajectory1_start[1] - trajectory2_start[1], 2)) > 30:
        return False

    # three circle collision check
    collision_check_result, _ = collision_check(trajectory1, trajectory2, veh_length, tem_len, circle_r)
    if collision_check_result:
        return True

    line1 = LineString(trajectory1[:, :2])
    line2 = LineString(trajectory2[:, :2])

    # Check if the trajectories intersect
    if line1.intersects(line2):
        return True
    return False