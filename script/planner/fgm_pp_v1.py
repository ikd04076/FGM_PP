import numpy as np
import math
import time

# from .sub_planner.speed_controller import SpeedController as SC
"""
Static Functions
"""


def _load_waypoint(conf):
    """
    Return s a list of waypoints from a file.
    """
    try:
        file_wps = np.loadtxt(conf.wpt_path, delimiter=conf.wpt_delim, skiprows=conf.wpt_rowskip)
        waypoints = list()
        for i in file_wps:
            wps_point = [i[conf.wpt_xind], i[conf.wpt_yind], 0]
            waypoints.append(wps_point)
        return waypoints

    except AttributeError:
        print("Config parsing error")
        exit()


def _get_distance(a, b):
    """
    Return the distance between two points.
    """
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return np.sqrt(dx ** 2 + dy ** 2)


def _get_lookahead_desired(current_speed):
    """
    Return the desired lookahead distance given the current speed.
    """
    # return 0.5 + (0.3 * current_speed)
    return 2

def _get_angle(range_index, range_len, angle_increment):
    """
    Return the angle of the range index.
    """
    lidar_angle = (range_index - (range_len / 2)) * angle_increment
    steering_angle = lidar_angle / 2
    return steering_angle


def _transform_point(PI, origin, target):
    """
    Return the transformed point given the origin and the rotation matrix.
    """
    theta = PI / 2 - origin[2]

    dx = target[0] - origin[0]
    dy = target[1] - origin[1]
    dtheta = target[2] + theta

    tf_point_x = dx * np.cos(theta) - dy * np.sin(theta)
    tf_point_y = dx * np.sin(theta) + dy * np.cos(theta)
    tf_point_theta = dtheta
    tf_point = [tf_point_x, tf_point_y, tf_point_theta]

    return tf_point


def _xyt_to_rt(PI, origin):
    """
    Return the rotation matrix and translation vector given the origin.
    """
    rtpoint = []

    x = origin[0]
    y = origin[1]

    rtpoint.append(np.sqrt(x * x + y * y))
    rtpoint.append(np.arctan2(y, x) - (PI / 2))
    return rtpoint


def _find_max_gap(free_space_ranges):
    """
    Return the maximum gap between two consecutive free space ranges.
    """
    # mask the bubble
    masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
    # get a slice for each contigous sequence of non-bubble data
    slices = np.ma.notmasked_contiguous(masked)
    max_len = slices[0].stop - slices[0].start
    chosen_slice = slices[0]
    # I think we will only ever have a maximum of 2 slices but will handle an
    # indefinitely sized list for portablility
    for sl in slices[1:]:
        sl_len = sl.stop - sl.start
        if sl_len > max_len:
            max_len = sl_len
            chosen_slice = sl
    return chosen_slice.start, chosen_slice.stop


def _find_steer_path(transformed_desired_point, actual_lookahead):
    """
    Return the steering angle and the distance to the desired point.
    """
    if transformed_desired_point[0] > 0:
        goal_path_radius = pow(actual_lookahead, 2) / (2 * transformed_desired_point[0])
        goal_path_theta = np.arcsin(transformed_desired_point[1] / goal_path_radius)
        steering_direction = -1  # Right

    else:
        goal_path_radius = pow(actual_lookahead, 2) / ((-2) * transformed_desired_point[0])
        goal_path_theta = np.arcsin(transformed_desired_point[1] / goal_path_radius)
        steering_direction = 1  # Left

    return goal_path_radius, goal_path_theta, steering_direction


class FGM_PP_V1:
    def __init__(self, conf, robot_scale=0.325):
        self.conf = conf

        self.SPEED_MAX = self.conf.max_speed
        self.SPEED_MIN = self.conf.min_speed
        self.ANGLE_MAX = 0
        self.ANGLE_MIN = 0

        self.PI = 3.141592
        self.RACECAR_LENGTH = robot_scale

        self.PREPROCESS_CONV_SIZE = 100
        self.BEST_POINT_CONV_SIZE = 80
        self.MAX_LIDAR_DIST = 3000000
        self.CURRENT_WP_CHECK_OFFSET = 2
        self.BUBBLE_RADIUS = 160

        self.waypoints = _load_waypoint(self.conf)

        self.current_waypoint_index = 0
        self.current_speed = 0.0
        self.current_position = [0] * 3
        self.scan_filtered = []
        self.scan_range = 0
        self.radians_per_elem = 0.00435

    def find_nearest_waypoint(self, current_position, waypoints, current_waypoint_index):
        """
        Find the nearest waypoint to the current position.
        """
        temp_index = current_waypoint_index
        nearest_distance = _get_distance(waypoints[temp_index], current_position)

        while True:
            temp_index += 1
            if temp_index >= len(waypoints) - 1:
                temp_index = 0

            temp_distance = _get_distance(waypoints[temp_index], current_position)

            if temp_distance < nearest_distance:
                nearest_distance = temp_distance
                current_waypoint_index = temp_index
            elif temp_distance > (nearest_distance + self.CURRENT_WP_CHECK_OFFSET) or (
                    temp_index == current_waypoint_index):
                break

        transformed_nearest_point = _transform_point(self.PI, current_position, waypoints[current_waypoint_index])
        if transformed_nearest_point[0] < 0:
            nearest_distance *= -1

        return current_waypoint_index, nearest_distance

    def find_desired_waypoint(self, current_position, waypoints, current_waypoint_index, lookahead_desired):
        """
        Find the desired waypoint to the current position.
        """
        temp_index = current_waypoint_index
        while True:
            if temp_index >= len(waypoints) - 1:
                temp_index = 0
            distance = _get_distance(waypoints[temp_index], current_position)
            if distance >= lookahead_desired:
                if temp_index - 2 >= 0 and temp_index + 2 < len(waypoints) - 1:
                    waypoints[temp_index][2] = np.arctan(
                        (waypoints[temp_index + 2][1] - waypoints[temp_index - 2][1]) /
                        waypoints[temp_index + 2][0] - waypoints[temp_index - 2][0])
                desired_point = waypoints[temp_index]
                actual_lookahead = distance
                break
            else:
                temp_index += 1

        return desired_point, actual_lookahead, waypoints

    def preprocess_lidar(self, scan_data):
        """
        Preprocess the lidar data.
        """
        self.scan_range = len(scan_data)
        self.scan_filtered = [0] * self.scan_range
        for i in range(self.scan_range):
            self.scan_filtered[i] = scan_data[i]

        proc_ranges = np.convolve(scan_data, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def obstacle_detection(self, scan_filtered):
        """
        This function is used to detect obstacles in the scan data.
        """
        scan_obstacle = list()
        i = 1
        d_group = 1.5
        d_pi = 0.00628
        while self.scan_range - 1 > i:
            temp_start_index = i
            temp_end_index = i
            temp_max_index = i
            temp_min_index = i
            i += 1
            while math.sqrt(math.pow(scan_filtered[i] * math.sin(math.radians(0.25)), 2) + math.pow(
                    scan_filtered[i - 1] - scan_filtered[i] * math.cos(math.radians(0.25)), 2)) < d_group + \
                    scan_filtered[i] * d_pi and (i + 1 < self.scan_range):
                if scan_filtered[i] > scan_filtered[temp_max_index]:
                    temp_max_index = i
                if scan_filtered[i] < scan_filtered[temp_min_index]:
                    temp_min_index = i
                i += 1
            temp_end_index = i - 1
            scan_obstacle.append([temp_start_index, temp_end_index, temp_max_index, scan_filtered[temp_max_index],
                                  scan_filtered[temp_min_index]])
            i += 1

        obstacle_split = list()
        obstacle_detect = list()
        for i in range(len(scan_obstacle)):
            if 4 > scan_obstacle[i][3] > 0:
                obstacle_split.append([scan_obstacle[i][0], scan_obstacle[i][1], scan_obstacle[i][2],
                                       scan_obstacle[i][3], scan_obstacle[i][4]])
                obstacle_detect.append([scan_obstacle[i][0], scan_obstacle[i][1], scan_obstacle[i][2],
                                        scan_obstacle[i][3], scan_obstacle[i][4]])

        obstacle_length = list()
        for i in range(len(obstacle_detect)):
            theta = (obstacle_detect[i][1] - obstacle_detect[i][0]) * 0.25
            length = math.sqrt(
                math.pow(scan_filtered[obstacle_detect[i][1]] * math.sin(math.radians(theta)), 2) + math.pow(
                    scan_filtered[obstacle_detect[i][0]] - scan_filtered[obstacle_detect[i][1]] * math.cos(
                        math.radians(theta)), 2))

            if length < 1:
                obstacle_length.append([obstacle_detect[i][0], obstacle_detect[i][1], obstacle_detect[i][2],
                                        obstacle_detect[i][3], obstacle_detect[i][4]])

        obstacle = False
        for i in range(len(obstacle_length)):
            if obstacle_length[i][0] > 680 or obstacle_length[i][1] < 400:
                obstacle = False
            else:
                print("Obstacle Caught.")
                obstacle = True
                break

        return obstacle

    def set_steering_angle(self, goal_path_radius, steering_direction):
        steer = steering_direction * np.arctan2(self.RACECAR_LENGTH, goal_path_radius)
        return steer

    def find_best_point(self, start_i, end_i, ranges):
        # do a sliding window average over the data in the max gap, this will
        # help the car to avoid hitting corners
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def fgm_convolution(self, scan_filtered):
        proc_ranges = scan_filtered[135:-135]
        closest_point = proc_ranges.argmin()

        min_index = closest_point - self.BUBBLE_RADIUS
        max_index = closest_point + self.BUBBLE_RADIUS
        if min_index < 0:
            min_index = 0
        if max_index > len(proc_ranges):
            max_index = len(proc_ranges) - 1

        proc_ranges[min_index:max_index] = 0

        gap_start, gap_end = _find_max_gap(proc_ranges)
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        steering_angle = _get_angle(best, len(proc_ranges), self.radians_per_elem)
        return steering_angle

    def driving(self, observation):
        """

        :param scan_data: scan data
        :param odom_data: odom data
        :return: steer, speed
        """

        self.current_position = np.array([observation['poses_x'], observation['poses_y'], observation['poses_theta']])
        self.current_speed = observation['linear_vels_x']

        self.scan_filtered = self.preprocess_lidar(observation['scans'])

        self.current_waypoint_index, nearest_distance = self.find_nearest_waypoint(self.current_position,
                                                                                   self.waypoints,
                                                                                   self.current_waypoint_index)
        lookahead_desired = _get_lookahead_desired(self.current_speed)
        desired_point, actual_lookahead, self.waypoints = self.find_desired_waypoint(self.current_position,
                                                                                     self.waypoints,
                                                                                     self.current_waypoint_index,
                                                                                     lookahead_desired)

        transformed_desired_point = _transform_point(self.PI, self.current_position, desired_point)
        obstacle_detection = self.obstacle_detection(self.scan_filtered)

        if obstacle_detection:
            steer = self.fgm_convolution(self.scan_filtered)
        else:
            goal_path_radius, _, steering_direction = _find_steer_path(transformed_desired_point,
                                                                       actual_lookahead)
            steer = self.set_steering_angle(goal_path_radius, steering_direction)

        if np.fabs(steer) > self.PI / 8:
            speed = self.SPEED_MIN

        else:
            angle_control = self.ANGLE_MAX * 4.25
            if np.fabs(angle_control) > 1:
                angle_control = 1
            speed = float(
                -(3 / self.PI) * (self.SPEED_MAX - self.SPEED_MIN) * np.fabs(angle_control) + self.SPEED_MAX)
            speed = np.fabs(speed)

            if self.SPEED_MAX < speed or speed < self.SPEED_MIN:
                print(speed, angle_control)

        return speed, steer