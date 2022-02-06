#!/usr/bin/env python

import numpy as np


class PurePursuitPlanner:
    def __init__(self, conf, robot_scale=0.3302):
        self.conf = conf
        self.PI = 3.141592
        self.CURRENT_WP_CHECK_OFFSET = 2
        self.DX_GAIN = 2.5
        self.RACECAR_LENGTH = robot_scale
        self.GRAVITY_ACCELERATION = 9.81

        self.LOOKAHEAD_MAX = 2.0
        self.LOOKAHEAD_MIN = 1.0
        self.SPEED_MAX = self.conf.max_speed
        self.SPEED_MIN = self.conf.min_speed
        self.MSC_MUXSIZE = 0
        self.MU = 1

        self.waypoints = self.get_waypoint()
        self.wp_index_current = 0
        self.current_position = np.array([0, 0, 0])
        self.lookahead_desired = 0
        self.steering_direction = 0
        self.goal_path_radius = 0
        self.goal_path_theta = 0
        self.actual_lookahead = 0
        self.transformed_desired_point = []
        self.desired_point = []
        self.dx = 0
        self.nearest_distance = 0
        self.manualSpeedArray = []

    def get_distance(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]

        return np.sqrt(dx ** 2 + dy ** 2)

    def transformPoint(self, origin, target):
        theta = self.PI / 2 - origin[2]

        dx = target[0] - origin[0]
        dy = target[1] - origin[1]
        dtheta = target[2] + theta

        tf_point_x = dx * np.cos(theta) - dy * np.sin(theta)
        tf_point_y = dx * np.sin(theta) + dy * np.cos(theta)
        tf_point_theta = dtheta
        tf_point = [tf_point_x, tf_point_y, tf_point_theta]

        return tf_point

    def get_waypoint(self):
        file_wps = np.loadtxt(self.conf.wpt_path, delimiter=self.conf.wpt_delim,
                                    skiprows=self.conf.wpt_rowskip)
        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[self.conf.wpt_xind], i[self.conf.wpt_yind], 0]
            temp_waypoint.append(wps_point)

        return temp_waypoint

    def find_nearest_wp(self):
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.get_distance(self.waypoints[wp_index_temp], self.current_position)

        while True:
            wp_index_temp += 1
            if wp_index_temp >= len(self.waypoints) - 1:
                wp_index_temp = 0

            temp_distance = self.get_distance(self.waypoints[wp_index_temp], self.current_position)

            if temp_distance < self.nearest_distance:
                self.nearest_distance = temp_distance
                self.wp_index_current = wp_index_temp
            elif temp_distance > (self.nearest_distance + self.CURRENT_WP_CHECK_OFFSET) or (
                    wp_index_temp == self.wp_index_current):
                break

        transformed_nearest_point = self.transformPoint(self.current_position, self.waypoints[self.wp_index_current])
        if transformed_nearest_point[0] < 0: self.nearest_distance *= -1

    def get_dx(self):
        wp_min = self.find_lookahead_wp(self.LOOKAHEAD_MIN)
        wp_max = self.find_lookahead_wp(self.LOOKAHEAD_MAX)

        wp_min = self.transformPoint(self.current_position, wp_min)
        wp_max = self.transformPoint(self.current_position, wp_max)

        self.dx = wp_max[0] - wp_min[0]

    def get_lookahead_desired(self):
        self.lookahead_desired = np.exp(
            -(self.DX_GAIN * np.fabs(self.dx) - np.log(self.LOOKAHEAD_MAX - self.LOOKAHEAD_MIN))) + self.LOOKAHEAD_MIN

    def find_lookahead_wp(self, length):
        wp_index_temp = self.wp_index_current
        while (1):
            if wp_index_temp >= len(self.waypoints) - 1: wp_index_temp = 0
            distance = self.get_distance(self.waypoints[wp_index_temp], self.current_position)

            if distance >= length: break
            wp_index_temp += 1
        return self.waypoints[wp_index_temp]

    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        while 1:
            if wp_index_temp >= len(self.waypoints) - 1: wp_index_temp = 0
            distance = self.get_distance(self.waypoints[wp_index_temp], self.current_position)
            if distance >= self.lookahead_desired:
                if wp_index_temp - 2 >= 0 and wp_index_temp + 2 < len(self.waypoints) - 1:
                    self.waypoints[wp_index_temp][2] = np.arctan(
                        (self.waypoints[wp_index_temp + 2][1] - self.waypoints[wp_index_temp - 2][1]) /
                        self.waypoints[wp_index_temp + 2][0] - self.waypoints[wp_index_temp - 2][0])
                self.desired_point = self.waypoints[wp_index_temp]
                self.actual_lookahead = distance
                break
            wp_index_temp += 1

    def find_path(self):
        # right cornering
        if self.transformed_desired_point[0] > 0:
            self.goal_path_radius = pow(self.actual_lookahead, 2) / (2 * self.transformed_desired_point[0])
            self.goal_path_theta = np.arcsin(self.transformed_desired_point[1] / self.goal_path_radius)
            self.steering_direction = -1

        # left cornering
        else:
            self.goal_path_radius = pow(self.actual_lookahead, 2) / ((-2) * self.transformed_desired_point[0])
            self.goal_path_theta = np.arcsin(self.transformed_desired_point[1] / self.goal_path_radius)
            self.steering_direction = 1

    def set_steering_angle(self):
        steering_angle = np.arctan2(self.RACECAR_LENGTH, self.goal_path_radius)

        return self.steering_direction * steering_angle

    def set_speed(self):
        controlled_speed_max = self.SPEED_MAX
        controlled_speed_min = self.SPEED_MIN
        for i in range(0, self.MSC_MUXSIZE):
            if (self.wp_index_current > self.manualSpeedArray[i][0]) and (
                    self.wp_index_current < self.manualSpeedArray[i][1]):
                controlled_speed_max = self.manualSpeedArray[i][2]
                controlled_speed_min = self.manualSpeedArray[i][3]
                break
        return np.exp(-(self.DX_GAIN * np.fabs(self.dx) - np.log(
            controlled_speed_max - controlled_speed_min))) + controlled_speed_min

    def driving(self, obs):
        self.current_position = np.array([obs['poses_x'], obs['poses_y'], obs['poses_theta']])

        self.find_nearest_wp()
        self.get_dx()
        self.get_lookahead_desired()
        self.find_desired_wp()
        self.transformed_desired_point = self.transformPoint(self.current_position, self.desired_point)
        self.find_path()

        steer = self.set_steering_angle()
        speed = self.set_speed()

        return speed, steer
