import numpy as np
import threading
from queue import Queue
import time


class PP(threading.Thread):
    def __init__(self, main_global_q, global_main_q):
        super(PP, self).__init__()
        self.main_global_q =  main_global_q
        self.global_main_q = global_main_q

        self.RACECAR_LENGTH = 0.3302

        self.wp_index_current = 0
        self.current_position = [0] * 3
        self.nearest_distance = 0
        self.lookahead_desired = 0
        self.desired_point = 0
        self.actual_lookahead = 0
        self.CURRENT_WP_CHECK_OFFSET= 2
        self.transformed_desired_point = 0

        self.steering_direction = 0
        self.goal_path_radius = 0
        self.goal_path_theta = 0

        self.current_speed = 5.0
        self.scan_filtered = []
        self.scan_range = 0


    def get_lookahead_desired(self):
        _vel = self.current_speed
        self.lookahead_desired = 0.5 + (0.3 * _vel)

    def find_path(self):
        #right cornering
        if self.transformed_desired_point[0] > 0:
            self.goal_path_radius = pow(self.lookahead_desired, 2)/(2*self.transformed_desired_point[0])
            self.goal_path_theta = np.arcsin(self.transformed_desired_point[1]/self.goal_path_radius)
            self.steering_direction = -1

        #left cornering
        else:
            self.goal_path_radius = pow(self.lookahead_desired, 2)/((-2)*self.transformed_desired_point[0])
            self.goal_path_theta = np.arcsin(self.transformed_desired_point[1]/self.goal_path_radius)
            self.steering_direction = 1

    def setSteeringAngle(self):
        steering_angle = np.arctan2(self.RACECAR_LENGTH, self.goal_path_radius)
        steer = self.steering_direction * steering_angle
        return steer


    def run(self):
        """
        :param scan_data: scan data
        :param odom_data: odom data
        :return: steer, speed
        """
        while True:
            data = self.main_global_q.get()
            self.current_position = [data[0][0], data[0][1], data[0][2]]
            self.current_speed = data[1]
            self.transformed_desired_point = data[3]

            self.get_lookahead_desired()
            self.find_path()
            steer = self.setSteeringAngle()
            self.global_main_q.put(steer)
