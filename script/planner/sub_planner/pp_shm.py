import numpy as np
import threading
from multiprocessing import Lock
from queue import Queue
import time
from multiprocessing import shared_memory, Process

class PP(Process):
    def __init__(self, current_position, current_speed, transformed_desired_point, steer, lock, sp):
        super().__init__()
        self.cp = current_position
        self.cs = current_speed
        #self.sf = scan_filtered
        self.tdp = transformed_desired_point
        self.steer = steer
        self.lock = lock
        self.sp = sp

        self.RACECAR_LENGTH = 0.3302

        self.wp_index_current = 0
        self.current_position = [0] * 3
        self.nearest_distance = 0
        self.lookahead_desired = 0
        self.desired_point = 0
        self.actual_lookahead = 0
        self.CURRENT_WP_CHECK_OFFSET= 2
        self.transformed_desired_point = [0]*3

        self.steering_direction = 0
        self.goal_path_radius = 0
        self.goal_path_theta = 0

        #self.current_speed = 5.0
        #self.scan_filtered = []
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
            try:
                self.goal_path_radius = pow(self.lookahead_desired, 2)/((-2)*self.transformed_desired_point[0])
                self.goal_path_theta = np.arcsin(self.transformed_desired_point[1]/self.goal_path_radius)
                self.steering_direction = 1
            except ZeroDivisionError:
                print("ZeroDivision")

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
            if self.sp[0]==1:
                for i in range(len(self.cp)):
                    self.current_position[i] = self.cp[i]
                    self.transformed_desired_point[i] = self.tdp[i]
                self.current_speed = self.cs[0]
                #print(self.current_position, self.current_speed, self.transformed_desired_point)

                self.get_lookahead_desired()
                self.find_path()
                with self.lock:
                    self.steer[0] = float(self.setSteeringAngle())
                #print(self.steer)
            time.sleep(0.005)



