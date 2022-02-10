#!/usr/bin/env python

import math
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray

class FGMPlanner:
    def __init__(self, conf, robot_scale=0.3302):
        self.conf = conf
        self.PI = 3.141592
        self.rf_distance = 2.5
        self.RACECAR_LENGTH = 0.325
        self.SPEED_MAX = self.conf.max_speed
        self.SPEED_MIN = self.conf.min_speed

        self.ROBOT_SCALE = robot_scale
        self.THRESHOLD = 3.0
        self.GAP_SIZE = 1
        self.FILTER_SCALE = 1.1
        self.GAP_THETA_GAIN = 20.0
        self.REF_THETA_GAIN = 1.5

        self.scan_range = 0
        self.desired_gap = 0
        self.desired_wp_rt = [0, 0]

        self.wp_num = 1
        self.waypoints = self.get_waypoint()
        self.wp_index_current = 0
        self.current_position = [0] * 3
        self.nearest_distance = 0

        self.max_angle = 0
        self.wp_angle = 0

        self.gaps = []
        self.for_gap = [0, 0, 0]
        self.for_point = 0
        self.interval = 0.00435
        self.front_idx = 539
        self.theta_for = self.PI / 3
        self.gap_cont = 0
        self.dmin_past = 1.0

    def getDistance(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]

        return np.sqrt(dx ** 2 + dy ** 2)

    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

        _vel = self.current_speed

        #self.LOOK = 1.5 + (0.3 * _vel)
        self.LOOK = 0.5 + (0.5 * _vel)

        while True:
            wp_index_temp+=1

            if wp_index_temp >= self.wp_num-1:
                wp_index_temp = 0
                # print(self.lap_time)

            temp_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

            if temp_distance < self.nearest_distance:
                self.nearest_distance = temp_distance
                self.wp_index_current = wp_index_temp
            elif ((temp_distance > (self.nearest_distance + self.LOOK*1.2)) or (wp_index_temp == self.wp_index_current)):
                break
        
        temp_distance = 0
        idx_temp = self.wp_index_current
        while True:
            if idx_temp >= self.wp_num-1:
                idx_temp = 0
            temp_distance = self.getDistance(self.waypoints[idx_temp], self.current_position)
            if temp_distance > self.LOOK: break
            idx_temp += 1

        transformed_nearest_point = self.transformPoint(self.current_position, self.waypoints[idx_temp])
        self.desired_wp_rt = self.xyt2rt(transformed_nearest_point)

        self.idx_temp = idx_temp
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mpc"
        marker.id = 2
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position.x = self.waypoints[self.idx_temp][0]
        marker.pose.position.y = self.waypoints[self.idx_temp][1]
        marker.pose.position.z = 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        # print(self.waypoints[self.idx_temp], self.idx_temp)
        self.marker_pub.publish(marker)
    
    def xyt2rt(self, origin):
        rtpoint = []

        x = origin[0]
        y = origin[1]

        # rtpoint[0] = r, [1] = theta
        rtpoint.append(np.sqrt(x * x + y * y))
        rtpoint.append(np.arctan2(y, x) - (self.PI / 2))

        return rtpoint

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



    def GAP(self):
        if self.gap_cont <= 1:
            self.for_point = (self.theta_for / self.interval)
            # [0] = start_idx, [1] = end_idx
            self.for_gap[0] = self.front_idx - self.for_point
            self.for_gap[1] = self.front_idx + self.for_point

            self.gap_cont += 1

    def find_gap(self, scan):
        self.gaps = []

        i = 0
        while i < self.scan_range - self.GAP_SIZE:

            if scan[i] > self.THRESHOLD:
                start_idx_temp = i
                end_idx_temp = i
                max_temp = scan[i]
                max_idx_temp = i

                for j in range(1, self.GAP_SIZE):

                    if scan[i] > self.THRESHOLD:
                        if scan[i] > max_temp:
                            max_temp = scan[i]
                            max_idx_temp = i
                        else:
                            max_temp = -1
                            break

                if max_temp == -1:
                    break

                else:
                    while (scan[i] > self.THRESHOLD) and (i + 1 < self.scan_range):
                        i += 1
                        if scan[i] > max_temp:
                            max_temp = scan[i]
                            max_idx_temp = i
                    i += 1
                    end_idx_temp = i

                    gap_temp = [0] * 3
                    gap_temp[0] = start_idx_temp
                    gap_temp[1] = end_idx_temp
                    gap_temp[2] = max_idx_temp
                    self.gaps.append(gap_temp)
            i += 1

    def for_find_gap(self, scan):
        self.for_point = int(self.theta_for / self.interval)
        # [0] = start_idx, [1] = end_idx
        start_idx_temp = self.front_idx - self.for_point
        end_idx_temp = self.front_idx + self.for_point

        max_idx_temp = start_idx_temp
        max_temp = scan[start_idx_temp]

        for i in range(start_idx_temp, end_idx_temp):
            if max_temp < scan[i]:
                max_temp = scan[i]
                max_idx_temp = i
        # [0] = start_idx, [1] = end_idx, [2] = max_idx_temp
        self.for_gap[0] = start_idx_temp
        self.for_gap[1] = end_idx_temp
        self.for_gap[2] = max_idx_temp

    def find_best_gap(self, ref):
        num = len(self.gaps)
        if num == 0:
            return self.for_gap
        else:
            step = (int)(ref[1] / self.interval)
            ref_idx = self.front_idx + step

            gap_idx = 0

            if self.gaps[0][0] > ref_idx:
                distance = self.gaps[0][0] - ref_idx
            elif self.gaps[0][1] < ref_idx:
                distance = ref_idx - self.gaps[0][1]
            else:
                distance = 0
                gap_idx = 0

            i = 1
            while (i < num):
                if self.gaps[i][0] > ref_idx:
                    temp_distance = self.gaps[i][0] - ref_idx
                    if temp_distance < distance:
                        distance = temp_distance
                        gap_idx = i
                elif self.gaps[i][1] < ref_idx:
                    temp_distance = ref_idx - self.gaps[i][1]
                    if temp_distance < distance:
                        distance = temp_distance
                        gap_idx = i

                else:
                    temp_distance = 0
                    distance = 0
                    gap_idx = i
                    break

                i += 1

            return self.gaps[gap_idx]

    def main_drive(self, goal):
        # goal - [2] = max_idx,
        self.max_angle = (goal[2] - self.front_idx) * self.interval
        self.wp_angle = self.desired_wp_rt[1]

        # range_min_values = [0]*10
        temp_avg = 0
        dmin = 0
        for i in range(10):
            dmin += self.scan_filtered[i]
        dmin /= 10

        i = 0
        while i < self.scan_range - 7:
            j = 0
            while j < 10:
                if i + j > 1079:
                    temp_avg += 0
                else:
                    temp_avg += self.scan_filtered[i + j]
                j += 1

            temp_avg /= 10

            if dmin > temp_avg:
                if temp_avg == 0:
                    temp_avg = dmin
                dmin = temp_avg
            temp_avg = 0
            i += 3

        if dmin == 0:
            dmin = self.dmin_past

        controlled_angle = ((self.GAP_THETA_GAIN / dmin) * self.max_angle + self.REF_THETA_GAIN * self.wp_angle) / (
                self.GAP_THETA_GAIN / dmin + self.REF_THETA_GAIN)
        distance = 1.0
        path_radius = distance / (2 * np.sin(controlled_angle))
        steering_angle = np.arctan(self.RACECAR_LENGTH / path_radius)

        if np.fabs(steering_angle) > self.PI / 8:
            speed = self.SPEED_MIN

        else:
            angle_control = self.max_angle * 4.25
            if np.fabs(angle_control) > 1:
                angle_control = 1
            speed = float(
                -(3 / self.PI) * (self.SPEED_MAX - self.SPEED_MIN) * np.fabs(angle_control) + self.SPEED_MAX)
            speed = np.fabs(speed)

        if self.SPEED_MAX < speed or speed < self.SPEED_MIN:
            print(speed, angle_control)

        self.dmin_past = dmin

        return speed, steering_angle

    def subcallback_scan(self, msg):
        self.scan_range = len(msg)
        self.front_idx = int(self.scan_range / 2)

        self.scan_origin = [0] * self.scan_range
        self.scan_filtered = [0] * self.scan_range

        for i in range(self.scan_range):
            self.scan_origin[i] = msg[i]
            self.scan_filtered[i] = msg[i]

        for i in range(self.scan_range - 1):
            if self.scan_origin[i] * self.FILTER_SCALE < self.scan_origin[i + 1]:
                unit_length = self.scan_origin[i] * self.interval
                filter_num = self.ROBOT_SCALE / unit_length

                j = 1
                while j < filter_num + 1:
                    if i + j < self.scan_range:
                        if self.scan_filtered[i + j] > self.scan_origin[i]:
                            self.scan_filtered[i + j] = self.scan_origin[i]
                        else:
                            break
                    else:
                        break
                    j += 1

            elif self.scan_filtered[i] > (self.scan_origin[i + 1] * self.FILTER_SCALE):
                unit_length = self.scan_origin[i + 1] * self.interval
                filter_num = self.ROBOT_SCALE / unit_length

                j = 0
                while j < filter_num + 1:
                    if i - j > 0:
                        if self.scan_filtered[i - j] > self.scan_origin[i + 1]:
                            self.scan_filtered[i - j] = self.scan_origin[i + 1]
                        else:
                            break
                    else:
                        break
                    j += 1

    def driving(self, obs):
        self.current_position = np.array([obs['poses_x'], obs['poses_y'], obs['poses_theta']])
        self.subcallback_scan(obs['scans'])
        self.GAP()
        self.find_gap(self.scan_filtered)
        self.for_find_gap(self.scan_filtered)
        self.desired_gap = self.find_best_gap(self.desired_wp_rt)
        speed, steer = self.main_drive(self.desired_gap)
        return speed, steer
