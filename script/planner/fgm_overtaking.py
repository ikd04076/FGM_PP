from .sub_planner.sc_for_fgm import SpeedController as SC
import numpy as np
import math
import time


class FGM:
    def __init__(self, params):

        self.RACECAR_LENGTH = params.robot_length
        self.ROBOT_LENGTH = params.robot_length
        self.SPEED_MAX = params.max_speed
        self.SPEED_MIN = params.min_speed

        self.MU = params.mu
        self.GRAVITY_ACC = params.g
        self.PI = params.pi
        self.ROBOT_SCALE = params.robot_scale
        self.mode = params.speed_controller

        self.LOOK = params.fgm['look']
        self.THRESHOLD = params.fgm['threshold']
        self.GAP_SIZE = params.fgm['gap_size']
        self.FILTER_SCALE = params.fgm['filter_scale']
        self.GAP_THETA_GAIN = params.fgm['gap_theta_gain']
        self.REF_THETA_GAIN = params.fgm['ref_theta_gain']

        self.BEST_POINT_CONV_SIZE = 80

        self.waypoint_real_path = params.wpt_path
        self.waypoint_delimeter = params.wpt_delimeter

        self.scan_range = 0
        self.desired_gap = 0
        self.speed_gain = 0
        self.steering_gain = 0
        self.gain_cont = 0
        self.speed_cont = 0
        self.desired_wp_rt = [0, 0]

        self.speed_up = 0

        self.wp_num = 1
        self.waypoints = self.get_waypoint()
        self.wp_index_current = 0
        self.current_position = [0] * 3
        self.nearest_distance = 0

        self.max_angle = 0
        self.wp_angle = 0
        self.detect_range_s = 299
        self.detect_range_e = 779
        self.gaps = []
        self.for_gap = [0, 0, 0]
        self.for_point = 0

        self.interval = 0.00435  # 1도 = 0.0175라디안

        self.front_idx = 0
        self.theta_for = self.PI / 3
        self.gap_cont = 0

        self.current_speed = 5.0
        self.dmin_past = 0
        self.lap = 0

        self.scan_obs = []
        self.dect_obs = []
        self.len_obs = []
        self.obs = False
        self.ovt = False
        self.past_point = 0

        self.closest_wp_dist = 0
        self.closest_obs_dist = 0
        self.speed_control = SC(params)

    def find_nearest_obs(self, obs):
        min_di = 0
        min_dv = 0
        if len(obs) <= 1:
            min_di = 20
            min_dv = 20
        else:
            min_di = self.getDistance(self.current_position, obs[0])
            for i in range(len(obs)):
                _dist = self.getDistance(self.current_position, obs[i])
                if _dist <= min_di:
                    min_di = _dist
                    min_dv = self.getDistance(self.waypoints[self.wp_index_current], obs[i])

        self.closest_obs_dist = min_di
        self.closest_wp_dist = min_dv

    def getDistance(self, a, b):
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

    def xyt2rt(self, origin):
        rtpoint = []

        x = origin[0]
        y = origin[1]

        # rtpoint[0] = r, [1] = theta
        rtpoint.append(np.sqrt(x * x + y * y))
        rtpoint.append(np.arctan2(y, x) - (self.PI / 2))

        return rtpoint

    def get_waypoint(self):
        file_wps = np.genfromtxt(self.waypoint_real_path, delimiter=self.waypoint_delimeter, dtype='float')

        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0], i[1], 0]
            temp_waypoint.append(wps_point)
            self.wp_num += 1
        return temp_waypoint

    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

        while True:
            wp_index_temp += 1

            if wp_index_temp >= self.wp_num - 1:
                wp_index_temp = 0

            temp_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

            if temp_distance < self.nearest_distance:
                self.nearest_distance = temp_distance
                self.wp_index_current = wp_index_temp
            elif (temp_distance > (self.nearest_distance + self.LOOK * 1.2)) or (
                    wp_index_temp == self.wp_index_current):
                break

        temp_distance = 0
        idx_temp = self.wp_index_current
        while True:
            if idx_temp >= self.wp_num - 1:
                idx_temp = 0
            temp_distance = self.getDistance(self.waypoints[idx_temp], self.current_position)
            if temp_distance > self.LOOK: break
            idx_temp += 1

        transformed_nearest_point = self.transformPoint(self.current_position, self.waypoints[idx_temp])
        self.desired_wp_rt = self.xyt2rt(transformed_nearest_point)

    def subCallback_scan(self, scan_data):

        self.scan_range = len(scan_data)

        self.front_idx = (int(self.scan_range / 2))

        self.scan_origin = [0] * self.scan_range
        self.scan_filtered = [0] * self.scan_range

        for i in range(self.scan_range):
            self.scan_origin[i] = scan_data[i]
            self.scan_filtered[i] = scan_data[i]

        for i in range(self.scan_range - 1):
            if self.scan_origin[i] * self.FILTER_SCALE < self.scan_filtered[i + 1]:
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

            elif self.scan_filtered[i] > self.scan_origin[i + 1] * self.FILTER_SCALE:
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

        return self.scan_filtered

    def find_gap(self, scan):
        self.gaps = []

        i = 0

        while i < self.scan_range - self.GAP_SIZE:

            if scan[i] > self.THRESHOLD:
                start_idx_temp = i
                end_idx_temp = i
                # max_temp = scan[i]
                # max_idx_temp = i

                while ((scan[i] > self.THRESHOLD) and (i + 1 < self.scan_range)):
                    i += 1
                    # if scan[i] > max_temp:
                    #     max_temp = scan[i]
                    #     max_idx_temp = i
                if scan[i] > self.THRESHOLD:
                    i += 1
                end_idx_temp = i

                gap_temp = [0] * 3
                gap_temp[0] = start_idx_temp
                gap_temp[1] = end_idx_temp
                # gap_temp[2] = max_idx_temp
                self.gaps.append(gap_temp)
            i += 1

    def for_find_gap(self, scan):
        self.for_point = (int)(self.theta_for / self.interval)
        # [0] = start_idx, [1] = end_idx

        start_idx_temp = (self.front_idx) - self.for_point
        end_idx_temp = (self.front_idx) + self.for_point

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

    # ref - [0] = r, [1] = theta
    def find_best_gap(self, ref):
        num = len(self.gaps)

        if num == 0:
            return self.for_gap
        else:

            step = (int(ref[1] / self.interval))

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
            # 가장 작은 distance를 갖는 gap만 return
            return self.gaps[gap_idx]

    def find_best_point(self, best_gap):
        averaged_max_gap = np.convolve(self.scan_filtered[best_gap[0]:best_gap[1]], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + best_gap[0]

    def main_drive(self, max_gap):
        self.max_angle = (max_gap - self.front_idx) * self.interval  # (goal[2] - self.front_idx) * self.interval
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
        # path_radius = 경로 반지름
        path_radius = distance / (2 * np.sin(controlled_angle))
        #
        steering_angle = np.arctan(self.RACECAR_LENGTH / path_radius)

        steer = steering_angle
        if self.ovt:
            speed = self.speed_control.routine(self.scan_filtered, self.current_speed, steering_angle, 1)
        else:
            speed = self.speed_control.routine(self.scan_filtered, self.current_speed, steering_angle, 0)
        self.dmin_past = dmin

        return steer, speed

    def obs_dect(self):
        #for i in range(1, self.scan_range - 1):
        scan_f = []
        scan_temp = []
        for i in range(0, 269):
            scan_temp = [0]*4
            scan_temp[0] = self.scan_origin[int(i)*4]
            scan_temp[1] = self.scan_origin[int(i) * 4+1]
            scan_temp[2] = self.scan_origin[int(i) * 4+2]
            scan_temp[3] = self.scan_origin[int(i) * 4+3]
            scan_temp = np.array(scan_temp)
            max= scan_temp.argmax()
            scan_f.append(scan_temp[max])
        self.scan_obs = []
        i=1
        d_group = 2
        d_pi = 0.00628
        while(len(scan_f) - 1>i):
            start_idx_temp = i
            end_idx_temp = i
            max_idx_temp = i
            min_idx_temp = i
            i = i+1
            while  math.sqrt(math.pow(scan_f[i]*math.sin(math.radians(0.25)),2) + math.pow(scan_f[i-1]-scan_f[i]*math.cos(math.radians(0.25)),2)) < d_group + scan_f[i]*d_pi and (i+1 < 269 ):
                if scan_f[i] > scan_f[max_idx_temp]:
                    max_idx_temp = i
                if scan_f[i] < scan_f[min_idx_temp]:
                    min_idx_temp = i
                i = i+1
            end_idx_temp = i-1
            obs_temp = [0]*6
            obs_temp[0] = start_idx_temp
            obs_temp[1] = end_idx_temp
            obs_temp[2] = max_idx_temp
            obs_temp[3] = min_idx_temp
            obs_temp[4] = scan_f[max_idx_temp]
            obs_temp[5] = scan_f[min_idx_temp]
            self.scan_obs.append(obs_temp)
            i+=1

        self.dect_obs=[]
        for i in range(len(self.scan_obs)):
            if self.scan_obs[i][5] < 5 and self.scan_obs[i][5] > 0:
                obs_temp = [0]*6
                obs_temp[0] = self.scan_obs[i][0]
                obs_temp[1] = self.scan_obs[i][1]
                obs_temp[2] = self.scan_obs[i][2]
                obs_temp[3] = self.scan_obs[i][3]
                obs_temp[4] = self.scan_obs[i][4]
                obs_temp[5] = self.scan_obs[i][5]
                self.dect_obs.append(obs_temp)
        #print(self.dect_obs)
        self.len_obs=[]

        for i in range(len(self.dect_obs)):
            theta = (self.dect_obs[i][1] - self.dect_obs[i][0])
            lengh = math.sqrt(math.pow(scan_f[self.dect_obs[i][1]]*math.sin(math.radians(theta)),2) + math.pow(scan_f[self.dect_obs[i][0]]-scan_f[self.dect_obs[i][1]]*math.cos(math.radians(theta)),2))
            #print(i,lengh)
            if lengh < 1 and lengh >0:
                obs_temp = [0]*6
                obs_temp[0] = self.dect_obs[i][0]
                obs_temp[1] = self.dect_obs[i][1]
                obs_temp[2] = self.dect_obs[i][2]
                obs_temp[3] = self.dect_obs[i][3]
                obs_temp[4] = self.dect_obs[i][4]
                obs_temp[5] = self.dect_obs[i][5]
                self.len_obs.append(obs_temp)

        self.ovt = False
        for i in range(len(self.len_obs)):
            if self.len_obs[i][0] > 720/4 or self.len_obs[i][1] < 360/4:
                # print(self.len_obs)
                self.obs = False
                self.ovt = False
            else:
                # print(self.scan_obs)
                # print(self.dect_obs)
                # print(self.len_obs)
                # print(scan_f[162],scan_f[163],scan_f[164],scan_f[165])
                #print(self.scan_origin[475],self.scan_origin[476],self.scan_origin[477],self.scan_origin[478])
                if self.obs== True:
                    self.ovt = True
                self.obs = True
                break

    def find_ovt_gap(self):
        num = len(self.gaps)

        if num == 0:
            return self.for_gap
        else:

            step = (int(self.past_point / self.interval))

            #ref_idx = self.front_idx + step
            ref_idx = self.past_point
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
            # 가장 작은 distance를 갖는 gap만 return
            return self.gaps[gap_idx]


    def driving(self, scan_data, odom_data):
        """
        :param scan_data: scan data
        :param odom_data: odom data
        :return: steer, speed
        """
        scan_data = self.subCallback_scan(scan_data)
        self.current_position = [odom_data['x'], odom_data['y'], odom_data['theta']]
        self.current_speed = odom_data['linear_vel']
        self.find_desired_wp()

        # obstacle = self.define_obstacles(self.scan_filtered)
        # self.find_nearest_obs(obstacle)

        self.find_gap(scan_data)
        #장애물감지, 추월알고리즘 사용가능한지 판단
        self.obs_dect()

        self.for_find_gap(scan_data)
        if self.ovt==False:
            self.desired_gap = self.find_best_gap(self.desired_wp_rt)
            self.best_point = self.find_best_point(self.desired_gap)
        elif self.wp_index_current>=0 and self.wp_index_current <= 740:
            self.desired_gap = self.find_ovt_gap()
            self.best_point = (self.desired_gap[0]+self.desired_gap[1])/2
            print(self.wp_index_current)
            self.ovt = False
        else:
            self.desired_gap = self.find_best_gap(self.desired_wp_rt)
            self.best_point = self.find_best_point(self.desired_gap)
            self.ovt=False
        # print(self.desired_gap)
        # print(self.best_point)
        print(self.current_speed)
        steer, speed = self.main_drive(self.best_point)
        self.past_point = self.best_point
        return speed, steer
