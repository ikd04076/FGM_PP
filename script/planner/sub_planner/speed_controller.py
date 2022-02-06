import numpy as np

class SpeedController:
    def __init__(self, params):

        self.params = params
        self.mode = self.params.speed_controller

        self.MU = params.mu
        self.GRAVITY_ACC = params.g
        self.PI = params.pi
        self.WHEEL_BASE = params.robot_length
        self.SPEED_MAX = 11
        self.SPEED_MIN = 8

        self.scan = []
        self.current_speed = 5.0
        self.steering_angle = 0.0
        self.current_idx = 0

        self.braking_a = params.braking_a
        self.braking_b = params.braking_b
        self.sus_a = params.sus_a
        self.sus_b = params.sus_b

        self.wps, self.wp_num = self.load_wps()

    def const_speed(self):
        speed_straight = 8
        speed_corner = 4
        straight_steer = np.pi / 18

        if np.abs(self.steering_angle) > straight_steer:
            const_speed = speed_corner
        else:
            const_speed = speed_straight

        return const_speed

    def braking_distance(self):
        current_distance = np.fabs(np.average(self.scan[499:580]))

        if np.isnan(current_distance):
            print("SCAN ERROR")
            current_distance = 1.0

        braking_speed = np.sqrt(2 * self.MU * self.GRAVITY_ACC * np.fabs(current_distance))

        braking_speed *= 1

        if braking_speed >= self.SPEED_MAX:
            braking_speed = self.SPEED_MAX

        return braking_speed

    def angle_based(self, max_speed=8.0, min_speed=4.0):
        if np.fabs(self.steering_angle) > self.PI/8:
            angular_speed = min_speed
        else:
            angular_speed = float(-(3/self.PI)*(max_speed - min_speed)*np.fabs(self.steering_angle)+max_speed)

        return angular_speed

    # Road Direction Based Speed Control
    def load_wps(self):
        wpt_path = self.params.wpt_path
        wpt_delimiter = self.params.wpt_delimeter

        file_wps = np.genfromtxt(wpt_path, delimiter=wpt_delimiter, dtype='float')

        temp_waypoint = []
        wp_num = 0
        for i in file_wps:
            wps_point = [i[0], i[1], 0]
            temp_waypoint.append(wps_point)
            wp_num += 1

        return temp_waypoint, wp_num

    def get_distance(self,origin,target):
        _dx = origin[0] - target[0]
        _dy = origin[1] - target[1]

        _res = np.sqrt(_dx ** 2 + _dy ** 2)

        return _res

    def find_next_target_wp(self):
        look_const = 2.0

        current_idx = self.current_idx
        wp_target = self.current_idx + 1

        temp_distance = 0
        while True:
            if wp_target >= self.wp_num - 1:
                wp_target = 0

            temp_distance = self.get_distance(self.wps[wp_target], self.wps[current_idx])

            if temp_distance > look_const: break
            wp_target += 1

        return wp_target

    def find_road_direction(self):
        current_point = self.wps[self.current_idx]
        next_idx = self.find_next_target_wp()
        target_point = self.wps[next_idx]

        dx = current_point[0] - target_point[0]
        dy = current_point[1] - target_point[1]

        road_direction = np.arctan2(dy, dx)

        return road_direction

    def direction_speed(self):
        current_distance = np.fabs(np.average(self.scan[360:720]))
        direction_speed = 0
        road_direction = np.fabs(self.find_road_direction())

        braking_speed = self.braking_distance()

        if current_distance < 3:
            print('angle')
            if self.current_speed < 10:
                direction_speed = self.angle_based()
            else:
                direction_speed = self.angle_based(self.current_speed,7)
            # direction_speed = self.speed_suspension(angle_speed)
        else:
            print('braking')
            # direction_speed = float(-(3 / self.PI) * (braking_speed - self.current_speed) * np.fabs(road_direction) + braking_speed)
            direction_speed = self.speed_suspension(braking_speed)
            
        
        return direction_speed

    def speed_suspension(self, set_speed):
        final_speed = 0
        if self.current_speed <= set_speed:
            final_speed = self.braking_distance()
        else:
            #sus_b
            final_speed = self.current_speed - np.fabs((set_speed - self.current_speed) * 0.2)

        return final_speed

    def routine(self, scan, speed, steer, idx):
        calculated_speed = 0

        self.scan = scan
        self.current_speed = speed
        self.steering_angle = steer
        self.current_idx = idx

        if self.mode == 0:
            # Const Speed
            calculated_speed = self.const_speed()
        elif self.mode == 1:
            # Braking_Distance_based Speed
            braking_speed = self.braking_distance()
            calculated_speed = self.speed_suspension(braking_speed)
        elif self.mode == 2:
            # Angle_Based Speed
            calculated_speed = self.angle_based()
        elif self.mode == 3:
            # Braking_distance + Road Direction based Speed
            calculated_speed = self.direction_speed()

        return calculated_speed
