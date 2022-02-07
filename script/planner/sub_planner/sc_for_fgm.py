import numpy as np


class SpeedController:
    def __init__(self, params):

        self.params = params

        self.MU = params.mu
        self.GRAVITY_ACC = params.g
        self.PI = params.pi
        self.WHEEL_BASE = params.robot_length
        self.SPEED_MAX = params.max_speed
        self.SPEED_MIN = params.min_speed

        self.scan = []
        self.current_speed = 5.0
        self.steering_angle = 0.0
        self.current_idx = 0

        self.braking_a = params.braking_a
        self.braking_b = params.braking_b
        self.sus_a = params.sus_a
        self.sus_b = params.sus_b

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
        if np.fabs(self.steering_angle) > self.PI / 8:
            angular_speed = min_speed
        else:
            angular_speed = float(-(3 / self.PI) * (max_speed - min_speed) * np.fabs(self.steering_angle) + max_speed)

        return angular_speed

    def combined_speed(self):
        current_distance = np.fabs(np.average(self.scan[360:720]))
        direction_speed = 0
        road_direction = np.fabs(self.find_road_direction())

        braking_speed = self.braking_distance()

        if current_distance < 3:
            # print('angle')
            if self.current_speed < 10:
                direction_speed = self.angle_based()
            else:
                direction_speed = self.angle_based(self.current_speed, 7)
        else:
            # print('braking')
            direction_speed = self.speed_suspension(braking_speed)

        return direction_speed

    def speed_suspension(self):
        final_speed = 0
        braking_speed = self.braking_distance()
        if self.current_speed <= braking_speed:
            final_speed = braking_speed
        else:
            # sus_b
            final_speed = self.current_speed - np.fabs((braking_speed - self.current_speed) * 0.2)

        return final_speed

    def routine(self, scan, speed, steer, switch):
        calculated_speed = 0

        self.scan = scan
        self.current_speed = speed
        self.steering_angle = steer

        if switch == 0:
            calculated_speed = self.speed_suspension()
        elif switch == 1:
            # Angle_Based Speed
            calculated_speed = self.angle_based()
        elif switch == 2:
            # Braking_distance + Road Direction based Speed
            calculated_speed = self.combined_speed()

        return calculated_speed
