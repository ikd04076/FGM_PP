import numpy as np
import threading
import time
from multiprocessing import shared_memory, Process

class FGM(Process):
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 100 # PREPROCESS_consecutive_SIZE
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees

    def __init__(self, current_position, current_speed, scan_filtered ,transformed_desired_point, steer, lock, sp):
        super().__init__()
        self.cp = current_position
        self.cs = current_speed
        self.sf = scan_filtered
        self.tdp = transformed_desired_point
        self.steer = steer
        self.lock = lock
        self.sp = sp
        self.scan_filtered = [0] * 1080

        self.radians_per_elem = 0.00435


    def find_max_gap(self, free_space_ranges):

        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):

        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):

        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2

        return steering_angle

    def run(self):
        while True:
            if self.sp[0]==1:
                # self.current_position = self.cp
                # self.transformed_desired_point = self.tdp
                # self.scan_filtered = self.sf
                self.current_speed = self.cs[0]
                # for i in range(len(self.cp)):
                #     # self.current_position[i] = self.cp[i]
                #     self.transformed_desired_point[i] = self.tdp[i]
                for i in range(len(self.sf)):
                    self.scan_filtered[i] = self.sf[i]

                self.scan_filtered = np.array(self.scan_filtered)#.reshape((1,1080))
                closest = self.scan_filtered.argmin()

                min_index = closest - self.BUBBLE_RADIUS
                max_index = closest + self.BUBBLE_RADIUS
                if min_index < 0: min_index = 0
                if max_index >= len(self.scan_filtered): max_index = len(self.scan_filtered) - 1
                self.scan_filtered[min_index:max_index] = 0

                gap_start, gap_end = self.find_max_gap(self.scan_filtered)

                best = self.find_best_point(gap_start, gap_end, self.scan_filtered)

                steer = self.get_angle(best, len(self.scan_filtered))
                with self.lock:
                    self.steer[0] = float(steer)

            # time.sleep(0.005)