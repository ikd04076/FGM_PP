import numpy as np
import threading
from .speed_controller import SpeedController as SC


class overtaking_fgm(threading.Thread):
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 100  # PREPROCESS_consecutive_SIZE
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees

    def __init__(self, main_local_q, local_main_q):
        super(overtaking_fgm, self).__init__()
        self.main_local_q = main_local_q
        self.local_main_q = local_main_q
        self.radians_per_elem = 0.00435



    def ttc(self):
        self.obs_x = self.obs[0][5] * np.cos((np.pi / 720) * (self.obs[0][3] - 180))


    def run(self):
        while True:
            data = self.main_local_q.get()
            #self.current_position = [data[0][0], data[0][1], data[0][2]]
            #self.current_speed = data[1]
            self.scan_filtered = data[2]
            #self.transformed_desired_point = data[3]
            self.obs = data[4]

            t_tc = self.ttc()
            #find_gap

            # 장애물 고려 x , 장애물없는 맵에서 차량추월만 하자
            # t_tc 계산
            if self.t_tc<2:
                 if self.x_obs + self.LAR > self.x_ego and self.x_ego > self.x_obs:
                     asd#change lane 이부분에서 이전에 선택한 gapd에 weight값을 추가하는부분이 필요할지도?
                 else:
                     asd#pass
            else:
                asd# keep lane

            #find_best_gap


            # false는 keep lane
            # true는 X_obs + LAR > X_ego & X_ego > X_obs and 추월할 차량의 옆 라이다 길이가 일정 이상
            # true는 change lane, false는 pass
            self.local_main_q.put(steer)