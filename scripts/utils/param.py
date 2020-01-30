import rospkg


class Param:
    def __init__(self):
        self.pkg_name = 'dira'
        self.node_name = 'Team812'
        self.path = rospkg.RosPack().get_path(self.pkg_name)  # absolute path

        self.model_lane_path = self.path + '/scripts/models/u_net_pruned.json'
        self.model_sign_path = self.path + '/scripts/models/sign.h5'
        self.model_lane_weights = self.path + '/scripts/models/lane_weights.h5'
        self.model_obstacle_path = self.path + '/scripts/models/cars.xml'
        self.model_ssd_path = self.path + '/models'

        self.max_speed = 65
        self.base_speed = 55
        self.min_speed = 30

        self.max_steer_angle = 60.0

        self.delay_time = 1.
        self.turning_time = 1.
        self.obstacle_avoid_time = 0.3
        self.time_decay = 0.1
