from tracker import Robot
import numpy as np


class Target(Robot):
    def __init__(self, simulator, name: str, id: int, position: np.array) -> None:
        super().__init__(simulator, name, id, position)

    def job(self):
        # return 0
        if self.waypoint_ctrl():
            self.ang = (self.ang + np.deg2rad(15)) % 360
            rot_mat = np.asarray([[np.cos(self.ang), -np.sin(self.ang)],
                                  [np.sin(self.ang), np.cos(self.ang)]])
            self.waypoint_ctrl(speed=20, desierd_pos=(self.position
                                                      - np.dot([0, 10], rot_mat)))
