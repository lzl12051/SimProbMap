import numpy as np


class Sensor:
    def __init__(self, tracker) -> None:
        self.type = 'Cam'
        self.tracker = tracker
        self.coverage_radius = 150

    def get_detection(self, targets):
        detections = []
        for t in targets:
            if np.linalg.norm(t.position - self.tracker.position) < self.coverage_radius:
                detections.append(t.position+np.random.rand(2)*10-10)
        return detections

        # return [self.tracker.position[0] + np.random.rand()*2*self.coverage_radius-self.coverage_radius,
        #         self.tracker.position[1] + np.random.rand() *
        #         2*self.coverage_radius-self.coverage_radius,
        #         np.random.rand()]


class Robot:
    def __init__(self, name: str, id: int, position: np.array) -> None:
        self.name = name
        self.id = id
        self.position = position
        self.speed = np.array([0.0, 0.0])
        # self.rate = rate

    def run(self, rate):
        self.position = self.speed*(1/rate) + self.position


class Tracker(Robot):
    def __init__(self, name: str, id: int, position: np.array) -> None:
        super().__init__(name, id, position)
        self.sensor = Sensor(self)
        self.neighbor = []
