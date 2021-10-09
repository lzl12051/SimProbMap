import numpy as np
import warnings
import logging


class Sensor:
    def __init__(self, tracker, sensor_rad=150) -> None:
        self.type = 'Cam'
        self.tracker = tracker
        self.coverage_radius = sensor_rad

    def get_detection(self, targets):
        detections = []
        for target in targets:
            if np.linalg.norm(target.position - self.tracker.position) < self.coverage_radius:
                detection = (target.position-self.tracker.position)
                logging.info(
                    f"Ture {self.tracker.name}{self.tracker.id} detection:\n\t\t" + str(detection))
                detection += np.random.normal(loc=0, scale=2, size=2)
                distance = np.linalg.norm(detection)
                if distance > self.coverage_radius:
                    detection = detection * (self.coverage_radius/distance)
                detections.append(detection)
                logging.info(
                    f"Noisy {self.tracker.name}{self.tracker.id} detection:\n\t\t"+str(detection))
        return detections


class Robot:
    def __init__(self, name: str, id: int, position: np.array, rate: int) -> None:
        """The base Robot class

        Parameters
        ----------
        name : str
            Robot's name
        id : int
            Robot's ID
        position : np.array
            Robot's start position
        rate : int
            Simulator's update rate
        """
        self.name = name
        self.id = id
        self.position = position
        self.desierd_position = self.position
        self.in_position = True
        self.speed = 0.0
        self.rate = rate
        self.max_speed = 10.0  # m/s

    def move(self):
        """Move the robot to a position

        Parameters
        ----------
        f : Function, optional
            a funcrtion that returns a position, by default None
        """
        logging.warning("Moving function not impelemented")
        # warnings.warn("moving function not Impelement")
        pass

    def waypoint_ctrl(self, speed, desierd_pos: np.array = None):
        # if got new waypoint, update
        if not np.any(desierd_pos == None):
            self.desierd_position = desierd_pos

        if speed > self.max_speed:
            logging.warning("Given speed is over maximum")
        self.speed = speed

        if np.any(self.position != self.desierd_position):
            one_time_step_length = speed/self.rate
            difference = self.desierd_position - self.position
            distance = np.linalg.norm(difference)
            # if it's close enough to the goal position, make it in position
            if distance <= one_time_step_length:
                self.position = self.desierd_position
            else:
                # move to the goal point at the given speed
                direction = difference/distance
                self.position = self.position + direction*self.speed/self.rate

    def job(self):
        pass
        # self.position = self.speed*(1/self.rate) + self.position


class Tracker(Robot):
    def __init__(self, name: str, id: int, position: np.array, sensor_rad, rate: int) -> None:
        super().__init__(name, id, position, rate)
        self.sensor = Sensor(self, sensor_rad)
        self.neighbor = set()

    def job(self):
        self.waypoint_ctrl(speed=50, desierd_pos=np.array([500, 500]))
