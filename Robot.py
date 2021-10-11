import logging

import numpy as np


class Robot:
    def __init__(self, simulator,  name: str, id: int, position: np.array, rate: int) -> None:
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
        # host simulator
        self.simulator = simulator
        self.name = name
        self.id = id
        self.position = position
        self.desierd_position = self.position
        self.in_position = True
        self.speed = 0.0
        self.rate = rate
        self.max_speed = 100.0  # m/s

    def waypoint_ctrl(self, speed=None, desierd_pos: np.array = None):
        # if got new waypoint, update
        if not np.any(desierd_pos == None):
            self.desierd_position = desierd_pos

        if speed == None:
            self.speed = self.max_speed
        else:
            if speed > self.max_speed:
                logging.warning("Given speed is over maximum")
            self.speed = speed

        if np.any(self.position != self.desierd_position):
            self.in_position = False
            one_time_step_length = speed/self.rate
            difference = self.desierd_position - self.position
            distance = np.linalg.norm(difference)
            # if it's close enough to the goal position, make it in position
            if distance <= one_time_step_length:
                self.position = self.desierd_position
                self.in_position = True
            else:
                # move to the goal point at the given speed
                direction = difference/distance
                self.position = self.position + direction*self.speed/self.rate

    def job(self):
        self.waypoint_ctrl()
        pass
