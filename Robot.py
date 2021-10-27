import logging

import numpy as np


class Robot:
    def __init__(self, simulator,  name: str, id: int, position: np.array) -> None:
        """The base Robot class

        Parameters
        ----------
        simulator: Simsim
            Host simulator object
        name : str
            Robot's name
        id : int
            Robot's ID
        position : np.array
            Robot's starting position
        """
        # host simulator
        self.simulator = simulator
        self.name = name
        self.id = id
        self.log_head = f"{self.name}_{self.id}"
        self.position = position
        # facing direction
        self.ang = np.random.random()*360
        self.desierd_position = self.position
        self.in_position = True
        self.speed = 0.0
        self.rate = self.simulator.rate
        self.max_speed = 100.0  # m/s

    def waypoint_ctrl(self, speed=None, desierd_pos: np.array = None):
        # if got new waypoint, update
        if not np.any(desierd_pos == None):
            logging.debug(f"{self.name}{self.id} set new desierd position")
            self.desierd_position = desierd_pos

        if speed is None:
            logging.debug(f"{self.name}_{self.id} set to max speed")
            self.speed = self.max_speed
        else:
            if speed > self.max_speed:
                logging.warning("Given speed is over maximum")
            self.speed = speed

        if np.any(self.position != self.desierd_position):
            logging.debug(
                f"{self.name}_{self.id} moving to {self.desierd_position}")
            self.in_position = False
            one_time_step_length = self.speed/self.rate
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
                # print("MOVED")
        return self.in_position

    def job(self):
        self.waypoint_ctrl()
        pass
