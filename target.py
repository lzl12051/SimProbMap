from tracker import Robot
import numpy as np


class Target(Robot):
    def __init__(self, name: str, id: int, position: np.array, rate) -> None:
        super().__init__(name, id, position, rate)
