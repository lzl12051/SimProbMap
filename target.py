from tracker import Robot
import numpy as np


class Target(Robot):
    def __init__(self, simulator, name: str, id: int, position: np.array, rate) -> None:
        super().__init__(simulator, name, id, position, rate)
