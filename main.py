from Simsim import Simsim
import numpy as np
import logging

if __name__ == "__main__":
    s = Simsim()
    s.rate = 5
    s.add_tracker('tracker', np.array([500, 300]), sensor_rad=250)
    s.add_tracker('tracker', np.array([300, 500]), sensor_rad=250)
    s.add_tracker('tracker', np.array([700, 500]), sensor_rad=250)
    # s.add_tracker('tracker', np.array([700, 580]), sensor_rad=200)
    # s.add_tracker('tracker', np.array([620, 220]), sensor_rad=450)
    # s.add_target('tgt', np.array([150, 50]))
    s.add_target('tgt', np.array([500, 490]))
    # s.add_target('tgt', np.array([180, 500]))
    s.add_target('tgt', np.array([500, 300]))
    s.add_target('tgt', np.array([300, 500]))
    # s.add_target('tgt', np.array([630, 500]))
    # s.add_edges([[0, 1], [0, 2], [3, 2], [1, 3], [1, 4], [3, 4]])
    s.add_edges([[0, 1], [1, 2], [0, 2]])
    s.run(logging.DEBUG, ground_truth=True)
