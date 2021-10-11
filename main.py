from Simsim import Simsim
import numpy as np
import logging

if __name__ == "__main__":
    s = Simsim()
    s.rate = 5
    s.add_tracker('tracker', np.array([200, 100]), sensor_rad=300)
    s.add_tracker('tracker', np.array([100, 100]), sensor_rad=200)
    s.add_tracker('tracker', np.array([500, 380]), sensor_rad=200)
    s.add_tracker('tracker', np.array([700, 580]), sensor_rad=200)
    s.add_target('tgt', np.array([50, 50]))
    s.add_target('tgt', np.array([140, 160]))
    s.add_target('tgt', np.array([180, 500]))
    s.add_target('tgt', np.array([500, 300]))
    s.add_target('tgt', np.array([300, 500]))
    s.add_target('tgt', np.array([630, 500]))
    s.add_edges([[0, 1], [0, 2], [3, 2], [1, 3]])
    # s.add_edges([[0,1]])
    s.run(logging.DEBUG, ground_truth=True)