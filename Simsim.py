import matplotlib.pyplot as plt
import numpy as np
from target import Target
from tracker import Tracker
import logging
from collections import deque


class Simsim:
    class Topics:
        def __init__(self) -> None:
            self.topics = dict()

        def add_topic(self, topic_name):
            self.topics[topic_name] = deque(maxlen=20)

        def del_topic(self, topic_name):
            try:
                del self.topics[topic_name]
            except:
                pass

        def publish(self, topic_name, msg):
            self.topics[topic_name].append(msg)

        def subs(self, topic_name):
            return self.topics[topic_name]

    def __init__(self) -> None:
        self.trackers = []
        self.edges = []
        self.targets = []
        self.map_size = [1000, 1000]
        self.rate = 30
        self.topics = self.Topics()

        plt.ion()
        plt.axis('equal')
        self.fig, (self.plt_sim, self.plt_pm) = plt.subplots(
            1, 2, figsize=(10, 5), dpi=160)

    def add_tracker(self, name, position, sensor_rad):
        tracker = Tracker(self, name, len(self.trackers),
                          position, sensor_rad, self.rate)
        self.trackers.append(tracker)

    def add_edges(self, edges):
        self.edges.extend(edges)
        for e in edges:
            self.trackers[e[0]].neighbor.add(e[1])
            self.trackers[e[1]].neighbor.add(e[0])

    def add_target(self, name, position):
        target = Target(self, name, len(self.targets), position, self.rate)
        self.targets.append(target)

    def _update_all(self):
        """Simulate once, update all trackers and targets
        """
        for tracker in self.trackers:
            tracker.job()
        for target in self.targets:
            target.job()
        pass

    def run(self, log_lvl=logging.WARN, ground_truth=False):
        logging.basicConfig(format='%(asctime)s.%(msecs)03d %(levelname)s: %(message)s',
                            datefmt='%m/%d/%Y %H:%M:%S', level=log_lvl)
        while 1:
            self._update_all()
            self.plt_sim.cla()
            self.plt_pm.cla()
            self.plt_sim.set_xlim(0, self.map_size[0])
            self.plt_sim.set_ylim(0, self.map_size[1])
            self.plt_pm.set_xlim(0, self.map_size[0])
            self.plt_pm.set_ylim(0, self.map_size[1])

            if ground_truth:
                for target in self.targets:
                    self.plt_sim.scatter(
                        target.position[0], target.position[1], marker='x', s=2, color='r')
            for e in self.edges:
                t0 = self.trackers[e[0]]
                t1 = self.trackers[e[1]]
                self.plt_sim.plot([t0.position[0], t1.position[0]],
                                  [t0.position[1], t1.position[1]],
                                  linewidth=1, color='g', alpha=0.5)

            for t in self.trackers:
                # draw the trackers
                self.plt_sim.scatter(t.position[0], t.position[1],
                                     marker='s', s=20, c='b')
                self.plt_sim.annotate(t.id, (t.position[0], t.position[1]+10))
                # draw the camera coverage
                circle = plt.Circle(
                    (t.position), t.sensor.coverage_radius, fill=False, color='grey', alpha=0.3)
                self.plt_sim.add_patch(circle)

                for est in t.target_estimates:
                    det_pos = est[0:2]
                    det_abs_pos = det_pos+t.position
                    self.plt_sim.scatter(det_abs_pos[0], det_abs_pos[1],
                                         marker='^', s=2)
                    for ind in t.prob_map.prob_map:
                        new_ind = np.array(ind) - self.map_size + t.position
                        self.plt_pm.scatter(
                            new_ind[0], new_ind[1], marker='s', s=1, c='r', alpha=t.prob_map.prob_map[ind])
            plt.pause(1/self.rate)
